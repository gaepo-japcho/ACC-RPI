"""
ACC HMI GUI Application – PyQt5

데이터 소스:
  - acc_can.CanInterface 로부터 `AccInfo`, `VehicleInfo`, `EcuStatus` 를 50ms 주기로 폴링해 렌더링.
  - 버튼 클릭 / 페달 입력은 `send_button_input / send_acc_setting / send_pedal_input` 으로 즉시 전달.
  - 로컬 상태머신은 두지 않는다. (상태 소유자는 acc_can.)

Ref: STK020, STK021, SYS018, SYS019, SYS029, SWR018, SWR019, SWR028
"""
import math
import os
import signal
import sys

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QFont, QFontDatabase, QColor, QPalette, QPainter, QPen, QBrush,
    QConicalGradient, QImage, QPixmap,
)

from interfaces.acc_status import AccStatus
from interfaces.button_input import ButtonInput
from interfaces.pedal_input import PedalInput
from interfaces.acc_setting import AccSetting

from acc_hmi.acc_state import (
    ACTIVE_STATUSES, BUTTON_AVAILABILITY,
    MIN_SET_SPEED_CMS, MAX_SET_SPEED_CMS, SPEED_INCREMENT_CMS,
    SPEED_GAUGE_MAX_CMS, DEFAULT_DISTANCE_LEVEL, status_from_int,
)


# ═══════════════════════════════════════════════════════════════
#  Design Tokens
# ═══════════════════════════════════════════════════════════════
BG = "#F2F5F9"
PANEL = "#FFFFFF"
BORDER = "#D7E0EA"
MONO = "Menlo"
TEXT = "#1D2733"
TEXT_SUB = "#5F6B7A"
TEXT_FAINT = "#8A95A3"
SURFACE = "#FFFFFF"
SURFACE_ALT = "#EEF3F8"

CLR = {
    AccStatus.OFF:       "#5F5E5A",
    AccStatus.STANDBY:   "#BA7517",
    AccStatus.CRUISING:  "#1D9E75",
    AccStatus.FOLLOWING: "#378ADD",
    AccStatus.FAULT:     "#E24B4A",
}
CLR_HUD = {
    AccStatus.OFF:       "#9E9D99",
    AccStatus.STANDBY:   "#F0A030",
    AccStatus.CRUISING:  "#3FFFB0",
    AccStatus.FOLLOWING: "#60B8FF",
    AccStatus.FAULT:     "#FF6B6B",
}
STATE_TEXT = {
    AccStatus.OFF: "OFF", AccStatus.STANDBY: "STANDBY",
    AccStatus.CRUISING: "CRUISING", AccStatus.FOLLOWING: "FOLLOWING",
    AccStatus.FAULT: "FAULT",
}
DIST_CLR = {1: "#E24B4A", 2: "#EF9F27", 3: "#1D9E75"}

_BTN_BASE = (
    f"background: {SURFACE}; color: {TEXT_SUB}; border: 1px solid {BORDER};"
    "border-radius: 10px; padding: 0 28px; letter-spacing: 1px;"
)
_BTN_DISABLED = f"background: {SURFACE_ALT}; color: {TEXT_FAINT}; border-color: {BORDER};"


def _font(size: int = 10, bold: bool = False) -> QFont:
    f = QFont(MONO, size)
    if bold:
        f.setWeight(QFont.Bold)
    return f


def _btn_style(color: str) -> str:
    return f"""
        QPushButton {{ {_BTN_BASE} }}
        QPushButton:hover {{ border-color: {color}; color: {color}; background: {color}12; }}
        QPushButton:pressed {{ background: {color}22; }}
        QPushButton:disabled {{ {_BTN_DISABLED} }}
    """


def _btn_active_style(color: str) -> str:
    return f"""
        QPushButton {{
            background: {color}16; color: {color};
            border: 2px solid {color}; border-radius: 10px;
            padding: 0 28px; letter-spacing: 1px;
        }}
        QPushButton:hover {{ background: {color}24; }}
        QPushButton:pressed {{ background: {color}34; }}
        QPushButton:disabled {{ {_BTN_DISABLED} }}
    """


# ═══════════════════════════════════════════════════════════════
#  Arc Gauge
# ═══════════════════════════════════════════════════════════════

class _ArcGauge(QWidget):
    max_val: float = 100.0
    unit: str = "%"
    label_fmt: str = "{:.0f}"
    grad_colors: tuple[str, str, str] = ("#1D9E75", "#EF9F27", "#E24B4A")

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0.0
        self._state = AccStatus.OFF
        self._extra_text = ""
        self.setMinimumSize(220, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, value: float, state: AccStatus = AccStatus.OFF, extra: str = ""):
        self._value = max(0.0, min(value, self.max_val))
        self._state = state
        self._extra_text = extra
        self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)

        w, h = self.width(), self.height()
        side = min(w, h)
        cx, cy = w / 2, h * 0.52
        r = side * 0.40
        arc_w = side * 0.045
        sa, span = 225, 270

        def v2a(v):
            return sa - (v / self.max_val) * span

        rect = QRectF(cx - r, cy - r, 2 * r, 2 * r)

        p.setPen(QPen(QColor("#D7DEE8"), arc_w, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(rect, int(v2a(self.max_val) * 16), int(span * 16))

        if self._value > 0.001:
            grad = QConicalGradient(cx, cy, sa)
            for i, c in enumerate(self.grad_colors):
                grad.setColorAt(i / max(len(self.grad_colors) - 1, 1), QColor(c))
            p.setPen(QPen(QBrush(grad), arc_w, Qt.SolidLine, Qt.RoundCap))
            p.drawArc(rect, int(sa * 16), int(-(self._value / self.max_val) * span * 16))

        for i in range(21):
            frac = i / 20
            val = frac * self.max_val
            ang = math.radians(sa - frac * span)
            ca, sa_ = math.cos(ang), math.sin(ang)
            major = (i % 5 == 0)
            inner = r - arc_w * 0.5 - (side * 0.06 if major else side * 0.03)
            outer = r - arc_w * 0.5 - 2

            p.setPen(QPen(
                QColor("#8A95A3" if self._value >= val else "#CAD2DC"),
                2 if major else 1, Qt.SolidLine, Qt.RoundCap,
            ))
            p.drawLine(
                QPointF(cx + ca * inner, cy - sa_ * inner),
                QPointF(cx + ca * outer, cy - sa_ * outer),
            )
            if major:
                fs = max(7, min(14, int(r * 0.09)))
                p.setFont(_font(fs))
                p.setPen(QColor(TEXT_SUB))
                tr = fs * 3
                lx, ly = cx + ca * (inner - side * 0.04), cy - sa_ * (inner - side * 0.04)
                p.drawText(QRectF(lx - tr, ly - tr / 2, tr * 2, tr), Qt.AlignCenter,
                           self._format_tick(val))

        na = math.radians(v2a(self._value))
        nlen = r * 0.68
        frac_s = self._value / self.max_val
        nc = QColor(
            self.grad_colors[0] if frac_s < 0.4
            else self.grad_colors[1] if frac_s < 0.75
            else self.grad_colors[2]
        )
        p.setPen(QPen(nc, 3, Qt.SolidLine, Qt.RoundCap))
        p.drawLine(QPointF(cx, cy), QPointF(cx + math.cos(na) * nlen, cy - math.sin(na) * nlen))

        p.setPen(QPen(nc, 2)); p.setBrush(QColor(SURFACE))
        p.drawEllipse(QPointF(cx, cy), 8, 8)
        p.setPen(Qt.NoPen); p.setBrush(nc)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

        tw = r * 1.2
        text_y = cy + r * 0.18
        fs_main = max(10, min(36, int(r * 0.28)))
        fs_unit = max(7, min(14, int(r * 0.10)))

        p.setPen(QColor(TEXT)); p.setFont(_font(fs_main, bold=True))
        mh = fs_main * 1.5
        p.drawText(QRectF(cx - tw / 2, text_y, tw, mh), Qt.AlignCenter, self.label_fmt.format(self._value))

        p.setPen(QColor(TEXT_SUB)); p.setFont(_font(fs_unit))
        uh = fs_unit * 1.8
        p.drawText(QRectF(cx - tw / 2, text_y + mh, tw, uh), Qt.AlignCenter, self.unit)

        if self._extra_text:
            fs_ex = max(7, min(12, int(r * 0.09)))
            p.setPen(QColor(CLR.get(self._state, "#555")))
            p.setFont(_font(fs_ex))
            p.drawText(QRectF(cx - tw / 2, text_y + mh + uh + 2, tw, fs_ex * 1.8),
                        Qt.AlignCenter, self._extra_text)

        p.end()

    def _format_tick(self, val: float) -> str:
        return self.label_fmt.format(val)


class SpeedGauge(_ArcGauge):
    max_val = float(SPEED_GAUGE_MAX_CMS)
    unit = "cm/s"
    label_fmt = "{:.0f}"
    grad_colors = ("#1D9E75", "#EF9F27", "#E24B4A")

    def _format_tick(self, val):
        return f"{int(val)}"


class PwmGauge(_ArcGauge):
    max_val = 100.0
    unit = "% PWM"
    label_fmt = "{:.0f}"
    grad_colors = ("#378ADD", "#EF9F27", "#E24B4A")

    def _format_tick(self, val):
        return f"{int(val)}"


# ═══════════════════════════════════════════════════════════════
#  Pedal Widgets
# ═══════════════════════════════════════════════════════════════

class _VPedalSlider(QWidget):
    """세로 슬라이더. 위=100%, 아래=0%."""
    valueChanged = pyqtSignal(int)
    GROOVE_W, HANDLE_D, MARGIN = 36, 44, 22

    def __init__(self, color: str, parent=None):
        super().__init__(parent)
        self._color = QColor(color)
        self._value = 0
        self._dragging = False
        self.setFixedWidth(self.HANDLE_D + 8)
        self.setMinimumHeight(100)
        self.setCursor(Qt.PointingHandCursor)
        self.setSizePolicy(QSizePolicy.Fixed, QSizePolicy.Expanding)

    def _val_to_y(self, v): return self.MARGIN + (self.height() - 2 * self.MARGIN) * (100 - v) / 100.0
    def _y_to_val(self, y):
        u = self.height() - 2 * self.MARGIN
        return max(0, min(100, round((1 - (y - self.MARGIN) / u) * 100))) if u > 0 else 0

    def setValue(self, v):
        v = max(0, min(100, v))
        if v != self._value:
            self._value = v
            self.valueChanged.emit(v)
            self.update()

    def paintEvent(self, event):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        gw, hd = self.GROOVE_W, self.HANDLE_D
        cx, gx = w / 2, w / 2 - gw / 2
        hy = self._val_to_y(self._value)

        p.setPen(Qt.NoPen); p.setBrush(QColor("#DCE3EB"))
        p.drawRoundedRect(QRectF(gx, 0, gw, h), gw / 2, gw / 2)

        if self._value > 0:
            p.setBrush(self._color)
            p.setClipRect(QRectF(gx, hy, gw, h - hy))
            p.drawRoundedRect(QRectF(gx, max(0, h - max(h - hy, gw)), gw, max(h - hy, gw)), gw / 2, gw / 2)
            p.setClipping(False)

        p.setBrush(QColor(SURFACE)); p.setPen(QPen(self._color, 2))
        p.drawEllipse(QPointF(cx, hy), hd / 2, hd / 2)
        p.end()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton: self._dragging = True; self.setValue(self._y_to_val(e.y()))
    def mouseMoveEvent(self, e):
        if self._dragging: self.setValue(self._y_to_val(e.y()))
    def mouseReleaseEvent(self, e): self._dragging = False


class PedalStrip(QFrame):
    """세로 슬라이더 페달 (액셀용)."""
    def __init__(self, label: str, color: str, parent=None):
        super().__init__(parent)
        self._color, self._callback = color, None
        self.setFixedWidth(92)
        self.setStyleSheet("background: transparent; border: none;")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4); layout.setSpacing(8)

        lbl = QLabel(label.upper()); lbl.setFont(_font(10)); lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {color}; background: transparent; letter-spacing: 2px;")
        layout.addWidget(lbl)

        self._pct = QLabel("0%"); self._pct.setFont(_font(15, bold=True))
        self._pct.setAlignment(Qt.AlignCenter)
        self._pct.setStyleSheet(f"color: {TEXT_SUB}; background: transparent;")
        layout.addWidget(self._pct)

        self._slider = _VPedalSlider(color)
        self._slider.valueChanged.connect(self._on_change)
        layout.addWidget(self._slider, stretch=1, alignment=Qt.AlignHCenter)

    def set_callback(self, fn): self._callback = fn

    def _on_change(self, val):
        self._pct.setText(f"{val}%")
        self._pct.setStyleSheet(f"color: {self._color if val > 5 else TEXT_SUB}; background: transparent;")
        if self._callback: self._callback(int(val))


class BrakeButtonPanel(QFrame):
    """브레이크 입력 버튼. 누르는 동안만 True. SYS018 디지털 ON/OFF."""

    def __init__(self, label: str, color: str, parent=None):
        super().__init__(parent)
        self._color, self._callback = color, None
        self.setMinimumWidth(120)
        self.setStyleSheet("background: transparent; border: none;")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4); layout.setSpacing(8)

        lbl = QLabel(label.upper()); lbl.setFont(_font(10)); lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {color}; background: transparent; letter-spacing: 2px;")
        layout.addWidget(lbl)

        layout.addStretch()

        self._button = QPushButton("BRAKE")
        self._button.setMinimumHeight(148)
        self._button.setFont(_font(14, True))
        self._button.setCursor(Qt.PointingHandCursor)
        self._button.setStyleSheet(self._button_style(False))
        self._button.pressed.connect(lambda: self._set_pressed(True))
        self._button.released.connect(lambda: self._set_pressed(False))
        layout.addWidget(self._button)
        layout.addStretch()

    def set_callback(self, fn):
        self._callback = fn

    def _button_style(self, pressed: bool) -> str:
        fg = "#fff" if pressed else self._color
        bg = f"{self._color}66" if pressed else SURFACE
        border = self._color
        return (
            "QPushButton{"
            f"background:{bg};color:{fg};border:2px solid {border};border-radius:18px;"
            "padding:8px 4px;letter-spacing:1px;"
            "}"
            f"QPushButton:hover{{background:{self._color}22;}}"
            f"QPushButton:pressed{{background:{self._color}77;color:#fff;}}"
        )

    def _set_pressed(self, pressed: bool):
        self._button.setStyleSheet(self._button_style(pressed))
        if self._callback:
            self._callback(pressed)


# ═══════════════════════════════════════════════════════════════
#  GAP Icon
# ═══════════════════════════════════════════════════════════════

class GapIcon(QWidget):
    """가로 배치 GAP 아이콘. [앞차] [■][■][□] [내차]"""
    def __init__(self, parent=None):
        super().__init__(parent)
        self._level = 2
        self._inactive = True
        self._color = QColor("#EF9F27")
        self.setFixedSize(140, 40)

    def set_level(self, level: int, inactive: bool = False):
        self._level = max(1, min(3, level))
        self._inactive = inactive
        self._color = QColor("#B0BAC7" if inactive else DIST_CLR.get(level, "#EF9F27"))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cy, cw, ch = h / 2, 22, 14
        gray = QColor("#B0BAC7")

        car_l = gray if self._inactive else QColor("#888")
        car_r = gray if self._inactive else self._color
        self._draw_car(p, 4, cy - ch / 2, cw, ch, car_l)
        self._draw_car(p, w - cw - 4, cy - ch / 2, cw, ch, car_r)

        gl, gr = 4 + cw + 6, w - cw - 4 - 6
        bw, bg = (gr - gl - 8) / 3, 4
        for i in range(3):
            if self._inactive:
                c = QColor("#E1E7EE")
            else:
                c = self._color if (i + 1) <= self._level else QColor("#D4DCE5")
            p.setPen(Qt.NoPen); p.setBrush(c)
            p.drawRoundedRect(QRectF(gl + i * (bw + bg), cy - 4, bw, 8), 3, 3)
        p.end()

    @staticmethod
    def _draw_car(p, x, y, w, h, color):
        p.setPen(QPen(color, 1.2))
        p.setBrush(QColor(color.red(), color.green(), color.blue(), 50))
        p.drawRoundedRect(QRectF(x, y, w, h), 3, 3)
        p.setPen(Qt.NoPen)
        p.setBrush(QColor(color.red(), color.green(), color.blue(), 80))
        p.drawRoundedRect(QRectF(x + w * 0.35, y + 2, w * 0.4, h * 0.4), 1.5, 1.5)
        p.setBrush(QColor(color.red(), color.green(), color.blue(), 120))
        p.drawEllipse(QPointF(x + 5, y + h), 3, 3)
        p.drawEllipse(QPointF(x + w - 5, y + h), 3, 3)


# ═══════════════════════════════════════════════════════════════
#  Main Window
# ═══════════════════════════════════════════════════════════════

def _make_btn(text, key, callback, buttons, color="#1D9E75", size="md"):
    btn = QPushButton(text)
    _h = {"lg": 90, "md": 70, "sm": 52}
    _fs = {"lg": 20, "md": 17, "sm": 14}
    btn.setFixedHeight(_h.get(size, 70))
    btn.setFont(_font(_fs.get(size, 17), bold=True))
    btn.setCursor(Qt.PointingHandCursor)
    btn.setStyleSheet(_btn_style(color))
    btn.clicked.connect(callback)
    buttons[key] = btn
    return btn


class HmiWindow(QMainWindow):
    """
    HMI 메인 윈도우. `CanInterface` 인스턴스를 주입받아 상태 폴링/입력 송신만 담당.

    폴링 주기: 50ms (SWR018 "HMI 갱신 50ms").
    """

    def __init__(self, can_interface):
        super().__init__()
        self._can = can_interface
        self.buttons: dict[str, QPushButton] = {}

        # 로컬에 유지하는 입력 중간상태 — 운전자 조작 즉시 피드백용
        self._brake_pressed = False
        self._accel_pwm = 0
        # ECU 가 아직 알려주지 않았을 때 send_acc_setting 에서 쓸 로컬 캐시
        self._last_set_speed = 0
        self._last_distance_level = DEFAULT_DISTANCE_LEVEL

        self.setWindowTitle("ACC HMI")
        self.setStyleSheet(f"background-color: {BG};")

        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        # ── Header ──
        hdr = QHBoxLayout()
        t = QLabel("ACC"); t.setFont(_font(14, True))
        t.setStyleSheet(f"color: {TEXT_SUB}; background: transparent; letter-spacing: 3px;")
        hdr.addWidget(t)
        s = QLabel("adaptive cruise control · driver cluster"); s.setFont(_font(8))
        s.setStyleSheet(f"color: {TEXT_FAINT}; background: transparent;")
        hdr.addWidget(s)
        hdr.addStretch()
        self._hud_link = QLabel("● CAN")
        self._hud_link.setFont(_font(9, True))
        self._hud_link.setStyleSheet(f"color: {TEXT_FAINT}; background: transparent;")
        hdr.addWidget(self._hud_link)
        root.addLayout(hdr)

        # ── FAULT Banner ──
        self.fault_banner = QLabel("FAULT")
        self.fault_banner.setFont(_font(11, True))
        self.fault_banner.setAlignment(Qt.AlignCenter)
        self.fault_banner.setStyleSheet(
            "background:#FDECEC;color:#B42318;border:1px solid #F3B7B5;border-radius:10px;padding:10px 14px;"
        )
        self.fault_banner.hide()
        root.addWidget(self.fault_banner)

        # ── Status Panel (상태 / TARGET / GAP) ──
        self.status_panel = QFrame()
        self.status_panel.setStyleSheet(f"background: {PANEL}; border-radius: 12px;")
        self.status_panel.setFixedHeight(96)
        sp = QHBoxLayout(self.status_panel)
        sp.setContentsMargins(24, 12, 24, 12)
        sp.setSpacing(0)

        state_col = QVBoxLayout(); state_col.setSpacing(4)
        state_col.addWidget(self._make_label("STATE", 8, TEXT_FAINT))
        state_row = QHBoxLayout(); state_row.setSpacing(8)
        self._hud_dot = QLabel(); self._hud_dot.setFixedSize(10, 10)
        state_row.addWidget(self._hud_dot, alignment=Qt.AlignVCenter)
        self._hud_state = QLabel("OFF"); self._hud_state.setFont(_font(22, True))
        state_row.addWidget(self._hud_state)
        self._hud_override = QLabel("OVERRIDE"); self._hud_override.setFont(_font(10, True))
        self._hud_override.setStyleSheet("color: #C98A00; background: transparent;")
        self._hud_override.hide()
        state_row.addWidget(self._hud_override)
        state_row.addStretch()
        state_col.addLayout(state_row)
        sp.addLayout(state_col, stretch=3)

        sp.addSpacing(20); sp.addWidget(self._vline()); sp.addSpacing(20)

        tgt_col = QVBoxLayout(); tgt_col.setSpacing(4)
        tgt_col.addWidget(self._make_label("TARGET", 8, TEXT_FAINT))
        self._hud_set = QLabel("— cm/s"); self._hud_set.setFont(_font(22, True))
        tgt_col.addWidget(self._hud_set)
        sp.addLayout(tgt_col, stretch=2)

        sp.addSpacing(20); sp.addWidget(self._vline()); sp.addSpacing(20)

        gap_col = QVBoxLayout(); gap_col.setSpacing(4)
        gap_col.addWidget(self._make_label("GAP", 8, TEXT_FAINT))
        gap_row = QHBoxLayout(); gap_row.setSpacing(8)
        self._hud_gap = GapIcon(); self._hud_gap.setAttribute(Qt.WA_TransparentForMouseEvents)
        gap_row.addWidget(self._hud_gap)
        gap_row.addStretch()
        gap_col.addLayout(gap_row)
        sp.addLayout(gap_col, stretch=3)
        root.addWidget(self.status_panel)

        # ── Gauges ──
        gr = QHBoxLayout(); gr.setSpacing(8)
        for gauge_cls, attr in [(SpeedGauge, "gauge"), (PwmGauge, "pwm_gauge")]:
            frame = QFrame()
            frame.setStyleSheet(f"background: {PANEL}; border-radius: 12px; border: 1px solid {BORDER};")
            fl = QVBoxLayout(frame); fl.setContentsMargins(4, 4, 4, 4)
            g = gauge_cls(); fl.addWidget(g); setattr(self, attr, g)
            gr.addWidget(frame, stretch=1)
        root.addLayout(gr, stretch=1)

        # ── Button Grid ──
        bottom = QHBoxLayout(); bottom.setSpacing(10)

        prim = QVBoxLayout(); prim.setSpacing(6)
        prim.addWidget(self._make_label("PRIMARY", 8, TEXT_FAINT))
        prim.addWidget(_make_btn("ACC", "acc_toggle",
            self._on_acc_toggle, self.buttons, "#1D9E75", size="lg"))
        prim.addWidget(_make_btn("CANCEL", "pause",
            self._on_cancel, self.buttons, "#EF9F27", size="md"))
        bottom.addLayout(prim, stretch=2)
        bottom.addWidget(self._vline())

        spd_zone = QVBoxLayout(); spd_zone.setSpacing(6)
        spd_zone.addWidget(self._make_label("SPEED", 8, TEXT_FAINT))
        spd_r1 = QHBoxLayout(); spd_r1.setSpacing(6)
        spd_r1.addWidget(_make_btn("SPD +", "speed_up",
            self._on_speed_up, self.buttons, "#1D9E75", size="md"))
        spd_r1.addWidget(_make_btn("SPD -", "speed_down",
            self._on_speed_down, self.buttons, "#1D9E75", size="md"))
        spd_zone.addLayout(spd_r1)
        spd_r2 = QHBoxLayout(); spd_r2.setSpacing(6)
        spd_r2.addWidget(_make_btn("SET", "set",
            self._on_set, self.buttons, "#1D9E75", size="md"))
        spd_r2.addWidget(_make_btn("RES", "res",
            self._on_res, self.buttons, "#378ADD", size="md"))
        spd_zone.addLayout(spd_r2)
        bottom.addLayout(spd_zone, stretch=3)
        bottom.addWidget(self._vline())

        dist_zone = QVBoxLayout(); dist_zone.setSpacing(6)
        dist_zone.addWidget(self._make_label("GAP", 8, TEXT_FAINT))
        for i in (1, 2, 3):
            dist_zone.addWidget(_make_btn(
                f"D{i}", f"dist_{i}",
                (lambda lv: lambda: self._on_distance(lv))(i),
                self.buttons, "#378ADD", size="sm",
            ))
        bottom.addLayout(dist_zone, stretch=1)
        bottom.addWidget(self._vline())

        pedal_zone = QVBoxLayout(); pedal_zone.setSpacing(6)
        pedal_zone.addWidget(self._make_label("PEDAL", 8, TEXT_FAINT))
        pedal_inner = QHBoxLayout(); pedal_inner.setSpacing(6)
        self.brake_strip = BrakeButtonPanel("brake", "#E24B4A")
        self.brake_strip.set_callback(self._on_brake)
        pedal_inner.addWidget(self.brake_strip, stretch=1)
        self.throttle_strip = PedalStrip("accel", "#1D9E75")
        self.throttle_strip.set_callback(self._on_accel)
        pedal_inner.addWidget(self.throttle_strip, stretch=1)
        pedal_zone.addLayout(pedal_inner, stretch=1)
        bottom.addLayout(pedal_zone, stretch=2)

        root.addLayout(bottom)

        # ── Refresh Timer (50ms, SWR018) ──
        self._timer = QTimer(); self._timer.timeout.connect(self._refresh); self._timer.start(50)
        self._refresh()

    # ── small helpers ──
    @staticmethod
    def _make_label(text, size, color):
        l = QLabel(text); l.setFont(_font(size))
        l.setStyleSheet(f"color:{color};background:transparent;letter-spacing:1px;")
        return l

    def _vline(self) -> QFrame:
        v = QFrame(); v.setFrameShape(QFrame.VLine)
        v.setStyleSheet(f"background: {BORDER};"); v.setFixedWidth(1)
        return v

    # ── Input → CanInterface ──

    def _on_acc_toggle(self):
        # OFF→ON 전이일 때만 ego 캡처해 SET_ACC_SPD 에 실어 보냄 (FSM T4b 가 SetSpeedReq 캡처).
        # ON→OFF 는 set_speed 무관 — 버튼만 송신.
        if status_from_int(self._can.get_acc_info().status) == AccStatus.OFF:
            self._capture_ego_setpoint()
        self._can.send_button_input(ButtonInput(
            btn_acc_off=True, btn_acc_set=None, btn_acc_res=None, btn_acc_cancel=None,
        ))

    def _on_cancel(self):
        self._can.send_button_input(ButtonInput(
            btn_acc_off=None, btn_acc_set=None, btn_acc_res=None, btn_acc_cancel=True,
        ))

    def _on_set(self):
        # STANDBY→CRUISING/FOLLOWING (T6/T7) — RPi 가 ego 를 SetSpeedReq 에 실어 보내야 ECU 가 캡처.
        self._capture_ego_setpoint()
        self._can.send_button_input(ButtonInput(
            btn_acc_off=None, btn_acc_set=True, btn_acc_res=None, btn_acc_cancel=None,
        ))

    def _capture_ego_setpoint(self) -> None:
        """현재 차속을 [MIN, MAX] 로 클램프해 다음 ACC_CTRL.SET_ACC_SPD 페이로드에 실어 보낸다.
        ECU FSM 의 T4b/T6/T7 진입 캡처 정책 (AccStateMachine.c v1.2) 과 1:1 대응."""
        ego = int(round(self._can.get_vehicle_info().current_speed))
        ego_clamped = max(MIN_SET_SPEED_CMS, min(MAX_SET_SPEED_CMS, ego))
        self._can.send_acc_setting(AccSetting(
            set_speed=ego_clamped,
            distance_level=self._last_distance_level,
        ))
        self._last_set_speed = ego_clamped

    def _on_res(self):
        self._can.send_button_input(ButtonInput(
            btn_acc_off=None, btn_acc_set=None, btn_acc_res=True, btn_acc_cancel=None,
        ))

    def _on_speed_up(self):
        new_spd = min(self._last_set_speed + SPEED_INCREMENT_CMS, MAX_SET_SPEED_CMS)
        self._can.send_acc_setting(AccSetting(
            set_speed=new_spd, distance_level=self._last_distance_level,
        ))
        self._last_set_speed = new_spd

    def _on_speed_down(self):
        new_spd = max(self._last_set_speed - SPEED_INCREMENT_CMS, MIN_SET_SPEED_CMS)
        self._can.send_acc_setting(AccSetting(
            set_speed=new_spd, distance_level=self._last_distance_level,
        ))
        self._last_set_speed = new_spd

    def _on_distance(self, level: int):
        lv = max(1, min(3, level))
        self._can.send_acc_setting(AccSetting(
            set_speed=self._last_set_speed, distance_level=lv,
        ))
        self._last_distance_level = lv

    def _on_brake(self, pressed: bool):
        self._brake_pressed = bool(pressed)
        self._can.send_pedal_input(PedalInput(
            brake=self._brake_pressed, accel_pwm=self._accel_pwm,
        ))

    def _on_accel(self, pwm: int):
        self._accel_pwm = max(0, min(100, int(pwm)))
        self._can.send_pedal_input(PedalInput(
            brake=self._brake_pressed, accel_pwm=self._accel_pwm,
        ))

    # ── Events ──

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.showNormal() if self.isFullScreen() else self.close()
        else:
            super().keyPressEvent(e)

    # ── Poll + Render ──

    def _refresh(self):
        acc_info = self._can.get_acc_info()
        veh_info = self._can.get_vehicle_info()
        ecu = self._can.get_ecu_status()

        status = status_from_int(acc_info.status)
        # ECU 피드백을 따라가도록 로컬 캐시 갱신
        self._last_set_speed = int(acc_info.set_speed)
        self._last_distance_level = int(acc_info.distance_level)

        active = status in ACTIVE_STATUSES
        current_speed = float(veh_info.current_speed)
        set_speed = int(acc_info.set_speed)

        # ── Gauges ──
        self.gauge.set_value(current_speed, status, f"SET {set_speed} cm/s" if active or status == AccStatus.STANDBY else "")
        # PWM 게이지: 액티브 상태에서는 현재/목표 비율을 근사치로, 그 외에는 운전자 액셀 입력
        if active and set_speed > 0:
            pwm_pct = min(100.0, max(0.0, current_speed / set_speed * 50.0))
        else:
            pwm_pct = float(self._accel_pwm)
        self.pwm_gauge.set_value(pwm_pct, status)

        # ── HUD ──
        sc = CLR_HUD[status]
        self._hud_dot.setStyleSheet(f"background:{sc};border-radius:7px;border:1px solid {sc};")
        self._hud_state.setText(STATE_TEXT[status])
        self._hud_state.setStyleSheet(f"color:{sc};background:transparent;letter-spacing:3px;")

        override = self._brake_pressed or self._accel_pwm > 5
        self._hud_override.setVisible(
            override and status in (AccStatus.STANDBY, AccStatus.CRUISING, AccStatus.FOLLOWING)
        )

        if active or status == AccStatus.STANDBY:
            self._hud_set.setText(f"SET {set_speed} cm/s")
            self._hud_set.setStyleSheet(f"color:{CLR[status] if active else TEXT_FAINT};background:transparent;")
        else:
            self._hud_set.setText("SET —")
            self._hud_set.setStyleSheet(f"color:{TEXT_FAINT};background:transparent;")

        self._hud_gap.set_level(int(acc_info.distance_level), inactive=(status == AccStatus.OFF))

        # ── FAULT 배너 (SWR019) ──
        # FAULT 진입 또는 실제 CAN 연결 중 HB lost/에러코드 수신 시 표시.
        # 시뮬 모드 (`is_connected() == False`) 에서는 HB 체크를 건너뛴다.
        fault_banner_text = self._fault_text(status, ecu, self._can.is_connected())
        if fault_banner_text:
            self.fault_banner.setText(fault_banner_text)
            self.fault_banner.show()
        else:
            self.fault_banner.hide()

        # ── CAN link 상태 ──
        if self._can.is_connected():
            link_color = "#1D9E75" if ecu.heartbeat_ok else "#EF9F27"
            link_text = "● CAN" if ecu.heartbeat_ok else "● CAN · no HB"
        else:
            link_color = "#8A95A3"
            link_text = "○ CAN sim"
        self._hud_link.setText(link_text)
        self._hud_link.setStyleSheet(f"color:{link_color};background:transparent;")

        # ── Buttons ──
        avail = BUTTON_AVAILABILITY[status]
        for key, btn in self.buttons.items():
            btn.setEnabled(avail.get(key, False))

        # ACC toggle highlight
        if "acc_toggle" in self.buttons:
            btn = self.buttons["acc_toggle"]
            if status == AccStatus.OFF:
                btn.setText("ACC OFF"); btn.setStyleSheet(_btn_style("#E24B4A"))
            else:
                btn.setText("ACC ON"); btn.setStyleSheet(_btn_active_style(CLR[status]))

        # Distance buttons highlight
        for i in (1, 2, 3):
            if (k := f"dist_{i}") in self.buttons:
                b = self.buttons[k]
                if int(acc_info.distance_level) == i and status != AccStatus.OFF:
                    b.setStyleSheet(_btn_active_style(DIST_CLR[i]))
                else:
                    b.setStyleSheet(_btn_style("#378ADD"))

    @staticmethod
    def _fault_text(status: AccStatus, ecu, can_connected: bool) -> str:
        """FAULT 배너에 표시할 문구. 없으면 빈 문자열."""
        if status == AccStatus.FAULT:
            if ecu.error_code:
                return f"FAULT  |  ECU error 0x{ecu.error_code:02X}  |  press ACC to clear"
            return "FAULT  |  press ACC to clear"
        if not can_connected:
            return ""
        if not ecu.heartbeat_ok:
            return "ECU HEARTBEAT LOST  |  checking CAN link…"
        if ecu.error_code:
            return f"ECU error 0x{ecu.error_code:02X}"
        return ""


# ═══════════════════════════════════════════════════════════════
#  YOLO Debug Preview (PyQt5)
# ═══════════════════════════════════════════════════════════════

class YoloPreview(QWidget):
    """Fusion 의 annotated 프레임을 30Hz 로 폴링해 표시하는 디버그 창."""

    def __init__(self, fusion):
        super().__init__()
        self._fusion = fusion
        self.setWindowTitle("YOLO Debug")
        self.setStyleSheet(f"background-color: {BG};")
        self._label = QLabel("waiting for frame…")
        self._label.setAlignment(Qt.AlignCenter)
        self._label.setStyleSheet(f"color: {TEXT};")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self._label)

        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(66)  # ~15Hz — 메인 스레드 부하 ↓ (버튼 이벤트 우선순위)

    def _refresh(self) -> None:
        frame = self._fusion.get_annotated_frame()
        if frame is None:
            return
        # Picamera2 RGB888 = QImage Format_RGB888 그대로 사용 가능
        h, w, _ = frame.shape
        img = QImage(frame.data, w, h, w * 3, QImage.Format_RGB888)
        # QImage 가 numpy 버퍼를 참조하므로 copy() 로 소유권 이전
        self._label.setPixmap(QPixmap.fromImage(img.copy()))


# ═══════════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════════

def gui_main(can_interface, fusion=None):
    """QApplication 을 띄우고 HMI 윈도우를 주입된 `can_interface` 로 실행."""
    # opencv-python 휠은 import 시 QT_QPA_PLATFORM_PLUGIN_PATH 를 자기 디렉토리(cv2/qt/plugins)로
    # 덮어쓴다. 거기 들어있는 vendored Qt5 는 시스템 PyQt5 와 ABI 가 안 맞아 xcb 로드가 실패하므로,
    # QApplication 생성 직전에 시스템 Qt5 플러그인 경로로 되돌려 놓는다.
    os.environ["QT_QPA_PLATFORM_PLUGIN_PATH"] = "/usr/lib/aarch64-linux-gnu/qt5/plugins"

    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sig_timer = QTimer(); sig_timer.timeout.connect(lambda: None); sig_timer.start(200)

    pal = QPalette()
    for role, color in [
        (QPalette.Window, BG), (QPalette.WindowText, TEXT),
        (QPalette.Base, SURFACE), (QPalette.Button, SURFACE), (QPalette.ButtonText, TEXT),
    ]:
        pal.setColor(role, QColor(color))
    app.setPalette(pal)

    QFontDatabase.addApplicationFont("JetBrainsMono-Regular.ttf")
    app.setFont(QFont(MONO, 10))

    win = HmiWindow(can_interface)
    win.showFullScreen()

    preview = None
    if fusion is not None and fusion.show_window:
        preview = YoloPreview(fusion)
        preview.resize(640, 480)
        preview.show()

    return app.exec_()
