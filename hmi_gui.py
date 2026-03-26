"""
ACC HMI GUI Application – PyQt5
Ref: STK020, STK021, SYS018, SYS019, SYS029, SWR018, SWR019
"""
import math
import random
import signal
import sys

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame, QSizePolicy,
)
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QFont, QFontDatabase, QColor, QPalette, QPainter, QPen, QBrush,
    QConicalGradient,
)

from acc_state import AccState, AccController
from camera_view import CameraView


# ═══════════════════════════════════════════════════════════════
#  Design Tokens
# ═══════════════════════════════════════════════════════════════
BG = "#060606"
PANEL = "#0a0a0a"
BORDER = "#151515"
MONO = "Menlo"

# 패널/게이지용 일반 색상
CLR = {
    AccState.OFF:       "#5F5E5A",
    AccState.STANDBY:   "#BA7517",
    AccState.CRUISING:  "#1D9E75",
    AccState.FOLLOWING: "#378ADD",
    AccState.FAULT:     "#E24B4A",
}
# HUD 오버레이용 밝은 색상
CLR_HUD = {
    AccState.OFF:       "#9E9D99",
    AccState.STANDBY:   "#F0A030",
    AccState.CRUISING:  "#3FFFB0",
    AccState.FOLLOWING: "#60B8FF",
    AccState.FAULT:     "#FF6B6B",
}
STATE_TEXT = {
    AccState.OFF: "OFF", AccState.STANDBY: "STANDBY",
    AccState.CRUISING: "CRUISING", AccState.FOLLOWING: "FOLLOWING",
    AccState.FAULT: "FAULT",
}
DIST_CLR = {1: "#E24B4A", 2: "#EF9F27", 3: "#1D9E75"}

# 버튼 기본 스타일
_BTN_BASE = (
    "background: #121212; color: #777; border: 2px solid #2a2a2a;"
    "border-radius: 10px; padding: 0 28px; letter-spacing: 1px;"
)
_BTN_DISABLED = "background: #0a0a0a; color: #333; border-color: #1a1a1a;"


def _font(size: int = 10, bold: bool = False) -> QFont:
    f = QFont(MONO, size)
    if bold:
        f.setWeight(QFont.Bold)
    return f


def _btn_style(color: str) -> str:
    """기본 버튼 스타일시트 생성."""
    return f"""
        QPushButton {{ {_BTN_BASE} }}
        QPushButton:hover {{ border-color: {color}; color: {color}; background: {color}18; }}
        QPushButton:pressed {{ background: {color}30; }}
        QPushButton:disabled {{ {_BTN_DISABLED} }}
    """


def _btn_active_style(color: str) -> str:
    """활성 상태 버튼 스타일시트."""
    return f"""
        QPushButton {{
            background: {color}18; color: {color};
            border: 2px solid {color}; border-radius: 10px;
            padding: 0 28px; letter-spacing: 1px;
        }}
        QPushButton:hover {{ background: {color}30; }}
        QPushButton:pressed {{ background: {color}40; }}
        QPushButton:disabled {{ {_BTN_DISABLED} }}
    """


# ═══════════════════════════════════════════════════════════════
#  Arc Gauge (공통 베이스)
# ═══════════════════════════════════════════════════════════════

class _ArcGauge(QWidget):
    """아크형 게이지 공통 베이스. 서브클래스에서 max_val, unit, gradient 색상 지정."""

    max_val: float = 100.0
    unit: str = "%"
    label_fmt: str = "{:.0f}"
    grad_colors: tuple[str, str, str] = ("#1D9E75", "#EF9F27", "#E24B4A")

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0.0
        self._state = AccState.OFF
        self._extra_text = ""  # 게이지 아래 추가 텍스트 (예: SET 속도)
        self.setMinimumSize(220, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, value: float, state: AccState = AccState.OFF, extra: str = ""):
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

        # Background arc
        p.setPen(QPen(QColor("#1a1a1a"), arc_w, Qt.SolidLine, Qt.RoundCap))
        p.drawArc(rect, int(v2a(self.max_val) * 16), int(span * 16))

        # Filled arc
        if self._value > 0.001:
            grad = QConicalGradient(cx, cy, sa)
            for i, c in enumerate(self.grad_colors):
                grad.setColorAt(i / max(len(self.grad_colors) - 1, 1), QColor(c))
            p.setPen(QPen(QBrush(grad), arc_w, Qt.SolidLine, Qt.RoundCap))
            p.drawArc(rect, int(sa * 16), int(-(self._value / self.max_val) * span * 16))

        # Ticks
        for i in range(21):
            frac = i / 20
            val = frac * self.max_val
            ang = math.radians(sa - frac * span)
            ca, sa_ = math.cos(ang), math.sin(ang)
            major = (i % 5 == 0)
            inner = r - arc_w * 0.5 - (side * 0.06 if major else side * 0.03)
            outer = r - arc_w * 0.5 - 2

            p.setPen(QPen(
                QColor("#d0d0d0" if self._value >= val else "#333"),
                2 if major else 1, Qt.SolidLine, Qt.RoundCap,
            ))
            p.drawLine(
                QPointF(cx + ca * inner, cy - sa_ * inner),
                QPointF(cx + ca * outer, cy - sa_ * outer),
            )
            if major:
                fs = max(7, min(14, int(r * 0.09)))
                p.setFont(_font(fs))
                p.setPen(QColor("#777"))
                tr = fs * 3
                lx, ly = cx + ca * (inner - side * 0.04), cy - sa_ * (inner - side * 0.04)
                p.drawText(QRectF(lx - tr, ly - tr / 2, tr * 2, tr), Qt.AlignCenter,
                           self._format_tick(val))

        # Needle
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

        # Hub
        p.setPen(QPen(nc, 2)); p.setBrush(QColor("#111"))
        p.drawEllipse(QPointF(cx, cy), 8, 8)
        p.setPen(Qt.NoPen); p.setBrush(nc)
        p.drawEllipse(QPointF(cx, cy), 3, 3)

        # Digital readout
        tw = r * 1.2
        text_y = cy + r * 0.18
        fs_main = max(10, min(36, int(r * 0.28)))
        fs_unit = max(7, min(14, int(r * 0.10)))

        p.setPen(QColor("#fff")); p.setFont(_font(fs_main, bold=True))
        mh = fs_main * 1.5
        p.drawText(QRectF(cx - tw / 2, text_y, tw, mh), Qt.AlignCenter, self.label_fmt.format(self._value))

        p.setPen(QColor("#555")); p.setFont(_font(fs_unit))
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
    max_val = 2.0
    unit = "m/s"
    label_fmt = "{:.2f}"
    grad_colors = ("#1D9E75", "#EF9F27", "#E24B4A")

    def _format_tick(self, val):
        return f"{val:.1f}"


class PwmGauge(_ArcGauge):
    max_val = 100.0
    unit = "% PWM"
    label_fmt = "{:.0f}"
    grad_colors = ("#378ADD", "#EF9F27", "#E24B4A")

    def _format_tick(self, val):
        return f"{int(val)}"


# ═══════════════════════════════════════════════════════════════
#  Custom Vertical Pedal Slider
# ═══════════════════════════════════════════════════════════════

class _VPedalSlider(QWidget):
    """QPainter 세로 슬라이더. 위=100%, 아래=0%."""
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

        p.setPen(Qt.NoPen); p.setBrush(QColor("#1a1a1a"))
        p.drawRoundedRect(QRectF(gx, 0, gw, h), gw / 2, gw / 2)

        if self._value > 0:
            p.setBrush(self._color)
            p.setClipRect(QRectF(gx, hy, gw, h - hy))
            p.drawRoundedRect(QRectF(gx, max(0, h - max(h - hy, gw)), gw, max(h - hy, gw)), gw / 2, gw / 2)
            p.setClipping(False)

        p.setBrush(QColor("#333")); p.setPen(QPen(self._color, 2))
        p.drawEllipse(QPointF(cx, hy), hd / 2, hd / 2)
        p.end()

    def mousePressEvent(self, e):
        if e.button() == Qt.LeftButton: self._dragging = True; self.setValue(self._y_to_val(e.y()))
    def mouseMoveEvent(self, e):
        if self._dragging: self.setValue(self._y_to_val(e.y()))
    def mouseReleaseEvent(self, e): self._dragging = False


class PedalStrip(QFrame):
    """세로 페달 스트립 – 화면 양쪽 가장자리 배치."""
    def __init__(self, label: str, color: str, parent=None):
        super().__init__(parent)
        self._color, self._callback = color, None
        self.setFixedWidth(90)
        self.setStyleSheet(f"background: {PANEL}; border-radius: 10px; border: 1px solid {BORDER};")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 12, 8, 12); layout.setSpacing(6)

        lbl = QLabel(label.upper()); lbl.setFont(_font(10)); lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {color}; background: transparent; letter-spacing: 2px;")
        layout.addWidget(lbl)

        self._pct = QLabel("0%"); self._pct.setFont(_font(16, bold=True))
        self._pct.setAlignment(Qt.AlignCenter)
        self._pct.setStyleSheet("color: #444; background: transparent;")
        layout.addWidget(self._pct)

        self._slider = _VPedalSlider(color)
        self._slider.valueChanged.connect(self._on_change)
        layout.addWidget(self._slider, stretch=1, alignment=Qt.AlignHCenter)

    def set_callback(self, fn): self._callback = fn

    def _on_change(self, val):
        self._pct.setText(f"{val}%")
        self._pct.setStyleSheet(f"color: {self._color if val > 5 else '#444'}; background: transparent;")
        if self._callback: self._callback(float(val))


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
        self._color = QColor("#333" if inactive else DIST_CLR.get(level, "#EF9F27"))
        self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cy, cw, ch = h / 2, 22, 14
        gray = QColor("#333")

        car_l = gray if self._inactive else QColor("#888")
        car_r = gray if self._inactive else self._color
        self._draw_car(p, 4, cy - ch / 2, cw, ch, car_l)
        self._draw_car(p, w - cw - 4, cy - ch / 2, cw, ch, car_r)

        gl, gr = 4 + cw + 6, w - cw - 4 - 6
        bw, bg = (gr - gl - 8) / 3, 4
        for i in range(3):
            if self._inactive:
                c = QColor("#1a1a1a")
            else:
                c = self._color if (i + 1) <= self._level else QColor("#2a2a2a")
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

def _make_btn(text, key, callback, buttons, color="#1D9E75", small=False):
    btn = QPushButton(text)
    btn.setFixedHeight(56 if small else 70)
    btn.setFont(_font(14 if small else 17, bold=True))
    btn.setCursor(Qt.PointingHandCursor)
    btn.setStyleSheet(_btn_style(color))
    btn.clicked.connect(callback)
    buttons[key] = btn
    return btn


class HmiWindow(QMainWindow):

    def __init__(self):
        super().__init__()
        self.ctrl = AccController()
        self.buttons: dict[str, QPushButton] = {}
        self._sim_front_vehicle = False

        self.setWindowTitle("ACC HMI")
        self.setStyleSheet(f"background-color: {BG};")

        central = QWidget()
        self.setCentralWidget(central)
        root = QHBoxLayout(central)
        root.setContentsMargins(8, 8, 8, 8); root.setSpacing(6)

        # Col 1: Brake
        self.brake_strip = PedalStrip("brake", "#E24B4A")
        self.brake_strip.set_callback(lambda v: (self.ctrl.set_brake(v), self._refresh()))
        root.addWidget(self.brake_strip)

        # Col 2: Left buttons
        left = QVBoxLayout(); left.setSpacing(8); left.addStretch()
        for txt, key, cb, clr in [
            ("ACC", "acc_toggle", self.ctrl.toggle_acc, "#1D9E75"),
            ("SET", "set", self.ctrl.press_set, "#1D9E75"),
            ("RES", "res", self.ctrl.press_res, "#378ADD"),
            ("CANCEL", "pause", self.ctrl.press_pause, "#EF9F27"),
        ]:
            left.addWidget(_make_btn(txt, key, lambda _, f=cb: (f(), self._refresh()), self.buttons, clr))
        left.addStretch()
        root.addLayout(left)

        # Col 3: Content
        content = QVBoxLayout(); content.setSpacing(8)

        # Header
        hdr = QHBoxLayout()
        t = QLabel("ACC"); t.setFont(_font(14, True)); t.setStyleSheet("color: #555; background: transparent; letter-spacing: 3px;")
        hdr.addWidget(t)
        s = QLabel("adaptive cruise control · HMI"); s.setFont(_font(8)); s.setStyleSheet("color: #333; background: transparent;")
        hdr.addWidget(s); hdr.addStretch()
        content.addLayout(hdr)

        # Camera + HUD
        self.camera_view = CameraView()
        self._hud = QFrame(self.camera_view)
        self._hud.setStyleSheet("background: rgba(0,0,0,0.80);")
        self._hud.setAttribute(Qt.WA_TransparentForMouseEvents)

        hl = QHBoxLayout(self._hud); hl.setContentsMargins(0, 6, 0, 6); hl.setSpacing(0)
        hl.addStretch()
        hc = QHBoxLayout(); hc.setSpacing(20)

        self._hud_dot = QLabel(); self._hud_dot.setFixedSize(14, 14)
        hc.addWidget(self._hud_dot)
        self._hud_state = QLabel("OFF"); self._hud_state.setFont(_font(22, True))
        hc.addWidget(self._hud_state)
        self._hud_override = QLabel("OVERRIDE"); self._hud_override.setFont(_font(14, True))
        self._hud_override.setStyleSheet("color: #FFD060; background: transparent;"); self._hud_override.hide()
        hc.addWidget(self._hud_override)

        for sep_text in ("│",):
            sep = QLabel(sep_text); sep.setFont(_font(18)); sep.setStyleSheet("color: #666; background: transparent;")
            hc.addWidget(sep)
        self._hud_set = QLabel("SET —"); self._hud_set.setFont(_font(20, True))
        hc.addWidget(self._hud_set)
        sep2 = QLabel("│"); sep2.setFont(_font(18)); sep2.setStyleSheet("color: #666; background: transparent;")
        hc.addWidget(sep2)

        self._hud_gap = GapIcon(); self._hud_gap.setAttribute(Qt.WA_TransparentForMouseEvents)
        hc.addWidget(self._hud_gap)
        self._hud_dist = QLabel(""); self._hud_dist.setFont(_font(18, True))
        hc.addWidget(self._hud_dist)
        self._hud_relspd = QLabel(""); self._hud_relspd.setFont(_font(16, True))
        hc.addWidget(self._hud_relspd)

        hl.addLayout(hc); hl.addStretch()
        content.addWidget(self.camera_view, stretch=4)

        # Gauges
        gr = QHBoxLayout(); gr.setSpacing(8)
        for gauge_cls, attr in [(SpeedGauge, "gauge"), (PwmGauge, "pwm_gauge")]:
            frame = QFrame()
            frame.setStyleSheet(f"background: {PANEL}; border-radius: 10px; border: 1px solid {BORDER};")
            fl = QVBoxLayout(frame); fl.setContentsMargins(4, 4, 4, 4)
            g = gauge_cls(); fl.addWidget(g); setattr(self, attr, g)
            gr.addWidget(frame, stretch=1)
        content.addLayout(gr, stretch=3)

        # SIM row
        sf = QFrame(); sf.setStyleSheet(f"background: {PANEL}; border-radius: 8px; border: 1px solid {BORDER};")
        sl = QHBoxLayout(sf); sl.setContentsMargins(12, 6, 12, 6); sl.setSpacing(8)
        sl.addWidget(self._make_label("SIM", 10, "#444"))
        for txt, cb in [("차량", self._toggle_front), ("+SPD", self._spd_up), ("−SPD", self._spd_dn), ("FAULT", self.ctrl.trigger_fault)]:
            b = QPushButton(txt); b.setFixedHeight(36); b.setFont(_font(11)); b.setCursor(Qt.PointingHandCursor)
            b.setStyleSheet("QPushButton{background:#111;color:#555;border:1px solid #2a2a2a;border-radius:6px;padding:0 14px}"
                            "QPushButton:hover{color:#999;border-color:#444}QPushButton:pressed{background:#222}")
            b.clicked.connect(lambda _, f=cb: (f(), self._refresh())); sl.addWidget(b)
        sl.addStretch(); content.addWidget(sf)
        root.addLayout(content, stretch=1)

        # Col 4: Right buttons
        right = QVBoxLayout(); right.setSpacing(8); right.addStretch()
        right.addWidget(_make_btn("SPD +", "speed_up", lambda: (self.ctrl.press_speed_up(), self._refresh()), self.buttons))
        right.addWidget(_make_btn("SPD −", "speed_down", lambda: (self.ctrl.press_speed_down(), self._refresh()), self.buttons))
        for i in (1, 2, 3):
            right.addWidget(_make_btn(f"D{i}", f"dist_{i}",
                (lambda lv: lambda: (self.ctrl.set_distance_level(lv), self._refresh()))(i),
                self.buttons, "#378ADD", small=True))
        right.addStretch(); root.addLayout(right)

        # Col 5: Accel
        self.throttle_strip = PedalStrip("accel", "#1D9E75")
        self.throttle_strip.set_callback(lambda v: (self.ctrl.set_accel(v), self._refresh()))
        root.addWidget(self.throttle_strip)

        # Connections
        self.camera_view.set_vehicle_callback(self._on_camera)
        self._timer = QTimer(); self._timer.timeout.connect(self._refresh); self._timer.start(50)
        self._refresh()

    @staticmethod
    def _make_label(text, size, color):
        l = QLabel(text); l.setFont(_font(size)); l.setStyleSheet(f"color:{color};background:transparent;letter-spacing:1px;")
        return l

    # ── Events ──

    def keyPressEvent(self, e):
        if e.key() == Qt.Key_Escape:
            self.showNormal() if self.isFullScreen() else self.close()
        else: super().keyPressEvent(e)

    def resizeEvent(self, e):
        super().resizeEvent(e)
        if hasattr(self, '_hud'):
            self._hud.setGeometry(0, 0, self.camera_view.width(), 56)

    def closeEvent(self, e):
        self.camera_view.stop(); super().closeEvent(e)

    # ── Camera ──

    def _on_camera(self, detected, bbox_area):
        if self._sim_front_vehicle:
            return
        if detected and bbox_area > 0:
            dist = max(200, min(5000, int(300_000 / max(bbox_area, 1))))
            self.ctrl.set_front_vehicle(True, dist)
            # TODO: LiDAR 거리 미분 + 자차속도 (SYS013). 현재 더미.
            self.ctrl.data.front_rel_speed_mps = round(random.uniform(-0.3, 0.1), 2)
        else:
            self.ctrl.set_front_vehicle(False, 0)
            self.ctrl.data.front_rel_speed_mps = 0.0

    # ── SIM ──

    def _toggle_front(self):
        self._sim_front_vehicle = not self._sim_front_vehicle
        if self._sim_front_vehicle:
            self.ctrl.set_front_vehicle(True, 800)
            self.ctrl.data.front_rel_speed_mps = -0.15
        else:
            self.ctrl.set_front_vehicle(False, 0)
            self.ctrl.data.front_rel_speed_mps = 0.0

    def _spd_up(self):  self.ctrl.data.current_speed_mps = round(self.ctrl.data.current_speed_mps + 0.1, 2)
    def _spd_dn(self):  self.ctrl.data.current_speed_mps = max(0.0, round(self.ctrl.data.current_speed_mps - 0.1, 2))

    # ── Refresh ──

    def _refresh(self):
        d = self.ctrl.data
        state = d.state
        active = state in (AccState.CRUISING, AccState.FOLLOWING)

        # Gauges
        self.gauge.set_value(d.current_speed_mps, state, f"SET {d.set_speed_mps:.2f} m/s")
        pwm = min(100, max(0, (d.current_speed_mps / max(d.set_speed_mps, 0.01)) * 50)) if active else d.accel_pct
        self.pwm_gauge.set_value(pwm, state)

        # HUD
        sc = CLR_HUD[state]
        self._hud_dot.setStyleSheet(f"background:{sc};border-radius:7px;border:1px solid {sc};")
        self._hud_state.setText(STATE_TEXT[state])
        self._hud_state.setStyleSheet(f"color:{sc};background:transparent;letter-spacing:3px;")

        override = d.brake_pct > 5 or d.accel_pct > 5
        self._hud_override.setVisible(override and state in (AccState.STANDBY, AccState.CRUISING, AccState.FOLLOWING))

        self._hud_set.setText(f"SET {d.set_speed_mps:.2f} m/s" if active or state == AccState.STANDBY else "SET —")
        self._hud_set.setStyleSheet(f"color:{'#3FFFB0' if active else '#888'};background:transparent;")

        self._hud_gap.set_level(d.distance_level, inactive=(state == AccState.OFF))

        if d.front_vehicle_detected:
            dm, rv = d.front_distance_mm / 1000.0, d.front_rel_speed_mps
            fc = "#FF6B6B" if dm < 0.3 else "#F0A030" if dm < 0.6 else "#3FFFB0"
            rc = "#FF6B6B" if rv < -0.05 else "#3FFFB0" if rv > 0.05 else "#888"
            self._hud_dist.setText(f"{dm:.2f}m"); self._hud_dist.setStyleSheet(f"color:{fc};background:transparent;")
            self._hud_relspd.setText(f"{'+' if rv > 0 else ''}{rv:.2f}m/s"); self._hud_relspd.setStyleSheet(f"color:{rc};background:transparent;")
        else:
            self._hud_dist.setText(""); self._hud_relspd.setText("")

        self._hud.setGeometry(0, 0, self.camera_view.width(), 56)
        self._hud.raise_()

        # Buttons
        avail = self.ctrl.get_button_availability()
        for key, btn in self.buttons.items():
            btn.setEnabled(avail.get(key, False))

        # ACC toggle highlight
        if "acc_toggle" in self.buttons:
            btn = self.buttons["acc_toggle"]
            if state == AccState.OFF:
                btn.setText("ACC OFF"); btn.setStyleSheet(_btn_style("#E24B4A"))
            else:
                btn.setText("ACC ON"); btn.setStyleSheet(_btn_active_style(CLR[state]))

        # Distance buttons highlight
        for i in (1, 2, 3):
            if (k := f"dist_{i}") in self.buttons:
                b = self.buttons[k]
                if d.distance_level == i and state != AccState.OFF:
                    b.setStyleSheet(_btn_active_style(DIST_CLR[i]))
                else:
                    b.setStyleSheet(_btn_style("#378ADD"))


# ═══════════════════════════════════════════════════════════════
#  Entry Point
# ═══════════════════════════════════════════════════════════════

def main():
    app = QApplication(sys.argv)
    app.setStyle("Fusion")

    signal.signal(signal.SIGINT, lambda *_: app.quit())
    sig_timer = QTimer(); sig_timer.timeout.connect(lambda: None); sig_timer.start(200)

    pal = QPalette()
    for role, color in [
        (QPalette.Window, BG), (QPalette.WindowText, "#e0e0e0"),
        (QPalette.Base, "#0a0a0a"), (QPalette.Button, "#121212"), (QPalette.ButtonText, "#999"),
    ]:
        pal.setColor(role, QColor(color))
    app.setPalette(pal)

    QFontDatabase.addApplicationFont("JetBrainsMono-Regular.ttf")
    app.setFont(QFont(MONO, 10))

    win = HmiWindow()
    win.showFullScreen()
    sys.exit(app.exec_())


if __name__ == "__main__":
    main()
