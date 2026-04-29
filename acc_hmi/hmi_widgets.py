"""
HMI 커스텀 위젯 모음.

PyQt5 기반 시각 컴포넌트만 모은 모듈. **ACC 상태 / CAN / 비즈니스 로직과 무관** —
호출측이 미리 해석한 원시값(숫자/색 문자열/플래그) 을 setter 로 받아 그리고, 사용자
입력을 callback / signal 로 알리는 역할만 한다.

상태 → 색 매핑 같은 해석은 `acc_state` 가 담당하며, `hmi_gui.HmiWindow._refresh` 가
이 모듈로 값을 흘려보낸다.

포함:
  * `_ArcGauge`, `SpeedGauge`, `PwmGauge` — 호 게이지
  * `_VPedalSlider`, `PedalStrip`, `BrakeButtonPanel` — 페달 입력
  * `GapIcon` — 차간 거리 단계 아이콘
  * `YoloPreview` — Fusion annotated 프레임 미리보기 창
"""
import math

from PyQt5.QtWidgets import QWidget, QFrame, QLabel, QPushButton, QVBoxLayout, QSizePolicy
from PyQt5.QtCore import Qt, QTimer, QRectF, QPointF, pyqtSignal
from PyQt5.QtGui import (
    QPainter, QPen, QBrush, QColor, QConicalGradient, QImage, QPixmap,
)

from acc_hmi.acc_state import SPEED_GAUGE_MAX_CMS
from acc_hmi.hmi_style import BG, SURFACE, TEXT, TEXT_SUB, font


# ═══════════════════════════════════════════════════════════════
#  Arc Gauge
# ═══════════════════════════════════════════════════════════════

class _ArcGauge(QWidget):
    """값과 부가 텍스트를 받아 호 형태로 그리는 추상 게이지.

    상태(`AccStatus`) 를 모른다 — 호출측이 `extra_color` 로 미리 해석된 색을 넘긴다.
    """

    max_val: float = 100.0
    unit: str = "%"
    label_fmt: str = "{:.0f}"
    grad_colors: tuple[str, str, str] = ("#1D9E75", "#EF9F27", "#E24B4A")

    def __init__(self, parent=None):
        super().__init__(parent)
        self._value = 0.0
        self._extra_text = ""
        self._extra_color = TEXT_SUB
        self.setMinimumSize(220, 200)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

    def set_value(self, value: float, extra: str = "", extra_color: str = TEXT_SUB):
        self._value = max(0.0, min(value, self.max_val))
        self._extra_text = extra
        self._extra_color = extra_color
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
                p.setFont(font(fs))
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

        p.setPen(QColor(TEXT)); p.setFont(font(fs_main, bold=True))
        mh = fs_main * 1.5
        p.drawText(QRectF(cx - tw / 2, text_y, tw, mh), Qt.AlignCenter, self.label_fmt.format(self._value))

        p.setPen(QColor(TEXT_SUB)); p.setFont(font(fs_unit))
        uh = fs_unit * 1.8
        p.drawText(QRectF(cx - tw / 2, text_y + mh, tw, uh), Qt.AlignCenter, self.unit)

        if self._extra_text:
            fs_ex = max(7, min(12, int(r * 0.09)))
            p.setPen(QColor(self._extra_color))
            p.setFont(font(fs_ex))
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

        lbl = QLabel(label.upper()); lbl.setFont(font(10)); lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {color}; background: transparent; letter-spacing: 2px;")
        layout.addWidget(lbl)

        self._pct = QLabel("0%"); self._pct.setFont(font(15, bold=True))
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

    def reset_to_zero(self):
        """슬라이더/표시값을 0 으로 강제 리셋. valueChanged 시그널 차단(중복 콜백 방지)."""
        self._slider.blockSignals(True)
        try:
            self._slider.setValue(0)
        finally:
            self._slider.blockSignals(False)
        self._pct.setText("0%")
        self._pct.setStyleSheet(f"color: {TEXT_SUB}; background: transparent;")


class BrakeButtonPanel(QFrame):
    """브레이크 입력 버튼. 누르는 동안만 True. SYS018 디지털 ON/OFF."""

    def __init__(self, label: str, color: str, parent=None):
        super().__init__(parent)
        self._color, self._callback = color, None
        self.setMinimumWidth(120)
        self.setStyleSheet("background: transparent; border: none;")
        layout = QVBoxLayout(self)
        layout.setContentsMargins(8, 4, 8, 4); layout.setSpacing(8)

        lbl = QLabel(label.upper()); lbl.setFont(font(10)); lbl.setAlignment(Qt.AlignCenter)
        lbl.setStyleSheet(f"color: {color}; background: transparent; letter-spacing: 2px;")
        layout.addWidget(lbl)

        layout.addStretch()

        self._button = QPushButton("BRAKE")
        self._button.setMinimumHeight(148)
        self._button.setFont(font(14, True))
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
    """가로 배치 GAP 아이콘. [앞차] [■][■][□] [내차].

    `color=None` 이면 inactive (회색). 호출측이 색 결정 (`acc_state.gap_color`).
    """
    _INACTIVE_FILL = QColor("#B0BAC7")

    def __init__(self, parent=None):
        super().__init__(parent)
        self._level = 2
        self._color: QColor | None = None  # None = inactive
        self.setFixedSize(140, 40)

    def set_level(self, level: int, color: str | None = None):
        self._level = max(1, min(3, level))
        self._color = QColor(color) if color else None
        self.update()

    def paintEvent(self, event):
        p = QPainter(self); p.setRenderHint(QPainter.Antialiasing)
        w, h = self.width(), self.height()
        cy, cw, ch = h / 2, 22, 14
        inactive = self._color is None
        accent = self._color if self._color is not None else self._INACTIVE_FILL

        car_l = self._INACTIVE_FILL if inactive else QColor("#888")
        car_r = self._INACTIVE_FILL if inactive else accent
        self._draw_car(p, 4, cy - ch / 2, cw, ch, car_l)
        self._draw_car(p, w - cw - 4, cy - ch / 2, cw, ch, car_r)

        gl, gr = 4 + cw + 6, w - cw - 4 - 6
        bw, bg = (gr - gl - 8) / 3, 4
        for i in range(3):
            if inactive:
                c = QColor("#E1E7EE")
            else:
                c = accent if (i + 1) <= self._level else QColor("#D4DCE5")
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
#  YOLO Debug Preview
# ═══════════════════════════════════════════════════════════════

class YoloPreview(QWidget):
    """Fusion 의 annotated 프레임을 ~15Hz 로 폴링해 표시하는 디버그 창."""

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
