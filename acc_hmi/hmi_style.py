"""
HMI 디자인 토큰 + Qt 스타일 헬퍼.

색상 팔레트, 폰트, 버튼 QSS 빌더 등 **순수 시각 자산**만 모은다.
- ACC 상태 (`AccStatus`) 와 무관 — 상태에 따른 색/라벨 매핑은 `acc_state` 가 소유.
- CAN/비즈니스 로직과 무관.
- `hmi_widgets`, `hmi_gui` 양쪽이 import 한다.
"""
from PyQt5.QtCore import Qt
from PyQt5.QtGui import QFont
from PyQt5.QtWidgets import QPushButton


# ── 색상 ────────────────────────────────────────────────────────
BG = "#F2F5F9"
PANEL = "#FFFFFF"
BORDER = "#D7E0EA"
TEXT = "#1D2733"
TEXT_SUB = "#5F6B7A"
TEXT_FAINT = "#8A95A3"
SURFACE = "#FFFFFF"
SURFACE_ALT = "#EEF3F8"

# 기본 모노스페이스 폰트 패밀리
MONO = "Menlo"


# ── 버튼 QSS ────────────────────────────────────────────────────
_BTN_BASE = (
    f"background: {SURFACE}; color: {TEXT_SUB}; border: 1px solid {BORDER};"
    "border-radius: 10px; padding: 0 28px; letter-spacing: 1px;"
)
_BTN_DISABLED = f"background: {SURFACE_ALT}; color: {TEXT_FAINT}; border-color: {BORDER};"


def font(size: int = 10, bold: bool = False) -> QFont:
    f = QFont(MONO, size)
    if bold:
        f.setWeight(QFont.Bold)
    return f


def btn_style(color: str) -> str:
    return f"""
        QPushButton {{ {_BTN_BASE} }}
        QPushButton:hover {{ border-color: {color}; color: {color}; background: {color}12; }}
        QPushButton:pressed {{ background: {color}22; }}
        QPushButton:disabled {{ {_BTN_DISABLED} }}
    """


def btn_active_style(color: str) -> str:
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


_BTN_HEIGHTS = {"lg": 90, "md": 70, "sm": 52}
_BTN_FONTS = {"lg": 20, "md": 17, "sm": 14}


def make_btn(text: str, color: str = "#1D9E75", size: str = "md") -> QPushButton:
    """라벨/색/크기로 스타일이 적용된 QPushButton 생성. clicked 연결은 호출측에서."""
    btn = QPushButton(text)
    btn.setFixedHeight(_BTN_HEIGHTS.get(size, 70))
    btn.setFont(font(_BTN_FONTS.get(size, 17), bold=True))
    btn.setCursor(Qt.PointingHandCursor)
    btn.setStyleSheet(btn_style(color))
    return btn
