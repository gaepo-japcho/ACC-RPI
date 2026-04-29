"""
ACC HMI GUI Application — PyQt5

데이터 소스:
  - `acc_can.CanInterface` 로부터 `AccInfo`, `VehicleInfo`, `EcuStatus` 를 50ms 주기로 폴링해 렌더링.
  - 버튼 클릭 / 페달 입력은 `send_button_input / send_acc_setting / send_pedal_input` 으로 즉시 전달.
  - 로컬 상태머신은 두지 않는다. (상태 소유자는 acc_can.)

본 모듈의 역할은 **오케스트레이션**:
  - 시각 자산 → `hmi_style`
  - 위젯 클래스 → `hmi_widgets`
  - 상태 → 표시값 해석 → `acc_state`

이 파일은 윈도우 레이아웃 조립, 입력 콜백 → CAN 송신, 폴링 결과를 위젯에 푸시하는 일만 한다.

Ref: STK020, STK021, SYS018, SYS019, SYS029, SWR018, SWR019, SWR028
"""
import os
import signal
import sys

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QPushButton, QLabel, QFrame,
)
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QFontDatabase, QPalette

from interfaces.acc_status import AccStatus
from interfaces.acc_setting import AccSetting
from interfaces.button_input import ButtonInput
from interfaces.pedal_input import PedalInput

from acc_hmi.acc_state import (
    BUTTON_AVAILABILITY,
    CLR, DIST_CLR, STATE_TEXT, CLR_HUD,
    DEFAULT_DISTANCE_LEVEL,
    MAX_SET_SPEED_CMS, MIN_SET_SPEED_CMS, SPEED_INCREMENT_CMS,
    fault_banner_text, gap_color, is_active, is_override_visible,
    link_indicator, pwm_display_pct, set_speed_label, status_from_int,
)
from acc_hmi.hmi_style import (
    BG, BORDER, MONO, PANEL, SURFACE, TEXT, TEXT_FAINT,
    btn_active_style, btn_style, font, make_btn,
)
from acc_hmi.hmi_widgets import (
    BrakeButtonPanel, GapIcon, PedalStrip, PwmGauge, SpeedGauge, YoloPreview,
)


class HmiWindow(QMainWindow):
    """HMI 메인 윈도우. `CanInterface` 인스턴스를 주입받아 상태 폴링/입력 송신만 담당.

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

        self._build_layout()

        # ── Refresh Timer (50ms, SWR018) ──
        self._timer = QTimer(); self._timer.timeout.connect(self._refresh); self._timer.start(50)
        self._refresh()

    # ════════════════════════════════════════════════════════════
    #  Layout
    # ════════════════════════════════════════════════════════════

    def _build_layout(self):
        central = QWidget()
        self.setCentralWidget(central)
        root = QVBoxLayout(central)
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(8)

        root.addLayout(self._build_header())
        root.addWidget(self._build_fault_banner())
        root.addWidget(self._build_status_panel())
        root.addLayout(self._build_gauges(), stretch=1)
        root.addLayout(self._build_button_grid())

    def _build_header(self) -> QHBoxLayout:
        hdr = QHBoxLayout()
        t = QLabel("ACC"); t.setFont(font(14, True))
        t.setStyleSheet(f"color: {TEXT}; background: transparent; letter-spacing: 3px;")
        hdr.addWidget(t)
        s = QLabel("adaptive cruise control · driver cluster"); s.setFont(font(8))
        s.setStyleSheet(f"color: {TEXT_FAINT}; background: transparent;")
        hdr.addWidget(s)
        hdr.addStretch()
        self._hud_link = QLabel("● CAN")
        self._hud_link.setFont(font(9, True))
        self._hud_link.setStyleSheet(f"color: {TEXT_FAINT}; background: transparent;")
        hdr.addWidget(self._hud_link)
        return hdr

    def _build_fault_banner(self) -> QLabel:
        self.fault_banner = QLabel("FAULT")
        self.fault_banner.setFont(font(11, True))
        self.fault_banner.setAlignment(Qt.AlignCenter)
        self.fault_banner.setStyleSheet(
            "background:#FDECEC;color:#B42318;border:1px solid #F3B7B5;border-radius:10px;padding:10px 14px;"
        )
        self.fault_banner.hide()
        return self.fault_banner

    def _build_status_panel(self) -> QFrame:
        panel = QFrame()
        panel.setStyleSheet(f"background: {PANEL}; border-radius: 12px;")
        panel.setFixedHeight(96)
        sp = QHBoxLayout(panel)
        sp.setContentsMargins(24, 12, 24, 12)
        sp.setSpacing(0)

        # STATE
        state_col = QVBoxLayout(); state_col.setSpacing(4)
        state_col.addWidget(self._make_label("STATE", 8, TEXT_FAINT))
        state_row = QHBoxLayout(); state_row.setSpacing(8)
        self._hud_dot = QLabel(); self._hud_dot.setFixedSize(10, 10)
        state_row.addWidget(self._hud_dot, alignment=Qt.AlignVCenter)
        self._hud_state = QLabel("OFF"); self._hud_state.setFont(font(22, True))
        state_row.addWidget(self._hud_state)
        self._hud_override = QLabel("OVERRIDE"); self._hud_override.setFont(font(10, True))
        self._hud_override.setStyleSheet("color: #C98A00; background: transparent;")
        self._hud_override.hide()
        state_row.addWidget(self._hud_override)
        state_row.addStretch()
        state_col.addLayout(state_row)
        sp.addLayout(state_col, stretch=3)

        sp.addSpacing(20); sp.addWidget(self._vline()); sp.addSpacing(20)

        # TARGET
        tgt_col = QVBoxLayout(); tgt_col.setSpacing(4)
        tgt_col.addWidget(self._make_label("TARGET", 8, TEXT_FAINT))
        self._hud_set = QLabel("— cm/s"); self._hud_set.setFont(font(22, True))
        tgt_col.addWidget(self._hud_set)
        sp.addLayout(tgt_col, stretch=2)

        sp.addSpacing(20); sp.addWidget(self._vline()); sp.addSpacing(20)

        # GAP
        gap_col = QVBoxLayout(); gap_col.setSpacing(4)
        gap_col.addWidget(self._make_label("GAP", 8, TEXT_FAINT))
        gap_row = QHBoxLayout(); gap_row.setSpacing(8)
        self._hud_gap = GapIcon(); self._hud_gap.setAttribute(Qt.WA_TransparentForMouseEvents)
        gap_row.addWidget(self._hud_gap)
        gap_row.addStretch()
        gap_col.addLayout(gap_row)
        sp.addLayout(gap_col, stretch=3)

        self.status_panel = panel
        return panel

    def _build_gauges(self) -> QHBoxLayout:
        gr = QHBoxLayout(); gr.setSpacing(8)
        for gauge_cls, attr in [(SpeedGauge, "gauge"), (PwmGauge, "pwm_gauge")]:
            frame = QFrame()
            frame.setStyleSheet(f"background: {PANEL}; border-radius: 12px; border: 1px solid {BORDER};")
            fl = QVBoxLayout(frame); fl.setContentsMargins(4, 4, 4, 4)
            g = gauge_cls(); fl.addWidget(g); setattr(self, attr, g)
            gr.addWidget(frame, stretch=1)
        return gr

    def _build_button_grid(self) -> QHBoxLayout:
        bottom = QHBoxLayout(); bottom.setSpacing(10)

        # PRIMARY — ACC / CANCEL
        prim = QVBoxLayout(); prim.setSpacing(6)
        prim.addWidget(self._make_label("PRIMARY", 8, TEXT_FAINT))
        prim.addWidget(self._register_btn("acc_toggle", "ACC", self._on_acc_toggle, "#1D9E75", "lg"))
        prim.addWidget(self._register_btn("pause", "CANCEL", self._on_cancel, "#EF9F27", "md"))
        bottom.addLayout(prim, stretch=2)
        bottom.addWidget(self._vline())

        # SPEED — SPD+/-, SET, RES
        spd_zone = QVBoxLayout(); spd_zone.setSpacing(6)
        spd_zone.addWidget(self._make_label("SPEED", 8, TEXT_FAINT))
        spd_r1 = QHBoxLayout(); spd_r1.setSpacing(6)
        spd_r1.addWidget(self._register_btn("speed_up", "SPD +", self._on_speed_up, "#1D9E75", "md"))
        spd_r1.addWidget(self._register_btn("speed_down", "SPD -", self._on_speed_down, "#1D9E75", "md"))
        spd_zone.addLayout(spd_r1)
        spd_r2 = QHBoxLayout(); spd_r2.setSpacing(6)
        spd_r2.addWidget(self._register_btn("set", "SET", self._on_set, "#1D9E75", "md"))
        spd_r2.addWidget(self._register_btn("res", "RES", self._on_res, "#378ADD", "md"))
        spd_zone.addLayout(spd_r2)
        bottom.addLayout(spd_zone, stretch=3)
        bottom.addWidget(self._vline())

        # GAP — D1/D2/D3
        dist_zone = QVBoxLayout(); dist_zone.setSpacing(6)
        dist_zone.addWidget(self._make_label("GAP", 8, TEXT_FAINT))
        for i in (1, 2, 3):
            dist_zone.addWidget(self._register_btn(
                f"dist_{i}", f"D{i}",
                (lambda lv: lambda: self._on_distance(lv))(i),
                "#378ADD", "sm",
            ))
        bottom.addLayout(dist_zone, stretch=1)
        bottom.addWidget(self._vline())

        # PEDAL — Brake / Accel
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

        return bottom

    def _register_btn(self, key: str, text: str, callback, color: str, size: str) -> QPushButton:
        btn = make_btn(text, color=color, size=size)
        btn.clicked.connect(callback)
        self.buttons[key] = btn
        return btn

    @staticmethod
    def _make_label(text, size, color):
        l = QLabel(text); l.setFont(font(size))
        l.setStyleSheet(f"color:{color};background:transparent;letter-spacing:1px;")
        return l

    def _vline(self) -> QFrame:
        v = QFrame(); v.setFrameShape(QFrame.VLine)
        v.setStyleSheet(f"background: {BORDER};"); v.setFixedWidth(1)
        return v

    # ════════════════════════════════════════════════════════════
    #  Input → CanInterface
    # ════════════════════════════════════════════════════════════

    def _on_acc_toggle(self):
        # OFF → ON 전이일 때 현재 자차 속도를 목표 속도로 캡처.
        # 액셀 페달 인가 중에도 ACC ON 가능 — ECU 가 SET_ACC_SPD=0 을 보고 ON 거부하는 현상 방지.
        # send_acc_setting + send_button_input 은 같은 _tx_acc_ctrl(50ms) 프레임에 실린다.
        status = status_from_int(self._can.get_acc_info().status)
        if status == AccStatus.OFF:
            captured = self._capture_current_speed()
            self._can.send_acc_setting(AccSetting(
                set_speed=captured, distance_level=self._last_distance_level,
            ))
            self._last_set_speed = captured
        self._can.send_button_input(ButtonInput(
            btn_acc_off=True, btn_acc_set=None, btn_acc_res=None, btn_acc_cancel=None,
        ))

    def _on_cancel(self):
        self._can.send_button_input(ButtonInput(
            btn_acc_off=None, btn_acc_set=None, btn_acc_res=None, btn_acc_cancel=True,
        ))

    def _on_set(self):
        # STANDBY 에서 액셀 인가 중 SET 누를 때 현재 자차 속도를 목표로 캡처.
        # set_speed 를 버튼 edge 와 같은 ACC_CTRL 프레임에 실어 ECU 가 stale 0 으로
        # OFF↔ON 핑퐁 치는 현상을 막는다.
        captured = self._capture_current_speed()
        self._can.send_acc_setting(AccSetting(
            set_speed=captured, distance_level=self._last_distance_level,
        ))
        self._last_set_speed = captured
        self._can.send_button_input(ButtonInput(
            btn_acc_off=None, btn_acc_set=True, btn_acc_res=None, btn_acc_cancel=None,
        ))
        # SET edge 와 함께 ECU 가 현재 PWM 을 캡처할 수 있도록 약간의 윈도우를 준 뒤 PWM 0 리셋.
        # (TX 주기: ACC_CTRL 50ms / VEH_CTRL 20ms — ECU 가 SET edge 를 처리할 때까지 PWM 유지)
        QTimer.singleShot(120, self._reset_user_pwm)

    def _capture_current_speed(self) -> int:
        """현재 자차 속도(cm/s) → ACC 목표 속도. [MIN, MAX] 범위로 clamp."""
        veh = self._can.get_vehicle_info()
        speed = int(round(float(veh.current_speed)))
        return max(MIN_SET_SPEED_CMS, min(MAX_SET_SPEED_CMS, speed))

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
        if self._brake_pressed:
            # 브레이크 입력 → 사용자 PWM 강제 0 (가속/브레이크 동시 인가 방지).
            self._accel_pwm = 0
            self.throttle_strip.reset_to_zero()
        self._can.send_pedal_input(PedalInput(
            brake=self._brake_pressed, accel_pwm=self._accel_pwm,
        ))

    def _on_accel(self, pwm: int):
        self._accel_pwm = max(0, min(100, int(pwm)))
        self._can.send_pedal_input(PedalInput(
            brake=self._brake_pressed, accel_pwm=self._accel_pwm,
        ))

    def _reset_user_pwm(self):
        """SET 후 사용자 PWM 을 0 으로 되돌림. ACC 가 throttle 제어 인계 받은 시점."""
        self._accel_pwm = 0
        self.throttle_strip.reset_to_zero()
        self._can.send_pedal_input(PedalInput(
            brake=self._brake_pressed, accel_pwm=self._accel_pwm,
        ))

    # ════════════════════════════════════════════════════════════
    #  Window events
    # ════════════════════════════════════════════════════════════

    def keyPressEvent(self, e):
        # F11 / Ctrl+Enter — 전체화면 토글 (양방향)
        if e.key() in (Qt.Key_F11, Qt.Key_Return) and (
            e.key() == Qt.Key_F11 or e.modifiers() & Qt.ControlModifier
        ):
            self.showNormal() if self.isFullScreen() else self.showFullScreen()
        elif e.key() == Qt.Key_Escape:
            # ESC: 전체화면이면 빠져나오기만, 아니면 종료 (alt-tab/X 가 막힐 때 탈출구).
            if self.isFullScreen():
                self.showNormal()
            else:
                self.close()
        else:
            super().keyPressEvent(e)

    def closeEvent(self, e):
        # X 버튼 / ESC 시 QApplication 까지 확실히 종료 (preview 창이 남아 있어도 같이 quit).
        QApplication.quit()
        super().closeEvent(e)

    # ════════════════════════════════════════════════════════════
    #  Poll + Render
    # ════════════════════════════════════════════════════════════

    def _refresh(self):
        acc_info = self._can.get_acc_info()
        veh_info = self._can.get_vehicle_info()
        ecu = self._can.get_ecu_status()

        status = status_from_int(acc_info.status)
        # ECU 피드백을 따라가도록 로컬 캐시 갱신
        self._last_set_speed = int(acc_info.set_speed)
        self._last_distance_level = int(acc_info.distance_level)

        active = is_active(status)
        current_speed = float(veh_info.current_speed)
        set_speed = int(acc_info.set_speed)
        can_connected = self._can.is_connected()

        self._render_gauges(status, active, current_speed, set_speed)
        self._render_hud(status, active, set_speed, int(acc_info.distance_level))
        self._render_fault_banner(status, ecu, can_connected)
        self._render_link(can_connected, ecu.heartbeat_ok)
        self._render_buttons(status, int(acc_info.distance_level))

    def _render_gauges(self, status: AccStatus, active: bool, current_speed: float, set_speed: int):
        extra = f"SET {set_speed} cm/s" if active or status == AccStatus.STANDBY else ""
        self.gauge.set_value(current_speed, extra=extra, extra_color=CLR.get(status, "#555"))
        self.pwm_gauge.set_value(pwm_display_pct(status, current_speed, set_speed, self._accel_pwm))

    def _render_hud(self, status: AccStatus, active: bool, set_speed: int, distance_level: int):
        sc = CLR_HUD[status]
        self._hud_dot.setStyleSheet(f"background:{sc};border-radius:7px;border:1px solid {sc};")
        self._hud_state.setText(STATE_TEXT[status])
        self._hud_state.setStyleSheet(f"color:{sc};background:transparent;letter-spacing:3px;")

        self._hud_override.setVisible(
            is_override_visible(status, self._brake_pressed, self._accel_pwm)
        )

        text, color = set_speed_label(status, set_speed, active)
        self._hud_set.setText(text)
        self._hud_set.setStyleSheet(f"color:{color};background:transparent;")

        self._hud_gap.set_level(distance_level, color=gap_color(status, distance_level))

    def _render_fault_banner(self, status: AccStatus, ecu, can_connected: bool):
        # SWR019 — FAULT 진입 / HB lost / 에러코드 수신 시 표시.
        text = fault_banner_text(status, ecu, can_connected)
        if text:
            self.fault_banner.setText(text)
            self.fault_banner.show()
        else:
            self.fault_banner.hide()

    def _render_link(self, can_connected: bool, heartbeat_ok: bool):
        text, color = link_indicator(can_connected, heartbeat_ok)
        self._hud_link.setText(text)
        self._hud_link.setStyleSheet(f"color:{color};background:transparent;")

    def _render_buttons(self, status: AccStatus, distance_level: int):
        avail = BUTTON_AVAILABILITY[status]
        for key, btn in self.buttons.items():
            btn.setEnabled(avail.get(key, False))

        # ACC toggle highlight
        if "acc_toggle" in self.buttons:
            btn = self.buttons["acc_toggle"]
            if status == AccStatus.OFF:
                btn.setText("ACC OFF"); btn.setStyleSheet(btn_style("#E24B4A"))
            else:
                btn.setText("ACC ON"); btn.setStyleSheet(btn_active_style(CLR[status]))

        # Distance buttons highlight
        for i in (1, 2, 3):
            if (k := f"dist_{i}") in self.buttons:
                b = self.buttons[k]
                if distance_level == i and status != AccStatus.OFF:
                    b.setStyleSheet(btn_active_style(DIST_CLR[i]))
                else:
                    b.setStyleSheet(btn_style("#378ADD"))


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
    # 기본은 maximized (X 버튼·title bar 노출). F11 로 fullscreen 토글, ESC 로 종료.
    win.showMaximized()

    preview = None
    if fusion is not None and fusion.show_window:
        preview = YoloPreview(fusion)
        preview.resize(640, 480)
        preview.show()

    return app.exec_()
