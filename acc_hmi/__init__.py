"""
ACC HMI module.

루트 `main.py` 에서 `import acc_hmi as hmi` 로 가져와 `hmi.gui_main(can_interface)` 로 실행한다.
PyQt5 는 `HmiWindow` / `gui_main` 접근 시에만 lazy import 된다 — Qt 가 없는 환경에서도
`acc_hmi.acc_state` 같은 하위 모듈을 개별 import 할 수 있도록.
"""

__all__ = ["HmiWindow", "gui_main"]


def __getattr__(name):
    if name in __all__:
        from acc_hmi.hmi_gui import HmiWindow, gui_main
        return {"HmiWindow": HmiWindow, "gui_main": gui_main}[name]
    raise AttributeError(f"module 'acc_hmi' has no attribute {name!r}")
