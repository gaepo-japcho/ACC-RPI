"""
CAN Interface abstraction for HMI ↔ Main ECU communication.
Ref: SYS030, SYS031, SWR020, SWR030

In production: uses python-can with USB-CAN or MCP2515 adapter.
In simulation: bypasses CAN, talks directly to AccController.
"""
import struct
from abc import ABC, abstractmethod


# CAN message IDs (placeholder – match actual DBC later)
CAN_ID_GUI_TO_ECU = 0x200   # 운전자 입력 (20ms cycle)
CAN_ID_ECU_TO_GUI = 0x201   # HMI 표시 데이터 (50ms cycle)


class CanInterfaceBase(ABC):
    @abstractmethod
    def send_driver_input(self, msg_bytes: bytes) -> None: ...

    @abstractmethod
    def receive_display_data(self) -> bytes | None: ...


class SimulatedCanInterface(CanInterfaceBase):
    """Loopback for standalone testing – no real CAN hardware."""
    def send_driver_input(self, msg_bytes: bytes) -> None:
        pass

    def receive_display_data(self) -> bytes | None:
        return None


class RealCanInterface(CanInterfaceBase):
    """Production CAN interface using python-can."""
    def __init__(self, channel: str = "can0", bustype: str = "socketcan"):
        import can
        self.bus = can.interface.Bus(channel=channel, bustype=bustype)

    def send_driver_input(self, msg_bytes: bytes) -> None:
        import can
        msg = can.Message(arbitration_id=CAN_ID_GUI_TO_ECU, data=msg_bytes, is_extended_id=False)
        self.bus.send(msg)

    def receive_display_data(self) -> bytes | None:
        msg = self.bus.recv(timeout=0.01)
        if msg and msg.arbitration_id == CAN_ID_ECU_TO_GUI:
            return bytes(msg.data)
        return None


def encode_driver_input(
    acc_toggle=False, set_btn=False, res_btn=False,
    speed_up=False, speed_down=False, dist_level=0,
    pause_btn=False, brake=False, accel=False,
) -> bytes:
    """Pack driver input into 8-byte CAN payload (SYS030)."""
    flags = (
        (acc_toggle << 0) | (set_btn << 1) | (res_btn << 2)
        | (speed_up << 3) | (speed_down << 4) | (pause_btn << 5)
        | (brake << 6) | (accel << 7)
    )
    return struct.pack("<BB6x", flags, dist_level & 0x03)
