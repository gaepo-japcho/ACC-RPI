"""
ACC Project — DBC 로드 + CAN 메시지 ID
========================================
진실의 원천: ACC-CANDB/acc_db.dbc

acc_can 하위 모듈 전역에서 이 파일을 거쳐 DBC 에 접근한다. 다른 모듈
(예: acc_fusion) 도 CAN 메시지를 직접 송신할 일이 생기면
`from acc_can._dbc import msg, msg_id, MSG_ID_...` 로 이용 가능.

DBC 로드는 import 시점에 한 번만 일어나며, 실패 시 예외가 그대로 전파되어
import 자체가 실패한다 (엄격 모드 — CAN ID 가 잘못 매핑된 채 런타임 진입
하는 사고 방지).
"""
from pathlib import Path

import cantools

# 상대경로: ACC-RPI/acc_can/_dbc.py → parents[2]=/mnt/c/acc → ACC-CANDB/
_DBC_PATH = Path(__file__).resolve().parents[2] / "ACC-CANDB" / "acc_db.dbc"
_db = cantools.database.load_file(str(_DBC_PATH))


def msg(name: str):
    """DBC 메시지 객체 조회. 이름 오타 시 KeyError 전파."""
    return _db.get_message_by_name(name)


def msg_id(name: str) -> int:
    """DBC 메시지의 CAN arbitration ID."""
    return msg(name).frame_id


# ── CAN Message IDs (DBC 조회 — 하드코딩 금지) ──────────────────────────────
MSG_ID_SENSOR_FUSION    = msg_id("SENSOR_FUSION")      # 0x110, SENSOR→ECU, 20ms
MSG_ID_SENSOR_HEARTBEAT = msg_id("SENSOR_HEARTBEAT")   # 0x111, SENSOR→ECU, 20ms
MSG_ID_VEH_CTRL         = msg_id("VEH_CTRL")           # 0x120, SENSOR→ECU, 20ms
MSG_ID_ACC_CTRL         = msg_id("ACC_CTRL")           # 0x510, SENSOR→ECU, 50ms
MSG_ID_ACC_STATUS       = msg_id("ACC_STATUS")         # 0x520, ECU→SENSOR, 50ms
MSG_ID_ECU_HEARTBEAT    = msg_id("ECU_HEARTBEAT")      # 0x410, ECU→*,      10ms
