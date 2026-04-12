from dataclasses import dataclass


@dataclass
class EcuStatus:
    heartbeat_ok: bool  # 150ms 이내 HB 수신 여부
    error_code: int  # 0=정상
