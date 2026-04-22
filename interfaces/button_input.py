from dataclasses import dataclass
from typing import Optional


@dataclass
class ButtonInput:
    btn_acc_off: Optional[bool]
    btn_acc_set: Optional[bool]
    btn_acc_res: Optional[bool]
    btn_acc_cancel: Optional[bool]
