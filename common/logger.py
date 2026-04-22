import logging
import sys
from pathlib import Path
from datetime import datetime

# ANSI color codes
_COLORS = {
    "DEBUG":    "\033[36m",   # Cyan
    "INFO":     "\033[32m",   # Green
    "WARNING":  "\033[33m",   # Yellow
    "ERROR":    "\033[31m",   # Red
    "CRITICAL": "\033[35m",   # Magenta
    "RESET":    "\033[0m",
}

_LOG_DIR = Path(__file__).parent.parent / "logs"
_initialized = False


class _ColorFormatter(logging.Formatter):
    FMT = "[{asctime}] [{levelname:<8}] [{name}] {message}"

    def format(self, record: logging.LogRecord) -> str:
        color = _COLORS.get(record.levelname, "")
        reset = _COLORS["RESET"]
        formatter = logging.Formatter(
            fmt=f"{color}{self.FMT}{reset}",
            datefmt="%H:%M:%S",
            style="{",
        )
        return formatter.format(record)


class _PlainFormatter(logging.Formatter):
    FMT = "[{asctime}] [{levelname:<8}] [{name}] {message}"

    def __init__(self):
        super().__init__(fmt=self.FMT, datefmt="%Y-%m-%d %H:%M:%S", style="{")


def _setup_root_logger(level: int = logging.DEBUG, log_to_file: bool = True) -> None:
    global _initialized
    if _initialized:
        return

    root = logging.getLogger("acc")
    root.setLevel(level)
    root.propagate = False

    # Console handler (colored)
    console = logging.StreamHandler(sys.stdout)
    console.setLevel(level)
    console.setFormatter(_ColorFormatter())
    root.addHandler(console)

    # File handler (plain text)
    if log_to_file:
        _LOG_DIR.mkdir(exist_ok=True)
        log_file = _LOG_DIR / f"{datetime.now():%Y%m%d_%H%M%S}.log"
        file_handler = logging.FileHandler(log_file, encoding="utf-8")
        file_handler.setLevel(level)
        file_handler.setFormatter(_PlainFormatter())
        root.addHandler(file_handler)

    _initialized = True


def get_logger(name: str, level: int = logging.DEBUG) -> logging.Logger:
    """Get a logger namespaced under 'acc.<name>'.

    Usage:
        from common import get_logger
        log = get_logger(__name__)
        log.info("hello")
    """
    _setup_root_logger()
    logger = logging.getLogger(f"acc.{name}")
    logger.setLevel(level)
    return logger
