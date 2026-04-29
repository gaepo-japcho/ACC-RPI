import tomllib
from pathlib import Path

_CONFIG_PATH = Path(__file__).parent.parent / "config.toml"

with _CONFIG_PATH.open("rb") as f:
    config = tomllib.load(f)
