import json
from pathlib import Path

from .models import PlannerConfig


def load_config(path: str | Path) -> PlannerConfig:
    with Path(path).open(encoding="utf-8") as handle:
        return PlannerConfig.from_dict(json.load(handle))
