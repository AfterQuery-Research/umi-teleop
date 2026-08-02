"""Small data types shared by the source process."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum, auto

import numpy as np


class Event(Enum):
    """A physical or test control event."""

    START = auto()
    STOP = auto()
    TOGGLE = auto()
    CANCEL = auto()
    QUIT = auto()
    ZERO = auto()


@dataclass
class Frame:
    """One timestamped RGB camera frame."""

    timestamp: float
    image: np.ndarray | None = None
