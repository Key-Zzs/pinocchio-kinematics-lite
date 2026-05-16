from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
import pytest

SRC = Path(__file__).resolve().parents[1] / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))

from pinocchio_kinematics_lite import NeroKinematics


@pytest.fixture
def pinocchio_available():
    return pytest.importorskip("pinocchio")


@pytest.fixture
def nero_kin(pinocchio_available):
    return NeroKinematics()


@pytest.fixture
def rng():
    return np.random.default_rng(42)
