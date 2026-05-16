"""Nero package exports.

Teleop interfaces are loaded lazily so kinematics-only users do not need the
hardware/zerorpc stack at import time.
"""

__all__ = ["NeroDualArmServer"]


def __getattr__(name):
    if name == "NeroDualArmServer":
        from nero.teleop.interface import NeroDualArmServer

        return NeroDualArmServer
    raise AttributeError(f"module 'nero' has no attribute {name!r}")
