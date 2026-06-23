"""Python bindings for the DeepRobotics Jueying Lite3 MotionSDK."""

from ._lib import (
    Command,
    Receiver,
    RobotCmd,
    RobotData,
    Sender,
    Timer,
    cubic_spline,
    pre_stand_up,
    record_initial_data,
    stand_up,
)

ROBOT = 1
SDK = 2
ROBOT_CONTROL = ROBOT
SDK_CONTROL = SDK

__all__ = [
    "Command",
    "Receiver",
    "RobotCmd",
    "RobotData",
    "ROBOT",
    "ROBOT_CONTROL",
    "SDK",
    "SDK_CONTROL",
    "Sender",
    "Timer",
    "cubic_spline",
    "pre_stand_up",
    "record_initial_data",
    "stand_up",
]
