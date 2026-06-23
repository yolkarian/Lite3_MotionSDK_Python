"""Typed dictionary shapes returned by the Lite3 MotionSDK bindings."""

from typing import TypedDict


class JointCmdDict(TypedDict):
    """Copied command fields for one joint."""

    position: float
    velocity: float
    torque: float
    kp: float
    kd: float


class JointDataDict(TypedDict):
    """Copied measured data fields for one joint."""

    position: float
    velocity: float
    torque: float
    temperature: float


class ImuDict(TypedDict):
    """Copied IMU fields from a robot-state packet."""

    timestamp: int
    angle_roll: float
    angle_pitch: float
    angle_yaw: float
    angular_velocity_roll: float
    angular_velocity_pitch: float
    angular_velocity_yaw: float
    acc_x: float
    acc_y: float
    acc_z: float
