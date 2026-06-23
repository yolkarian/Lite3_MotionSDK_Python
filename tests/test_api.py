import math

import pytest

import lite3_motion_sdk as sdk


def test_robot_cmd_joint_access() -> None:
    cmd = sdk.RobotCmd()
    assert cmd.joint(0) == {"position": 0.0, "velocity": 0.0, "torque": 0.0, "kp": 0.0, "kd": 0.0}

    cmd.set_joint(0, 1.0, 2.0, 3.0, 4.0, 5.0)
    assert cmd.joint(0) == {"position": 1.0, "velocity": 2.0, "torque": 3.0, "kp": 4.0, "kd": 5.0}
    assert len(cmd.joints()) == 12

    copied = cmd.copy()
    assert copied.joint(0) == cmd.joint(0)

    with pytest.raises(IndexError):
        cmd.joint(12)


def test_robot_data_access() -> None:
    data = sdk.RobotData()
    data.set_tick(123)
    data.set_imu(0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 9.8)
    data.set_joint(2, 0.25, 0.5, 0.75, 32.0)
    data.set_contact_force(2, 42.0)

    assert data.tick() == 123
    assert data.imu()["angle_roll"] == 1.0
    assert data.imu()["acc_z"] == pytest.approx(9.8, rel=1e-6)
    assert data.joint(2)["temperature"] == 32.0
    assert data.contact_force(2) == 42.0
    assert len(data.joints()) == 12
    assert len(data.contact_forces()) == 12


def test_motion_helpers() -> None:
    cmd = sdk.RobotCmd()
    data = sdk.RobotData()
    for index in range(12):
        data.set_joint(index, 0.1 * index, 0.0, 0.0, 0.0)

    sdk.record_initial_data(data, 0.0)
    sdk.pre_stand_up(cmd, 0.5, data)
    assert cmd.joint(1)["kp"] == pytest.approx(60.0)

    sdk.stand_up(cmd, 2.0, data)
    assert cmd.joint(1)["position"] == pytest.approx(-42.0 * math.pi / 180.0, rel=1e-5)
    assert cmd.joint(2)["position"] == pytest.approx(78.0 * math.pi / 180.0, rel=1e-5)


def test_cubic_spline() -> None:
    position, next_position, next2_position = sdk.cubic_spline(0.0, 0.0, 1.0, 0.0, 0.5, 0.001, 1.0)
    assert position == pytest.approx(0.5)
    assert next_position > position
    assert next2_position > next_position


def test_command_value() -> None:
    command = sdk.Command(0x1234, value=5)
    try:
        assert command.code() == 0x1234
        assert command.value() == 5
    finally:
        command.close()


def test_timer_can_be_created_and_closed() -> None:
    timer = sdk.Timer(ms=1)
    try:
        assert timer.current_time() >= 0.0
    finally:
        timer.close()


def test_constants() -> None:
    assert sdk.ROBOT == 1
    assert sdk.SDK == 2
