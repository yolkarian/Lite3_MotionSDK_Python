# Lite3 MotionSDK Python

Python bindings for the DeepRobotics Jueying Lite3 MotionSDK, built with `ziggy-pydust`.

Built wheels bundle the DeepRobotics runtime `.so` files matching the compiled extension architecture inside `lite3_motion_sdk/`. The native extension and wrapped SDK are linked with `$ORIGIN` runpaths, so users do not need to install the SDK shared libraries separately.

> Safety: joint-level control can move the robot unexpectedly. Keep the robot suspended, keep people away, verify UDP communication and command values first, and be ready to emergency-stop or stop `jy_exe`.

## Build and test

```bash
uv sync --all-groups
uv run pytest
```

Build only the native extension:

```bash
uv run python scripts/build_extension.py
```

Build a native-architecture wheel:

```bash
uv run python scripts/build_extension.py
uv build --wheel
```

Build an aarch64 wheel from an x86_64 host:

```bash
uv run python scripts/build_extension.py --target aarch64-linux-gnu
uv build --wheel
```

`setup.py` inspects `lite3_motion_sdk/_lib.abi3.so` and includes only the matching runtime pair in the wheel:

- `libdeeprobotics_legged_wrapped_sdk_x86_64.so` + `libdeeprobotics_legged_sdk_x86_64.so` for `linux_x86_64` wheels.
- `libdeeprobotics_legged_wrapped_sdk_aarch64.so` + `libdeeprobotics_legged_sdk_aarch64.so` for `linux_aarch64` wheels.

## Quick example

```python
import lite3_motion_sdk as sdk

cmd = sdk.RobotCmd()
cmd.set_joint(0, 0.0, 0.0, 0.0, 30.0, 1.0)

sender = sdk.Sender(ip="192.168.1.120", port=43893)
try:
    sender.robot_state_init()
    sender.send_cmd(cmd)
finally:
    sender.return_control()
    sender.close()
```

A stand-up style loop follows the original SDK demo flow:

```python
import lite3_motion_sdk as sdk

cmd = sdk.RobotCmd()
receiver = sdk.Receiver()
sender = sdk.Sender()
timer = sdk.Timer(ms=1)

receiver.register_update_flag()
receiver.start_work()
sender.robot_state_init()

state = receiver.get_state()
sdk.record_initial_data(state, 0.0)
start = timer.current_time()

for tick in range(10_000):
    if timer.interrupt():
        continue
    now = timer.interval_time(start)
    state = receiver.get_state()
    if tick < 1000:
        sdk.pre_stand_up(cmd, now, state)
    else:
        sdk.stand_up(cmd, now, state)
    if receiver.message_updated():
        sender.send_cmd(cmd)
        receiver.clear_update_flag()

sender.return_control()
```

## Python API

The public package re-exports the native module API from `lite3_motion_sdk._lib` and ships `py.typed` plus `.pyi` stubs.
Most methods intentionally mirror the SDK structs and C wrapper names, while keeping resource ownership in Python classes.

### Constants

| Name | Value | Meaning |
| :-- | --: | :-- |
| `ROBOT` / `ROBOT_CONTROL` | `1` | Return control to the robot's original controller. |
| `SDK` / `SDK_CONTROL` | `2` | Request SDK-side control. |

### Returned dictionary types

These are exported as `TypedDict` names from `lite3_motion_sdk` and are also declared in the shipped stubs.
The values returned by methods such as `RobotCmd.joint()` are plain Python dictionaries at runtime.

```python
class JointCmdDict(TypedDict):
    position: float
    velocity: float
    torque: float
    kp: float
    kd: float

class JointDataDict(TypedDict):
    position: float
    velocity: float
    torque: float
    temperature: float

class ImuDict(TypedDict):
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
```

Joint indexes are `0..11`. Leg order is `FL`, `FR`, `HL`, `HR`; each leg has three joints.

### `RobotCmd`

Mutable command buffer matching SDK `RobotCmd` with 12 `JointCmd` entries.
Use this object to fill commands and pass it to `Sender.send_cmd()`.

```python
cmd = sdk.RobotCmd()
cmd.set_joint(0, position, velocity, torque, kp, kd)
cmd.set_leg("FL", position, velocity, torque, kp, kd)
cmd.set_all(position, velocity, torque, kp, kd)
```

Methods:

| Signature | Description |
| :-- | :-- |
| `RobotCmd()` | Create a zero-initialized command buffer. |
| `zero() -> None` | Reset all 12 joint commands to zero. |
| `set_joint(index, position, velocity, torque, kp, kd) -> None` | Replace one joint command. Raises `IndexError` if `index` is not in `0..11`. |
| `set_leg(side, position, velocity, torque, kp, kd) -> None` | Replace all three commands for `side`, one of `"FL"`, `"FR"`, `"HL"`, `"HR"`. |
| `set_all(position, velocity, torque, kp, kd) -> None` | Replace all 12 joint commands. |
| `joint(index) -> JointCmdDict` | Return a copied dictionary for one joint command. |
| `joints() -> list[JointCmdDict]` | Return copied dictionaries for all 12 joint commands. |
| `copy() -> RobotCmd` | Return a deep copy of the command buffer. |

### `RobotData`

Copied snapshot matching SDK `RobotData`. `Receiver.get_state()` returns a `RobotData` copy,
so reading it is safe from Python ownership/lifetime issues. Mutating a `RobotData` instance is useful for tests
and helper functions, but it does not write back into the receiver's internal C++ buffer.

Methods:

| Signature | Description |
| :-- | :-- |
| `RobotData()` | Create a zero-initialized data snapshot. |
| `tick() -> int` / `set_tick(tick) -> None` | Read or set the packet tick. |
| `imu() -> ImuDict` | Return copied IMU fields. Angles and angular velocities follow the SDK units. |
| `set_imu(timestamp, angle_roll, angle_pitch, angle_yaw, angular_velocity_roll, angular_velocity_pitch, angular_velocity_yaw, acc_x, acc_y, acc_z) -> None` | Fill IMU fields. Mainly intended for tests and offline command generation. |
| `set_joint(index, position, velocity, torque, temperature) -> None` | Fill one joint-data entry. |
| `joint(index) -> JointDataDict` | Return copied data for one joint. |
| `joints() -> list[JointDataDict]` | Return copied data for all 12 joints. |
| `set_contact_force(index, force) -> None` | Fill one contact-force value. |
| `contact_force(index) -> float` | Return one contact-force value. |
| `contact_forces() -> list[float]` | Return all 12 contact-force values. |
| `copy() -> RobotData` | Return a deep copy of the snapshot. |

### `Sender`

Owns an SDK `SenderHandle` and releases it in `close()` / destructor.
Use `Sender(ip=..., port=...)` when the robot motion host is not the SDK default target.

Methods:

| Signature | Description |
| :-- | :-- |
| `Sender(*, ip: str | None = None, port: int = 43893)` | Create the sender. `ip=None` uses the SDK default destination. |
| `send_cmd(cmd: RobotCmd) -> None` | Send a 12-joint command through UDP. |
| `control_get(mode: int) -> None` | Call raw control handoff with `ROBOT` or `SDK`. |
| `return_control() -> None` | Equivalent to `control_get(ROBOT)`. |
| `request_sdk_control() -> None` | Equivalent to `control_get(SDK)`. |
| `all_joint_back_zero() -> None` | Send the SDK all-joints-back-zero command. |
| `robot_state_init() -> None` | Return joints to zero and acquire SDK control, matching the original demo flow. |
| `set_cmd(code: int, value: int) -> None` | Send a raw command code/value pair. |
| `close() -> None` | Destroy the underlying sender handle. Safe to call more than once. |

### `Receiver`

Owns an SDK `ReceiverHandle`. The Python API currently exposes the SDK update notification through an internal flag
instead of accepting arbitrary Python callbacks from C++ receiver threads.

Methods:

| Signature | Description |
| :-- | :-- |
| `Receiver()` | Create a receiver. |
| `start_work() -> None` | Start the SDK receive worker. |
| `register_update_flag() -> None` | Register an internal C callback that sets a flag when instruction `0x0906` arrives. |
| `message_updated() -> bool` | Read the internal update flag. |
| `clear_update_flag() -> None` | Clear the internal update flag after processing/sending. |
| `get_state() -> RobotData` | Copy the latest SDK `RobotData` into a Python-owned snapshot. |
| `close() -> None` | Destroy the underlying receiver handle. Safe to call more than once. |

### `Timer`

Owns an SDK `DRTimerHandle`. This mirrors the original 1 ms control-loop demo.

Methods:

| Signature | Description |
| :-- | :-- |
| `Timer(*, ms: int | None = None)` | Create a timer and optionally initialize its interval in milliseconds. |
| `init(ms: int) -> None` | Initialize or reinitialize the timer interval. |
| `interrupt() -> bool` | Return the SDK `DRTimer_interrupt()` result. The original demo continues the loop when this returns `True`. |
| `current_time() -> float` | Return SDK current time in seconds. |
| `interval_time(start_time: float) -> float` | Return elapsed time relative to `start_time`. |
| `close() -> None` | Destroy the underlying timer handle. Safe to call more than once. |

### `Command`

Thin owner for the SDK `Command` helper.

| Signature | Description |
| :-- | :-- |
| `Command(code: int, *, value: int | None = None, parameters: bytes | None = None)` | Create a value command or a parameter-buffer command. Passing both is invalid. |
| `code() -> int` | Return the command code. |
| `value() -> int` | Return the command value for value commands. |
| `parameters_size() -> int` | Return parameter buffer size. |
| `parameters() -> bytes` | Copy command parameters into Python `bytes`. |
| `close() -> None` | Destroy the underlying command and owned parameter buffer. |

### Motion helper functions

These are Python-callable versions of the original stand-up demo helpers.
They mutate the provided `RobotCmd` in place.

| Signature | Description |
| :-- | :-- |
| `record_initial_data(data: RobotData, time: float) -> None` | Record current joint positions as the next motion phase's initial state. |
| `pre_stand_up(cmd: RobotCmd, time: float, data: RobotData) -> None` | Fill `cmd` for the 1.0 s pre-stand phase. |
| `stand_up(cmd: RobotCmd, time: float, data: RobotData) -> None` | Fill `cmd` for the 1.5 s stand-up phase and hold final posture afterwards. |
| `cubic_spline(init_position, init_velocity, goal_position, goal_velocity, run_time, cycle_time, total_time) -> tuple[float, float, float]` | Return current, next, and next-next cubic-spline positions. |

Supported targets are Linux `x86_64` and Linux `aarch64`, matching the bundled SDK runtime libraries.
