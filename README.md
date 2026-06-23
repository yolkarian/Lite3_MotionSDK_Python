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

## API overview

- `RobotCmd`: mutable 12-joint command buffer.
- `RobotData`: copied snapshot of one robot-state packet.
- `Sender`: wraps `Sender_create`, `Sender_sendCmd`, and control handoff helpers.
- `Receiver`: wraps `Receiver_create`, `Receiver_startWork`, and `Receiver_getState`.
- `Timer`: wraps the SDK `DRTimer`.
- `Command`: thin wrapper around the SDK command helper.
- `record_initial_data`, `pre_stand_up`, `stand_up`, `cubic_spline`: Pythonic versions of the original demo helpers.

Supported targets are Linux `x86_64` and Linux `aarch64`, matching the bundled SDK runtime libraries.
