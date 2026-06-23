const std = @import("std");
const py = @import("pydust");

const root = @This();

const joint_count = 12;
const joints_per_leg = 3;
const robot_mode: u32 = 1;
const sdk_mode: u32 = 2;
const degree_to_radian = 3.1415926 / 180.0;

const CImuData = extern struct {
    timestamp: i32,
    buffer_float: [9]f32,
};

const CJointData = extern struct {
    position: f32,
    velocity: f32,
    torque: f32,
    temperature: f32,
};

const CLegData = extern struct {
    joint_data: [joint_count]CJointData,
};

const CJointCmd = extern struct {
    position: f32,
    velocity: f32,
    torque: f32,
    kp: f32,
    kd: f32,
};

const CRobotCmd = extern struct {
    joint_cmd: [joint_count]CJointCmd,
};

const CContactForce = extern struct {
    leg_force: [joint_count]f64,
};

const CRobotData = extern struct {
    tick: u32,
    imu: CImuData,
    joint_data: CLegData,
    contact_force: CContactForce,
};

const DRTimerHandle = opaque {};
const ReceiverHandle = opaque {};
const ReceiverCallback = *const fn (instruction: c_int, user_data: ?*anyopaque) callconv(.c) void;

extern fn Command_CreateWithValue(command_code: u32, command_value: i32) ?*anyopaque;
extern fn Command_CreateWithParameters(command_code: u32, parameters_size: usize, parameters: ?*anyopaque) ?*anyopaque;
extern fn Command_Destroy(handle: ?*anyopaque) void;
extern fn Command_GetCode(handle: ?*anyopaque) u32;
extern fn Command_GetValue(handle: ?*anyopaque) i32;
extern fn Command_GetParametersSize(handle: ?*anyopaque) usize;
extern fn Command_GetParameters(handle: ?*anyopaque) ?*anyopaque;

extern fn DRTimer_create() ?*DRTimerHandle;
extern fn DRTimer_destroy(timer: ?*DRTimerHandle) void;
extern fn DRTimer_init(timer: *DRTimerHandle, ms: c_int) void;
extern fn DRTimer_interrupt(timer: *DRTimerHandle) bool;
extern fn DRTimer_getIntervalTime(timer: *DRTimerHandle, start_time: f64) f64;
extern fn DRTimer_getCurrentTime(timer: *DRTimerHandle) f64;

extern fn Receiver_create() ?*ReceiverHandle;
extern fn Receiver_destroy(handle: ?*ReceiverHandle) void;
extern fn Receiver_startWork(handle: *ReceiverHandle) void;
extern fn Receiver_registerCallback(handle: *ReceiverHandle, callback: ReceiverCallback, user_data: ?*anyopaque) void;
extern fn Receiver_getState(handle: *ReceiverHandle) ?*CRobotData;

extern fn Sender_create() ?*anyopaque;
extern fn Sender_createWithIpPort(ip: [*:0]const u8, port: u16) ?*anyopaque;
extern fn Sender_destroy(handle: ?*anyopaque) void;
extern fn Sender_sendCmd(handle: ?*anyopaque, robot_cmd: *const CRobotCmd) void;
extern fn Sender_controlGet(handle: ?*anyopaque, mode: u32) void;
extern fn Sender_allJointBackZero(handle: ?*anyopaque) void;
extern fn Sender_robotStateInit(handle: ?*anyopaque) void;
extern fn Sender_setCmd(handle: ?*anyopaque, code: u32, value: u32) void;

const JointCmdSnapshot = struct {
    position: f32,
    velocity: f32,
    torque: f32,
    kp: f32,
    kd: f32,
};

const JointDataSnapshot = struct {
    position: f32,
    velocity: f32,
    torque: f32,
    temperature: f32,
};

const ImuSnapshot = struct {
    timestamp: i32,
    angle_roll: f32,
    angle_pitch: f32,
    angle_yaw: f32,
    angular_velocity_roll: f32,
    angular_velocity_pitch: f32,
    angular_velocity_yaw: f32,
    acc_x: f32,
    acc_y: f32,
    acc_z: f32,
};

const CubicSplineResult = struct { f64, f64, f64 };

var init_time: f64 = 0.0;
var init_angle_fl = [_]f64{ 0.0, 0.0, 0.0 };
var init_angle_fr = [_]f64{ 0.0, 0.0, 0.0 };
var init_angle_hl = [_]f64{ 0.0, 0.0, 0.0 };
var init_angle_hr = [_]f64{ 0.0, 0.0, 0.0 };

fn requireHandle(comptime T: type, handle: ?*T, comptime name: []const u8) !*T {
    return handle orelse py.RuntimeError(root).raise(name ++ " is closed");
}

fn requireOpaqueHandle(handle: ?*anyopaque, comptime name: []const u8) !*anyopaque {
    return handle orelse py.RuntimeError(root).raise(name ++ " is closed");
}

fn requireCreatedOpaque(handle: ?*anyopaque, comptime name: []const u8) !*anyopaque {
    return handle orelse py.RuntimeError(root).raise(name ++ " creation failed");
}

fn requireCreated(comptime T: type, handle: ?*T, comptime name: []const u8) !*T {
    return handle orelse py.RuntimeError(root).raise(name ++ " creation failed");
}

fn jointIndex(index: u8) !usize {
    if (index >= joint_count) {
        return py.IndexError(root).raiseFmt("joint index {} out of range [0, 12)", .{index});
    }
    return @intCast(index);
}

fn legBaseIndex(side: []const u8) !usize {
    if (std.mem.eql(u8, side, "FL")) return 0;
    if (std.mem.eql(u8, side, "FR")) return 3;
    if (std.mem.eql(u8, side, "HL")) return 6;
    if (std.mem.eql(u8, side, "HR")) return 9;
    return py.ValueError(root).raise("side must be one of 'FL', 'FR', 'HL', or 'HR'");
}

fn jointCmdSnapshot(value: CJointCmd) JointCmdSnapshot {
    return .{
        .position = value.position,
        .velocity = value.velocity,
        .torque = value.torque,
        .kp = value.kp,
        .kd = value.kd,
    };
}

fn jointDataSnapshot(value: CJointData) JointDataSnapshot {
    return .{
        .position = value.position,
        .velocity = value.velocity,
        .torque = value.torque,
        .temperature = value.temperature,
    };
}

fn imuSnapshot(value: CImuData) ImuSnapshot {
    return .{
        .timestamp = value.timestamp,
        .angle_roll = value.buffer_float[0],
        .angle_pitch = value.buffer_float[1],
        .angle_yaw = value.buffer_float[2],
        .angular_velocity_roll = value.buffer_float[3],
        .angular_velocity_pitch = value.buffer_float[4],
        .angular_velocity_yaw = value.buffer_float[5],
        .acc_x = value.buffer_float[6],
        .acc_y = value.buffer_float[7],
        .acc_z = value.buffer_float[8],
    };
}

fn jointsListFromCmd(cmd: *const CRobotCmd) !py.PyList(root) {
    const result = try py.PyList(root).new(joint_count);
    for (0..joint_count) |i| {
        try result.setItem(i, jointCmdSnapshot(cmd.joint_cmd[i]));
    }
    return result;
}

fn jointsListFromData(data: *const CRobotData) !py.PyList(root) {
    const result = try py.PyList(root).new(joint_count);
    for (0..joint_count) |i| {
        try result.setItem(i, jointDataSnapshot(data.joint_data.joint_data[i]));
    }
    return result;
}

fn contactForcesList(data: *const CRobotData) !py.PyList(root) {
    const result = try py.PyList(root).new(joint_count);
    for (0..joint_count) |i| {
        try result.setItem(i, data.contact_force.leg_force[i]);
    }
    return result;
}

fn cubicSplineValues(
    init_position: f64,
    init_velocity: f64,
    goal_position: f64,
    goal_velocity: f64,
    run_time_input: f64,
    cycle_time: f64,
    total_time: f64,
) !CubicSplineResult {
    if (total_time <= 0.0) {
        return py.ValueError(root).raise("total_time must be positive");
    }
    if (cycle_time <= 0.0) {
        return py.ValueError(root).raise("cycle_time must be positive");
    }

    const a = (goal_velocity * total_time - 2.0 * goal_position + init_velocity * total_time + 2.0 * init_position) / std.math.pow(f64, total_time, 3.0);
    const b = (3.0 * goal_position - goal_velocity * total_time - 2.0 * init_velocity * total_time - 3.0 * init_position) / std.math.pow(f64, total_time, 2.0);
    const c = init_velocity;
    const d = init_position;

    var run_time = if (run_time_input > total_time) total_time else run_time_input;
    const sub_goal_position = a * std.math.pow(f64, run_time, 3.0) + b * std.math.pow(f64, run_time, 2.0) + c * run_time + d;

    if (run_time + cycle_time > total_time) {
        run_time = total_time - cycle_time;
    }
    const next_time = run_time + cycle_time;
    const sub_goal_position_next = a * std.math.pow(f64, next_time, 3.0) + b * std.math.pow(f64, next_time, 2.0) + c * next_time + d;

    if (run_time + cycle_time * 2.0 > total_time) {
        run_time = total_time - cycle_time * 2.0;
    }
    const next2_time = run_time + cycle_time * 2.0;
    const sub_goal_position_next2 = a * std.math.pow(f64, next2_time, 3.0) + b * std.math.pow(f64, next2_time, 2.0) + c * next2_time + d;

    return .{ sub_goal_position, sub_goal_position_next, sub_goal_position_next2 };
}

fn writeLegPositionControl(cmd: *CRobotCmd, base: usize, goal_angle: [3]f64, goal_velocity: [3]f64) void {
    for (0..joints_per_leg) |j| {
        const index = base + j;
        cmd.joint_cmd[index].kp = 60.0;
        cmd.joint_cmd[index].kd = 0.7;
        cmd.joint_cmd[index].position = @floatCast(goal_angle[j]);
        cmd.joint_cmd[index].velocity = @floatCast(goal_velocity[j]);
    }
    for (0..joint_count) |i| {
        cmd.joint_cmd[i].torque = 0.0;
    }
}

fn swingToAngle(initial_angle: [3]f64, final_angle: [3]f64, total_time: f64, run_time: f64, cycle_time: f64, base: usize, cmd: *CRobotCmd) !void {
    var goal_angle = [_]f64{ 0.0, 0.0, 0.0 };
    var goal_angle_next = [_]f64{ 0.0, 0.0, 0.0 };
    var goal_velocity = [_]f64{ 0.0, 0.0, 0.0 };

    for (0..joints_per_leg) |j| {
        const spline = try cubicSplineValues(initial_angle[j], 0.0, final_angle[j], 0.0, run_time, cycle_time, total_time);
        goal_angle[j] = spline[0];
        goal_angle_next[j] = spline[1];
        goal_velocity[j] = (goal_angle_next[j] - goal_angle[j]) / cycle_time;
    }

    writeLegPositionControl(cmd, base, goal_angle, goal_velocity);
}

fn recordInitialData(data: *const CRobotData, time: f64) void {
    init_time = time;
    for (0..joints_per_leg) |j| {
        init_angle_fl[j] = data.joint_data.joint_data[j].position;
        init_angle_fr[j] = data.joint_data.joint_data[3 + j].position;
        init_angle_hl[j] = data.joint_data.joint_data[6 + j].position;
        init_angle_hr[j] = data.joint_data.joint_data[9 + j].position;
    }
}

fn fillPreStandUpCommand(cmd: *CRobotCmd, time: f64) !void {
    const standup_time = 1.0;
    const cycle_time = 0.001;
    const goal_angle = [_]f64{ 0.0 * degree_to_radian, -70.0 * degree_to_radian, 150.0 * degree_to_radian };

    if (time <= init_time + standup_time) {
        try swingToAngle(init_angle_fl, goal_angle, standup_time, time - init_time, cycle_time, 0, cmd);
        try swingToAngle(init_angle_fr, goal_angle, standup_time, time - init_time, cycle_time, 3, cmd);
        try swingToAngle(init_angle_hl, goal_angle, standup_time, time - init_time, cycle_time, 6, cmd);
        try swingToAngle(init_angle_hr, goal_angle, standup_time, time - init_time, cycle_time, 9, cmd);
    }
}

fn fillStandUpCommand(cmd: *CRobotCmd, time: f64) !void {
    const standup_time = 1.5;
    const cycle_time = 0.001;
    const goal_angle = [_]f64{ 0.0 * degree_to_radian, -42.0 * degree_to_radian, 78.0 * degree_to_radian };

    if (time <= init_time + standup_time) {
        try swingToAngle(init_angle_fl, goal_angle, standup_time, time - init_time, cycle_time, 0, cmd);
        try swingToAngle(init_angle_fr, goal_angle, standup_time, time - init_time, cycle_time, 3, cmd);
        try swingToAngle(init_angle_hl, goal_angle, standup_time, time - init_time, cycle_time, 6, cmd);
        try swingToAngle(init_angle_hr, goal_angle, standup_time, time - init_time, cycle_time, 9, cmd);
    } else {
        for (0..joint_count) |i| {
            cmd.joint_cmd[i].torque = 0.0;
            cmd.joint_cmd[i].kp = 80.0;
            cmd.joint_cmd[i].kd = 0.7;
        }
        for (0..4) |leg| {
            const base = 3 * leg;
            cmd.joint_cmd[base].position = 0.0;
            cmd.joint_cmd[base + 1].position = @floatCast(-42.0 * degree_to_radian);
            cmd.joint_cmd[base + 2].position = @floatCast(78.0 * degree_to_radian);
            cmd.joint_cmd[base].velocity = 0.0;
            cmd.joint_cmd[base + 1].velocity = 0.0;
            cmd.joint_cmd[base + 2].velocity = 0.0;
        }
    }
}

fn receiverUpdateCallback(instruction: c_int, user_data: ?*anyopaque) callconv(.c) void {
    if (instruction == 0x0906) {
        if (user_data) |ptr| {
            const flag: *u8 = @ptrCast(@alignCast(ptr));
            @atomicStore(u8, flag, 1, .release);
        }
    }
}

pub const RobotCmd = py.class(struct {
    pub const __doc__ = "Mutable command buffer for the 12 Lite3 joints.";

    const Self = @This();

    value: CRobotCmd,

    pub fn __init__(self: *Self) void {
        self.value = std.mem.zeroes(CRobotCmd);
    }

    pub fn zero(self: *Self) void {
        self.value = std.mem.zeroes(CRobotCmd);
    }

    pub fn set_joint(self: *Self, args: struct { index: u8, position: f32, velocity: f32, torque: f32, kp: f32, kd: f32 }) !void {
        const index = try jointIndex(args.index);
        self.value.joint_cmd[index] = .{
            .position = args.position,
            .velocity = args.velocity,
            .torque = args.torque,
            .kp = args.kp,
            .kd = args.kd,
        };
    }

    pub fn set_leg(self: *Self, args: struct { side: []const u8, position: f32, velocity: f32, torque: f32, kp: f32, kd: f32 }) !void {
        const base = try legBaseIndex(args.side);
        for (0..joints_per_leg) |j| {
            self.value.joint_cmd[base + j] = .{
                .position = args.position,
                .velocity = args.velocity,
                .torque = args.torque,
                .kp = args.kp,
                .kd = args.kd,
            };
        }
    }

    pub fn set_all(self: *Self, args: struct { position: f32, velocity: f32, torque: f32, kp: f32, kd: f32 }) void {
        for (0..joint_count) |i| {
            self.value.joint_cmd[i] = .{
                .position = args.position,
                .velocity = args.velocity,
                .torque = args.torque,
                .kp = args.kp,
                .kd = args.kd,
            };
        }
    }

    pub fn joint(self: *const Self, args: struct { index: u8 }) !JointCmdSnapshot {
        const index = try jointIndex(args.index);
        return jointCmdSnapshot(self.value.joint_cmd[index]);
    }

    pub fn joints(self: *const Self) !py.PyList(root) {
        return jointsListFromCmd(&self.value);
    }

    pub fn copy(self: *const Self) !*RobotCmd.definition {
        return py.init(root, RobotCmd.definition, .{ .value = self.value });
    }
});

pub const RobotData = py.class(struct {
    pub const __doc__ = "Snapshot of one Lite3 robot-state packet.";

    const Self = @This();

    value: CRobotData,

    pub fn __init__(self: *Self) void {
        self.value = std.mem.zeroes(CRobotData);
    }

    pub fn tick(self: *const Self) u32 {
        return self.value.tick;
    }

    pub fn set_tick(self: *Self, args: struct { tick: u32 }) void {
        self.value.tick = args.tick;
    }

    pub fn imu(self: *const Self) ImuSnapshot {
        return imuSnapshot(self.value.imu);
    }

    pub fn set_imu(self: *Self, args: struct {
        timestamp: i32,
        angle_roll: f32,
        angle_pitch: f32,
        angle_yaw: f32,
        angular_velocity_roll: f32,
        angular_velocity_pitch: f32,
        angular_velocity_yaw: f32,
        acc_x: f32,
        acc_y: f32,
        acc_z: f32,
    }) void {
        self.value.imu.timestamp = args.timestamp;
        self.value.imu.buffer_float = .{
            args.angle_roll,
            args.angle_pitch,
            args.angle_yaw,
            args.angular_velocity_roll,
            args.angular_velocity_pitch,
            args.angular_velocity_yaw,
            args.acc_x,
            args.acc_y,
            args.acc_z,
        };
    }

    pub fn set_joint(self: *Self, args: struct { index: u8, position: f32, velocity: f32, torque: f32, temperature: f32 }) !void {
        const index = try jointIndex(args.index);
        self.value.joint_data.joint_data[index] = .{
            .position = args.position,
            .velocity = args.velocity,
            .torque = args.torque,
            .temperature = args.temperature,
        };
    }

    pub fn joint(self: *const Self, args: struct { index: u8 }) !JointDataSnapshot {
        const index = try jointIndex(args.index);
        return jointDataSnapshot(self.value.joint_data.joint_data[index]);
    }

    pub fn joints(self: *const Self) !py.PyList(root) {
        return jointsListFromData(&self.value);
    }

    pub fn set_contact_force(self: *Self, args: struct { index: u8, force: f64 }) !void {
        const index = try jointIndex(args.index);
        self.value.contact_force.leg_force[index] = args.force;
    }

    pub fn contact_force(self: *const Self, args: struct { index: u8 }) !f64 {
        const index = try jointIndex(args.index);
        return self.value.contact_force.leg_force[index];
    }

    pub fn contact_forces(self: *const Self) !py.PyList(root) {
        return contactForcesList(&self.value);
    }

    pub fn copy(self: *const Self) !*RobotData.definition {
        return py.init(root, RobotData.definition, .{ .value = self.value });
    }
});

pub const Timer = py.class(struct {
    pub const __doc__ = "Wrapper around the MotionSDK DRTimer.";

    const Self = @This();

    handle: ?*DRTimerHandle = null,

    pub fn __init__(self: *Self, args: struct { ms: ?i32 = null }) !void {
        const handle = try requireCreated(DRTimerHandle, DRTimer_create(), "Timer");
        self.handle = handle;
        if (args.ms) |ms| {
            DRTimer_init(handle, @intCast(ms));
        }
    }

    pub fn close(self: *Self) void {
        if (self.handle) |handle| {
            DRTimer_destroy(handle);
            self.handle = null;
        }
    }

    pub fn __del__(self: *Self) void {
        self.close();
    }

    pub fn init(self: *Self, args: struct { ms: i32 }) !void {
        const handle = try requireHandle(DRTimerHandle, self.handle, "Timer");
        DRTimer_init(handle, @intCast(args.ms));
    }

    pub fn interrupt(self: *Self) !bool {
        const handle = try requireHandle(DRTimerHandle, self.handle, "Timer");
        return DRTimer_interrupt(handle);
    }

    pub fn current_time(self: *Self) !f64 {
        const handle = try requireHandle(DRTimerHandle, self.handle, "Timer");
        return DRTimer_getCurrentTime(handle);
    }

    pub fn interval_time(self: *Self, args: struct { start_time: f64 }) !f64 {
        const handle = try requireHandle(DRTimerHandle, self.handle, "Timer");
        return DRTimer_getIntervalTime(handle, args.start_time);
    }
});

pub const Sender = py.class(struct {
    pub const __doc__ = "UDP command sender for Lite3 joint commands.";

    const Self = @This();

    handle: ?*anyopaque = null,

    pub fn __init__(self: *Self, args: struct { ip: ?[]const u8 = null, port: u16 = 43893 }) !void {
        self.handle = if (args.ip) |ip| blk: {
            const ip_z = try py.allocator.dupeZ(u8, ip);
            defer py.allocator.free(ip_z);
            break :blk try requireCreatedOpaque(Sender_createWithIpPort(ip_z.ptr, args.port), "Sender");
        } else try requireCreatedOpaque(Sender_create(), "Sender");
    }

    pub fn close(self: *Self) void {
        if (self.handle) |handle| {
            Sender_destroy(handle);
            self.handle = null;
        }
    }

    pub fn __del__(self: *Self) void {
        self.close();
    }

    pub fn send_cmd(self: *Self, args: struct { cmd: *RobotCmd.definition }) !void {
        const handle = try requireOpaqueHandle(self.handle, "Sender");
        const nogil = py.nogil();
        defer nogil.acquire();
        Sender_sendCmd(handle, &args.cmd.value);
    }

    pub fn control_get(self: *Self, args: struct { mode: u32 }) !void {
        const handle = try requireOpaqueHandle(self.handle, "Sender");
        const nogil = py.nogil();
        defer nogil.acquire();
        Sender_controlGet(handle, args.mode);
    }

    pub fn return_control(self: *Self) !void {
        const handle = try requireOpaqueHandle(self.handle, "Sender");
        const nogil = py.nogil();
        defer nogil.acquire();
        Sender_controlGet(handle, robot_mode);
    }

    pub fn request_sdk_control(self: *Self) !void {
        const handle = try requireOpaqueHandle(self.handle, "Sender");
        const nogil = py.nogil();
        defer nogil.acquire();
        Sender_controlGet(handle, sdk_mode);
    }

    pub fn all_joint_back_zero(self: *Self) !void {
        const handle = try requireOpaqueHandle(self.handle, "Sender");
        const nogil = py.nogil();
        defer nogil.acquire();
        Sender_allJointBackZero(handle);
    }

    pub fn robot_state_init(self: *Self) !void {
        const handle = try requireOpaqueHandle(self.handle, "Sender");
        const nogil = py.nogil();
        defer nogil.acquire();
        Sender_robotStateInit(handle);
    }

    pub fn set_cmd(self: *Self, args: struct { code: u32, value: u32 }) !void {
        const handle = try requireOpaqueHandle(self.handle, "Sender");
        const nogil = py.nogil();
        defer nogil.acquire();
        Sender_setCmd(handle, args.code, args.value);
    }
});

pub const Receiver = py.class(struct {
    pub const __doc__ = "UDP receiver for Lite3 robot-state packets.";

    const Self = @This();

    handle: ?*ReceiverHandle = null,
    update_flag: u8 = 0,

    pub fn __init__(self: *Self) !void {
        self.handle = try requireCreated(ReceiverHandle, Receiver_create(), "Receiver");
        self.update_flag = 0;
    }

    pub fn close(self: *Self) void {
        if (self.handle) |handle| {
            Receiver_destroy(handle);
            self.handle = null;
        }
    }

    pub fn __del__(self: *Self) void {
        self.close();
    }

    pub fn start_work(self: *Self) !void {
        const handle = try requireHandle(ReceiverHandle, self.handle, "Receiver");
        Receiver_startWork(handle);
    }

    pub fn register_update_flag(self: *Self) !void {
        const handle = try requireHandle(ReceiverHandle, self.handle, "Receiver");
        Receiver_registerCallback(handle, receiverUpdateCallback, @ptrCast(&self.update_flag));
    }

    pub fn message_updated(self: *Self) bool {
        return @atomicLoad(u8, &self.update_flag, .acquire) != 0;
    }

    pub fn clear_update_flag(self: *Self) void {
        @atomicStore(u8, &self.update_flag, 0, .release);
    }

    pub fn get_state(self: *Self) !*RobotData.definition {
        const handle = try requireHandle(ReceiverHandle, self.handle, "Receiver");
        const state = Receiver_getState(handle) orelse return py.RuntimeError(root).raise("Receiver returned null RobotData");
        return py.init(root, RobotData.definition, .{ .value = state.* });
    }
});

pub const Command = py.class(struct {
    pub const __doc__ = "Wrapper around the SDK Command object.";

    const Self = @This();

    handle: ?*anyopaque = null,
    owned_parameters: ?[]u8 = null,

    pub fn __init__(self: *Self, args: struct { code: u32, value: ?i32 = null, parameters: ?py.PyBytes = null }) !void {
        if (args.value != null and args.parameters != null) {
            return py.ValueError(root).raise("pass either value or parameters, not both");
        }

        if (args.parameters) |parameter_object| {
            const parameter_bytes = try parameter_object.asSlice();
            const owned = try py.allocator.dupe(u8, parameter_bytes);
            self.owned_parameters = owned;
            self.handle = try requireCreatedOpaque(Command_CreateWithParameters(args.code, owned.len, @ptrCast(owned.ptr)), "Command");
        } else {
            self.handle = try requireCreatedOpaque(Command_CreateWithValue(args.code, args.value orelse 0), "Command");
        }
    }

    pub fn close(self: *Self) void {
        if (self.handle) |handle| {
            Command_Destroy(handle);
            self.handle = null;
        }
        if (self.owned_parameters) |parameter_bytes| {
            py.allocator.free(parameter_bytes);
            self.owned_parameters = null;
        }
    }

    pub fn __del__(self: *Self) void {
        self.close();
    }

    pub fn code(self: *Self) !u32 {
        const handle = try requireOpaqueHandle(self.handle, "Command");
        return Command_GetCode(handle);
    }

    pub fn value(self: *Self) !i32 {
        const handle = try requireOpaqueHandle(self.handle, "Command");
        return Command_GetValue(handle);
    }

    pub fn parameters_size(self: *Self) !usize {
        const handle = try requireOpaqueHandle(self.handle, "Command");
        return Command_GetParametersSize(handle);
    }

    pub fn parameters(self: *Self) !py.PyBytes {
        const handle = try requireOpaqueHandle(self.handle, "Command");
        const size = Command_GetParametersSize(handle);
        if (size == 0) {
            return py.PyBytes.create("");
        }
        const ptr = Command_GetParameters(handle) orelse return py.RuntimeError(root).raise("Command returned null parameters");
        const bytes: [*]const u8 = @ptrCast(ptr);
        return py.PyBytes.create(bytes[0..size]);
    }
});


pub fn cubic_spline(args: struct {
    init_position: f64,
    init_velocity: f64,
    goal_position: f64,
    goal_velocity: f64,
    run_time: f64,
    cycle_time: f64,
    total_time: f64,
}) !CubicSplineResult {
    return cubicSplineValues(args.init_position, args.init_velocity, args.goal_position, args.goal_velocity, args.run_time, args.cycle_time, args.total_time);
}

pub fn record_initial_data(args: struct { data: *RobotData.definition, time: f64 }) void {
    recordInitialData(&args.data.value, args.time);
}

pub fn pre_stand_up(args: struct { cmd: *RobotCmd.definition, time: f64, data: *RobotData.definition }) !void {
    _ = args.data;
    try fillPreStandUpCommand(&args.cmd.value, args.time);
}

pub fn stand_up(args: struct { cmd: *RobotCmd.definition, time: f64, data: *RobotData.definition }) !void {
    _ = args.data;
    try fillStandUpCommand(&args.cmd.value, args.time);
}

comptime {
    @setEvalBranchQuota(100000);
    py.rootmodule(root);
}
