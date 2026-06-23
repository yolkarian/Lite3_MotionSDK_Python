const std = @import("std");
const py = @import("./pydust.build.zig");

const Platform = enum {
    x86_64,
    aarch64,
};

const RuntimeLibs = struct {
    sdk: []const u8,
    wrapped_sdk: []const u8,
};

fn platformFromTarget(target: std.Build.ResolvedTarget) Platform {
    if (target.result.os.tag != .linux) {
        std.debug.panic("unsupported target OS '{s}', expected linux", .{@tagName(target.result.os.tag)});
    }

    return switch (target.result.cpu.arch) {
        .x86_64 => .x86_64,
        .aarch64 => .aarch64,
        else => |arch| std.debug.panic("unsupported target architecture '{s}', expected x86_64 or aarch64", .{@tagName(arch)}),
    };
}

fn runtimeLibsForPlatform(platform: Platform) RuntimeLibs {
    return switch (platform) {
        .x86_64 => .{
            .sdk = "libdeeprobotics_legged_sdk_x86_64.so",
            .wrapped_sdk = "libdeeprobotics_legged_wrapped_sdk_x86_64.so",
        },
        .aarch64 => .{
            .sdk = "libdeeprobotics_legged_sdk_aarch64.so",
            .wrapped_sdk = "libdeeprobotics_legged_wrapped_sdk_aarch64.so",
        },
    };
}

fn linkRuntimeLibs(step: *std.Build.Step.Compile, platform: Platform, add_source_tree_rpath: bool) void {
    const b = step.root_module.owner;
    const libs = runtimeLibsForPlatform(platform);

    step.root_module.addObjectFile(b.path(b.fmt("lite3_motion_sdk/{s}", .{libs.wrapped_sdk})));
    step.root_module.addObjectFile(b.path(b.fmt("lite3_motion_sdk/{s}", .{libs.sdk})));

    // The extension is installed as lite3_motion_sdk/_lib.abi3.so and the
    // DeepRobotics runtime libraries are packaged in that same directory.
    step.root_module.addRPathSpecial("$ORIGIN");

    // Zig test binaries live in zig-out/bin, so add the source package
    // directory as an extra development-time runpath for `zig build test`.
    if (add_source_tree_rpath) {
        step.root_module.addRPath(b.path("lite3_motion_sdk"));
    }
}

pub fn build(b: *std.Build) void {
    const target_query = b.standardTargetOptionsQueryOnly(.{});
    const target = b.resolveTargetQuery(target_query);
    const platform = platformFromTarget(target);
    const optimize = b.standardOptimizeOption(.{});

    const test_step = b.step("test", "Run Zig unit tests");
    const pydust = py.addPydust(b, .{
        .test_step = test_step,
    });

    const module = pydust.addPythonModule(.{
        .name = "lite3_motion_sdk._lib",
        .root_source_file = b.path("src/lite3_motion_sdk.zig"),
        .limited_api = true,
        .target = target_query,
        .optimize = optimize,
    });

    linkRuntimeLibs(module.library_step, platform, false);
    linkRuntimeLibs(module.test_step, platform, true);
}
