const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{
        .default_target = .{
            .abi = .msvc,
        },
    });
    const optimize = b.standardOptimizeOption(.{ .preferred_optimize_mode = .ReleaseFast });

    const mod = b.addModule("havok", .{
        .root_source_file = b.path("src/root.zig"),
        .target = target,
        .optimize = optimize,
    });

    const exe = b.addExecutable(.{
        .name = "havok",
        .root_module = b.createModule(.{ // TODO: with strip enabled, calling `wasm_runtime_load` breaks program
            .root_source_file = b.path("src/main.zig"),
            .target = target,
            .optimize = optimize,
            .imports = &.{
                .{ .name = "havok", .module = mod },
            },
        }),
    });

    // Enable thin LTO
    exe.lto = .thin;

    { // Install the wamr dependency
        const wamr_dep = b.dependency("wamr", .{
            .target = target,
            .optimize = optimize,
        });

        mod.addImport("wamr", wamr_dep.module("wamr"));

        exe.step.dependOn(wamr_dep.builder.getInstallStep());
    }

    b.installArtifact(exe);
}
