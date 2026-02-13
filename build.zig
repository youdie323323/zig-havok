const std = @import("std");

pub fn build(b: *std.Build) void {
    const is_production = b.option(
        bool,
        "production",
        "Enable production build",
    ) orelse false;

    const target = b.standardTargetOptions(.{
        .default_target = .{
            .abi = .msvc,
        },
    });

    const exe = b.addExecutable(.{
        .name = "havok",
        .root_module = b.createModule(
            if (is_production) .{
                .root_source_file = b.path("src/main.zig"),
                .target = target,
                .optimize = .ReleaseFast,
                .strip = true,
            } else .{
                .root_source_file = b.path("src/main.zig"),
                .target = target,
                .optimize = .Debug,
            },
        ),
    });

    {
        const wamr_dep = b.dependency("wamr", .{
            .target = target,
            .optimize = .ReleaseFast,
        });

        exe.root_module.addImport("wamr", wamr_dep.module("wamr"));

        exe.step.dependOn(wamr_dep.builder.getInstallStep());
    }

    b.installArtifact(exe);
}
