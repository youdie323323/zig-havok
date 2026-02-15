pub fn main() !void {
    var da: heap.DebugAllocator(.{}) = .{};
    defer _ = da.detectLeaks();

    const allocator = da.allocator();

    var physics = HavokPhysics.init(allocator) catch |err| {
        log.err("failed to initialize physics: {}", .{err});

        return;
    };
    defer physics.deinit();

    _ = physics.start() catch |err| {
        log.err("failed to start: {}", .{err});

        return;
    };

    _ = physics.shape.createSphere(.{ 1, 1, 1 }, 10);
}

const std = @import("std");
const log = std.log;
const heap = std.heap;

const wamr = @import("wamr").wasm_export;

const HavokPhysics = @import("HavokPhysics.zig");
