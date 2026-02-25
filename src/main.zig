pub fn main() !void {
    var da: heap.DebugAllocator(.{}) = .init;
    defer _ = da.detectLeaks();

    const allocator = da.allocator();

    var physics = try HavokPhysics.init(allocator);
    defer physics.deinit();

    {
        defer physics.free();

        const result, const world_id = physics.world.create();

        log.info("{any}, {any}", .{ result, world_id });

        _, const shape_id_capsule = physics.shape.createCapsule(.{ 1, 1, 1 }, .{ 5, 5, 5 }, 10);

        _ = physics.shape.setFilterInfo(shape_id_capsule, .{ 2, 2 });
    }
}

const std = @import("std");
const log = std.log;
const heap = std.heap;

const wamr = @import("wamr").wasm_export;

const HavokPhysics = @import("HavokPhysics.zig");
