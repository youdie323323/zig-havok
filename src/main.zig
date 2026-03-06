pub fn main() !void {
    var da: heap.DebugAllocator(.{}) = .init;
    defer _ = da.detectLeaks();

    const allocator = da.allocator();

    var physics = try HavokPhysics.init(allocator);
    defer physics.deinit();

    {
        defer physics.free();

        {
            const result, const world_id = physics.world.create();

            log.info("{any}, {any}", .{ result, world_id });
        }

        {
            const result, const sphere_shape_id = physics.shape.createSphere(.{ 5, 5, 5 }, 0.5);

            log.info("{any}, {any}", .{ result, sphere_shape_id });

            log.info("{any}", .{physics.shape.setFilterInfo(sphere_shape_id, .{ 1, 1 })});
            log.info("{any}", .{physics.shape.getFilterInfo(sphere_shape_id)});
        }
    }
}

const std = @import("std");
const log = std.log;
const heap = std.heap;

const wamr = @import("wamr").wasm_export;

const HavokPhysics = @import("HavokPhysics.zig");
