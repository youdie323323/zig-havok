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
            const result_1, const sphere_id = physics.shape.createSphere(.{ 5, 5, 5 }, 0.5);

            log.info("{any}, {any}", .{ result_1, sphere_id });

            log.info("{any}", .{physics.shape.setFilterInfo(sphere_id, .{ 1, 1 })});
            log.info("{any}", .{physics.shape.getFilterInfo(sphere_id)});

            // log.info("{any}", .{physics.shape.setMaterial(sphere_shape_id, .{ 1, 1 })});
            log.info("{any}", .{physics.shape.getMaterial(sphere_id)});

            log.info("{any}", .{physics.shape.setDensity(sphere_id, 0.5)});
            log.info("{any}", .{physics.shape.getDensity(sphere_id)});

            const result_2, const container = physics.shape.createContainer();

            log.info("{any}, {any}", .{ result_2, container });

            const result_3 = physics.shape.addChild(container, sphere_id, .{
                .{ 0, 0, 0 },
                .{ 0, 0, 0, 1 },
                .{ 1, 1, 1 },
            });

            log.info("{any}", .{result_3});
        }
    }
}

const std = @import("std");
const log = std.log;
const heap = std.heap;

const wamr = @import("wamr").wasm_export;

const HavokPhysics = @import("HavokPhysics.zig");
