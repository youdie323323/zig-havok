pub fn main() !void {
    var da: heap.DebugAllocator(.{}) = .init;
    defer _ = da.detectLeaks();

    const allocator = da.allocator();

    var physics = try HavokPhysics.init(allocator);
    defer physics.deinit();

    {
        defer physics.free();

        {
            const result_1, const world_id = physics.world.create();
            defer _ = physics.world.release(world_id);

            log.info("{any}, {any}", .{ result_1, world_id });

            log.info("{any}", .{physics.world.getBodyBuffer(world_id)});
        }

        {
            const result_1, const sphere_id = physics.shape.createSphere(.{ 5, 5, 5 }, 0.5);
            defer _ = physics.shape.release(sphere_id);

            log.info("{any}, {any}", .{ result_1, sphere_id });

            log.info("{any}", .{physics.shape.setFilterInfo(sphere_id, .{ 1, 1 })});
            log.info("{any}", .{physics.shape.getFilterInfo(sphere_id)});

            // log.info("{any}", .{physics.shape.setMaterial(sphere_shape_id, .{ 1, 1 })});
            log.info("{any}", .{physics.shape.getMaterial(sphere_id)});

            log.info("{any}", .{physics.shape.setDensity(sphere_id, 0.5)});
            log.info("{any}", .{physics.shape.getDensity(sphere_id)});

            log.info("{any}", .{physics.shape.createDebugDisplayGeometry(sphere_id)});

            const result_2, const container_id = physics.shape.createContainer();
            defer _ = physics.shape.release(container_id);

            log.info("{any}, {any}", .{ result_2, container_id });

            const result_3 = physics.shape.addChild(container_id, sphere_id, .{
                .{ 0, 0, 0 },
                .{ 0, 0, 0, 1 },
                .{ 1, 1, 1 },
            });

            log.info("{any}", .{result_3});

            const result_4, const container_num_children = physics.shape.getNumChildren(container_id);

            log.info("{any}, {d}", .{ result_4, container_num_children });
        }
    }
}

const std = @import("std");
const log = std.log;
const heap = std.heap;

const wamr = @import("wamr").wasm_export;

const HavokPhysics = @import("HavokPhysics.zig");
