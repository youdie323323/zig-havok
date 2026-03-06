const dt: Float = 1.0 / 20.0;
const dt_nanoseconds: u64 = @intFromFloat(dt * time.ns_per_s);

pub fn main() !void {
    var da: heap.DebugAllocator(.{}) = .init;
    defer _ = da.detectLeaks();

    const allocator = da.allocator();

    var physics = try HavokPhysics.init(allocator);
    defer physics.deinit();

    {
        defer physics.free();

        const result_1, const world_id = physics.world.create();

        {
            log.info("{any}", .{physics.world.setIdealStepTime(world_id, dt)});

            log.info("{any}, {any}", .{ result_1, world_id });

            log.info("{any}", .{physics.world.getBodyBuffer(world_id)});

            log.info("{any}", .{physics.world.setGravity(world_id, .{ 0, -9.807, 0 })});

            log.info("{any}", .{physics.world.getNumBodies(world_id)});
        }

        const result_2, const sphere_id = physics.shape.createSphere(.{ 5, 5, 5 }, 0.5);
        defer _ = physics.shape.release(sphere_id);

        {
            log.info("{any}, {any}", .{ result_2, sphere_id });

            log.info("{any}", .{physics.shape.setFilterInfo(sphere_id, .{ 1, 1 })});
            log.info("{any}", .{physics.shape.getFilterInfo(sphere_id)});

            log.info("{any}", .{physics.shape.setMaterial(sphere_id, .{ 0.75, 0.75, 0, .artihemic_mean, .artihemic_mean })});
            log.info("{any}", .{physics.shape.getMaterial(sphere_id)});

            log.info("{any}", .{physics.shape.setDensity(sphere_id, 0.5)});
            log.info("{any}", .{physics.shape.getDensity(sphere_id)});

            log.info("{any}", .{physics.shape.createDebugDisplayGeometry(sphere_id)});

            const result_3, const container_id = physics.shape.createContainer();
            defer _ = physics.shape.release(container_id);

            log.info("{any}, {any}", .{ result_3, container_id });

            log.info("{any}", .{physics.shape.addChild(container_id, sphere_id, .{
                .{ 0, 0, 0 },
                .{ 0, 0, 0, 1 },
                .{ 1, 1, 1 },
            })});

            log.info("{any}", .{physics.shape.getNumChildren(container_id)});
        }

        while (true) {
            _ = physics.world.step(world_id, dt);

            Thread.sleep(dt_nanoseconds);
        }
    }
}

const std = @import("std");
const log = std.log;
const heap = std.heap;
const time = std.time;
const Thread = std.Thread;

const wamr = @import("wamr").wasm_export;

const HavokPhysics = @import("HavokPhysics.zig");
const Float = HavokPhysics.Float;
