const dt: Float = 1.0 / 60.0;
const dt_nanoseconds: u64 = @intFromFloat(dt * time.ns_per_s);

pub fn main() !void {
    var timer = try Timer.start();

    const start_time = timer.read();

    {
        var da: heap.DebugAllocator(.{}) = .init;

        const allocator = da.allocator();

        var physics = try Physics.init(allocator);

        defer {
            physics.deinit();

            _ = da.detectLeaks();
        }

        for (0..10000) |_| {
            defer physics.free();

            _, const world_id = physics.world.create();
            defer _ = physics.world.release(world_id);

            { // Setup world
                _ = physics.world.setIdealStepTime(world_id, dt);
                _ = physics.world.getBodyBuffer(world_id);
                _ = physics.world.setGravity(world_id, .{ 0, -9.807, 0 });
                _ = physics.world.getNumBodies(world_id);
            }

            _, const sphere_id = physics.shape.createSphere(.{ 5, 5, 5 }, 0.5);
            defer _ = physics.shape.release(sphere_id);

            { // Setup sphere
                _ = physics.shape.setFilterInfo(sphere_id, .{ 1, 1 });
                _ = physics.shape.getFilterInfo(sphere_id);

                _ = physics.shape.setMaterial(sphere_id, .{ 0.75, 0.75, 0, .artihemic_mean, .artihemic_mean });
                _ = physics.shape.getMaterial(sphere_id);

                _ = physics.shape.setDensity(sphere_id, 0.5);
                _ = physics.shape.getDensity(sphere_id);

                inline for (0..5) |_| {
                    _, const debug_geometry_id = physics.debug_geometry.create(sphere_id);

                    _ = physics.debug_geometry.release(debug_geometry_id);
                }

                _, const container_id = physics.shape.createContainer();
                defer _ = physics.shape.release(container_id);

                _ = physics.shape.addChild(container_id, sphere_id, .{
                    .{ 0, 0, 0 },
                    .{ 0, 0, 0, 1 },
                    .{ 1, 1, 1 },
                });

                _ = physics.shape.getNumChildren(container_id);

                _, const body_id = physics.body.create();
                defer _ = physics.body.release(body_id);

                _ = physics.body.setShape(body_id, container_id);
            }
        }
    }

    const elapsed_ns = timer.read() - start_time;
    const elapsed_ms = @as(f64, @floatFromInt(elapsed_ns)) / 1_000_000.0;

    debug.print("Setup and Teardown Elapsed Time: {d:.4} ms\n", .{elapsed_ms});
}

const std = @import("std");
const log = std.log;
const heap = std.heap;
const time = std.time;
const debug = std.debug;
const Timer = time.Timer;

const Physics = @import("havok").Physics;
const Result = Physics.Result;
const Body = Physics.Body;
const Float = Physics.Float;
