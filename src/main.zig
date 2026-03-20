const dt: Float = 1.0 / 60.0;
const dt_nanoseconds: u64 = @intFromFloat(time.ns_per_s * dt);

pub fn main() !void {
    var da: heap.DebugAllocator(.{}) = .init;

    const allocator = da.allocator();

    var physics = try Physics.init(allocator);

    defer {
        physics.deinit();

        _ = da.detectLeaks();
    }

    {
        defer physics.free();

        _, const world_id = physics.world.create();
        defer _ = physics.world.release(world_id);

        _ = physics.world.setIdealStepTime(world_id, dt);
        _ = physics.world.setGravity(world_id, .{ 0, -9.81, 0 });

        _, const box_id = physics.shape.createBox(.{ 0, 0, 0 }, .{ 0, 0, 0, 1 }, .{ 0.25, 1, 0.5 });
        defer _ = physics.shape.release(box_id);

        _ = physics.shape.setDensity(box_id, 1);

        _, const body_id = physics.body.create();
        defer _ = physics.body.release(body_id);

        _ = physics.body.setShape(body_id, box_id);

        _, const mass_properties = physics.shape.buildMassProperties(box_id);
        _ = physics.body.setMassProperties(body_id, mass_properties);

        _ = physics.body.setMotionType(body_id, .dynamic);

        _ = physics.body.setGravityFactor(body_id, 1);

        _ = physics.body.setPosition(body_id, .{ 100, 100, 100 });

        _ = physics.world.addBody(world_id, body_id, false);

        for (0..100) |i| {
            _ = physics.world.step(world_id, dt);

            _, const box_position = physics.body.getPosition(body_id);

            log.debug("Step {d:3}: Position = ({d:>6.2}, {d:>6.2}, {d:>6.2})", .{
                i, box_position[0], box_position[1], box_position[2],
            });
        }
    }
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
