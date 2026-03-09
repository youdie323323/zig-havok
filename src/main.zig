const dt: Float = 1.0 / 60.0;
const dt_nanoseconds: u64 = @intFromFloat(dt * time.ns_per_s);

pub fn main() !void {
    var da: heap.DebugAllocator(.{}) = .init;
    defer _ = da.detectLeaks();

    const allocator = da.allocator();

    var physics = try Physics.init(allocator);
    defer physics.deinit();

    {
        defer physics.free();

        const result_1, const world_id = physics.world.create();

        {
            log.info("a {any}", .{physics.world.setIdealStepTime(world_id, dt)});

            log.info("b {any}, {any}", .{ result_1, world_id });

            log.info("c {any}", .{physics.world.getBodyBuffer(world_id)});

            log.info("d {any}", .{physics.world.setGravity(world_id, .{ 0, -9.807, 0 })});

            log.info("e {any}", .{physics.world.getNumBodies(world_id)});
        }

        const result_2, const sphere_id = physics.shape.createSphere(.{ 5, 5, 5 }, 0.5);
        defer _ = physics.shape.release(sphere_id);

        {
            log.info("f {any}, {any}", .{ result_2, sphere_id });

            log.info("g {any}", .{physics.shape.setFilterInfo(sphere_id, .{ 1, 1 })});
            log.info("h {any}", .{physics.shape.getFilterInfo(sphere_id)});

            log.info("i {any}", .{physics.shape.setMaterial(sphere_id, .{ 0.75, 0.75, 0, .artihemic_mean, .artihemic_mean })});
            log.info("j {any}", .{physics.shape.getMaterial(sphere_id)});

            log.info("k {any}", .{physics.shape.setDensity(sphere_id, 0.5)});
            log.info("l {any}", .{physics.shape.getDensity(sphere_id)});

            inline for (0..5) |_| {
                _, const debug_geometry_id = physics.debug_geometry.create(sphere_id);
                defer _ = physics.debug_geometry.release(debug_geometry_id);

                log.info("m {any}, {any}", .{ debug_geometry_id, physics.debug_geometry.getInfo(debug_geometry_id) });
            }

            const result_3, const container_id = physics.shape.createContainer();
            defer _ = physics.shape.release(container_id);

            log.info("n {any}, {any}", .{ result_3, container_id });

            log.info("o {any}", .{physics.shape.addChild(container_id, sphere_id, .{
                .{ 0, 0, 0 },
                .{ 0, 0, 0, 1 },
                .{ 1, 1, 1 },
            })});

            log.info("p {any}", .{physics.shape.getNumChildren(container_id)});

            const result_4, const body_id = physics.body.create();
            defer _ = physics.body.release(body_id);

            log.info("q {any}, {any}", .{ result_4, body_id });

            log.info("r {any}", .{physics.body.setShape(body_id, container_id)});
        }

        while (true) {
            _ = physics.world.step(world_id, dt);

            log.info("s {any}", .{physics.getStatistics()});

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

const Physics = @import("havok").Physics;
const Result = Physics.Result;
const Body = Physics.Body;
const Float = Physics.Float;
