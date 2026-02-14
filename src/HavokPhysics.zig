const HavokPhysics = @This();

const Emscripten = struct {
    pub const Bind = struct {
        pub const Type = struct {
            pub const Id = i32;

            pub const Kind = enum {
                void,
                bool,
                int,
                float,
                bigint,
                std_string,
                std_wstring,
                @"enum",
                tuple,
                emval,

                memory_view,
            };

            pub const Instance = struct {
                pub const Initializer = *const fn (...) callconv(.c) void;

                name: []const u8,
                kind: Kind,

                destructor: ?Initializer = null,
                /// Describes if destructor gets the first argument as *HavokPhysics.
                is_destructor_wasm: bool = false,

                size: ?u32 = null,

                is_signed: ?bool = null,

                true_value: ?u32 = null,
                false_value: ?u32 = null,

                data_type: ?u8 = null,

                tuple_constructor: ?Initializer = null,
                tuple_elements: ?Tuple.Elements = null,
                tuple_converters: ?Converters = null,

                pub const emval: *const Instance = &.{
                    .name = "emscripten::val",
                    .kind = .emval,
                };

                const void_wire_inner = 0;
                const void_wire: *const anyopaque = @ptrCast(&void_wire_inner);

                const false_wire_inner = false;
                const false_wire: *const anyopaque = @ptrCast(&false_wire_inner);

                const true_wire_inner = true;
                const true_wire: *const anyopaque = @ptrCast(&true_wire_inner);

                pub fn fromWire(self: *const Instance, wire: u32) *const anyopaque {
                    return switch (self.kind) {
                        .void => void_wire,
                        .bool => if (wire != 0)
                            true_wire
                        else
                            false_wire,
                        else => unreachable,
                    };
                }

                pub fn toWire(self: *const Instance, val: *const anyopaque) u32 {
                    return switch (self.kind) {
                        .void => 0,
                        .bool => if (@as(*const bool, @ptrCast(@alignCast(val))).*)
                            self.true_value.?
                        else
                            self.false_value.?,
                        else => unreachable,
                    };
                }
            };

            pub const Registry = std.AutoHashMap(Id, *const Instance);

            pub const Converters = []*const Instance;

            pub const ConvertersHandler = *const fn (
                physics: *HavokPhysics,
                context: *anyopaque,
                converters: Converters,
            ) Converters;
        };

        pub const Tuple = struct {
            pub const Element = struct {
                getter_return_type: i32,
                getter: wamr.wasm_function_inst_t,
                getter_context: i32,

                setter_arg_type: i32,
                setter: wamr.wasm_function_inst_t,
                setter_context: i32,
            };

            pub const Elements = array_list.Managed(Element);

            name: []u8,

            constructor: wamr.wasm_function_inst_t,
            destructor: wamr.wasm_function_inst_t,

            elements: Elements,
        };

        pub const TupleRegistry = std.AutoHashMap(Type.Id, Tuple);

        pub const DependencyWaiter = struct {
            context: *anyopaque,

            i: usize,

            type_ids: []const Type.Id,

            type_converters: Type.Converters,

            type_converters_handler: Type.ConvertersHandler,
            free_type_converters_handler_return: bool,

            registered: *RegisteredCounter,
            unregistered: *RegisteredCounter,
        };

        pub const AwaitingDependencies = std.AutoHashMap(Type.Id, array_list.Managed(*DependencyWaiter));

        pub const InvokerContext = struct {
            const max_args = 8;

            invoker: wamr.wasm_function_inst_t,

            function: u32,

            return_type_instance: *const Type.Instance,

            arg_type_instances: [max_args]*const Type.Instance = undefined,

            destructors: [max_args]struct { bool, Type.Instance.Initializer } = undefined,
        };
    };

    pub const Val = struct {
        pub const @"undefined": u32 = 2;
        pub const @"null": u32 = 4;
        pub const @"true": u32 = 6;
        pub const @"false": u32 = 8;

        pub const Handle = struct {
            value: ?*anyopaque,
            ref_count: u32,
        };

        pub const Handles = array_list.Managed(Handle);

        pub const FreeList = array_list.Managed(u32);
    };
};

fn MethodImpl(comptime Return: type) type {
    return *const fn (physics: *HavokPhysics, invoker_context_index: u8, ...) callconv(.c) Return;
}

fn ExternalizeTuple(comptime Tuple: type) type {
    comptime {
        const tuple_fields = @typeInfo(Tuple).@"struct".fields;

        var fields: [tuple_fields.len]builtin.Type.StructField = undefined;

        for (tuple_fields, 0..) |field, i|
            fields[i] = .{
                .name = fmt.comptimePrint("{d}", .{i + 1}),
                .type = field.type,
                .default_value_ptr = field.default_value_ptr,
                .is_comptime = field.is_comptime,
                .alignment = field.alignment,
            };

        return @Type(.{ .@"struct" = .{
            .layout = .@"extern",
            .fields = &fields,
            .decls = &.{},
            .is_tuple = false,
        } });
    }
}

fn noopImpl(comptime Return: type) MethodImpl(Return) {
    comptime return struct {
        fn impl(_: *HavokPhysics, _: u8, ...) callconv(.c) Return {
            return mem.zeroes(Return);
        }
    }.impl;
}

/// Basic free destructor for pointer.
fn freeDesturctor(physics: *HavokPhysics, ptr: u32) callconv(.c) void {
    _ = physics.callExported("free", .{ptr}) catch unreachable;
}

const Vector3 = @Vector(3, f64);
const Quaternion = @Vector(4, f64);
const Rotation = @Vector(3, Vector3);
const QTransform = struct { Vector3, Quaternion };
const QSTransform = struct { Vector3, Quaternion, Vector3 };
const Transform = struct { Vector3, Rotation };
const AABB = struct { Vector3, Vector3 };

const BodyId = ExternalizeTuple(struct { c_longlong });
const ShapeId = ExternalizeTuple(struct { c_longlong });
const ConstraintId = ExternalizeTuple(struct { c_longlong });
const WorldId = ExternalizeTuple(struct { c_longlong });
const CollectorId = ExternalizeTuple(struct { c_longlong });
const DebugGeometryId = ExternalizeTuple(struct { c_longlong });

const ResultStatus = enum(c_int) {
    ok,
    fail,
    invalid_handle,
    invalid_args,
    not_implemented,
};

fn ReturnTypeOf(comptime @"fn": anytype) type {
    return @typeInfo(@TypeOf(@"fn")).@"fn".return_type.?;
}

pub const Shape = struct {
    physics: *HavokPhysics,

    const CreaterReturnType = struct { ResultStatus, ShapeId };

    pub fn createSphere(self: *@This(), center: Vector3, radius: f64) CreaterReturnType {
        return create_sphere_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateSphere"), &center, &radius);
    }

    pub fn createCapsule(self: *@This(), point_a: Vector3, point_b: Vector3, radius: f64) CreaterReturnType {
        return create_capsule_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateCapsule"), &point_a, &point_b, &radius);
    }

    pub fn createCylinder(self: *@This(), point_a: Vector3, point_b: Vector3, radius: f64) CreaterReturnType {
        return create_cylinder_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateCylinder"), &point_a, &point_b, &radius);
    }

    pub fn createBox(self: *@This(), center: Vector3, rotation: Quaternion, extents: Vector3) CreaterReturnType {
        return create_box_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateBox"), &center, &rotation, &extents);
    }

    pub fn createConvexHull(self: *@This(), vertices: u32, num_vertices: usize) CreaterReturnType {
        return create_convex_hull_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateConvexHull"), &vertices, &num_vertices);
    }

    pub fn createMesh(self: *@This(), vertices: u32, num_vertices: usize, triangles: u32, num_triangles: usize) CreaterReturnType {
        return create_mesh_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateMesh"), &vertices, &num_vertices, &triangles, &num_triangles);
    }

    pub fn createHeightField(self: *@This(), num_x_samples: usize, num_z_samples: usize, scale: Vector3, heights: u32) CreaterReturnType {
        return create_height_field_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateHeightField"), &num_x_samples, &num_z_samples, &scale, &heights);
    }

    pub fn createContainer(self: *@This()) CreaterReturnType {
        return create_container_impl(self.physics, comptime cached_function_indices.get("HP_Shape_CreateContainer"));
    }

    pub var create_sphere_impl = noopImpl(ExternalizeTuple(CreaterReturnType));
    pub var create_capsule_impl = noopImpl(ExternalizeTuple(CreaterReturnType));
    pub var create_cylinder_impl = noopImpl(ExternalizeTuple(CreaterReturnType));
    pub var create_box_impl = noopImpl(ExternalizeTuple(CreaterReturnType));
    pub var create_convex_hull_impl = noopImpl(ExternalizeTuple(CreaterReturnType));
    pub var create_mesh_impl = noopImpl(ExternalizeTuple(CreaterReturnType));
    pub var create_height_field_impl = noopImpl(ExternalizeTuple(CreaterReturnType));
    pub var create_container_impl = noopImpl(ExternalizeTuple(CreaterReturnType));

    pub fn release(self: *@This(), a: u32) !u32 {
        return self.physics.callExported("HP_Shape_Release", .{a});
    }

    pub fn getType(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetType", .{ a, b });
    }

    pub fn addChild(self: *@This(), a: u32, b: u32, c: u32) !u32 {
        return self.physics.callExported("HP_Shape_AddChild", .{ a, b, c });
    }
    pub fn removeChild(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_RemoveChild", .{ a, b });
    }

    pub fn getNumChildren(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetNumChildren", .{ a, b });
    }

    pub fn getChildShape(self: *@This(), a: u32, b: u32, c: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetChildShape", .{ a, b, c });
    }

    pub fn setChildQSTransform(self: *@This(), a: u32, b: u32, c: u32) !u32 {
        return self.physics.callExported("HP_Shape_SetChildQSTransform", .{ a, b, c });
    }
    pub fn getChildQSTransform(self: *@This(), a: u32, b: u32, c: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetChildQSTransform", .{ a, b, c });
    }

    pub fn setFilterInfo(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_SetFilterInfo", .{ a, b });
    }
    pub fn getFilterInfo(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetFilterInfo", .{ a, b });
    }

    pub fn setMaterial(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_SetMaterial", .{ a, b });
    }
    pub fn getMaterial(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetMaterial", .{ a, b });
    }

    pub fn setDensity(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_SetDensity", .{ a, b });
    }
    pub fn getDensity(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetDensity", .{ a, b });
    }

    pub fn getBoundingBox(self: *@This(), a: u32, b: u32, c: u32) !u32 {
        return self.physics.callExported("HP_Shape_GetBoundingBox", .{ a, b, c });
    }

    pub fn castRay(self: *@This(), a: u32, b: u32, c: u32, d: u32, e: u32) !u32 {
        return self.physics.callExported("HP_Shape_CastRay", .{ a, b, c, d, e });
    }

    pub fn buildMassProperties(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_BuildMassProperties", .{ a, b });
    }

    pub fn setTrigger(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_SetTrigger", .{ a, b });
    }

    pub fn pathIteratorGetNext(self: *@This(), a: u32, b: u32, c: u32) !u32 {
        return self.physics.callExported("HP_Shape_PathIterator_GetNext", .{ a, b, c });
    }

    pub fn createDebugDisplayGeometry(self: *@This(), a: u32, b: u32) !u32 {
        return self.physics.callExported("HP_Shape_CreateDebugDisplayGeometry", .{ a, b });
    }
};

allocator: mem.Allocator,

module: wamr.wasm_module_t = null,
module_inst: wamr.wasm_module_inst_t = null,
exec_env: wamr.wasm_exec_env_t = null,
table_inst: wamr.wasm_table_inst_t = .{},

cached_functions: [cached_function_indices.kvs.len]wamr.wasm_function_inst_t = @splat(null),
cached_indirect_functions: std.AutoHashMap(u32, wamr.wasm_function_inst_t),

emval_handles: Emscripten.Val.Handles,
emval_free_list: Emscripten.Val.FreeList,

embind_arena: heap.ArenaAllocator,
/// Allocator especially for embind. Its custom implementation is embind_arena.
embind_allocator: mem.Allocator,

embind_type_registry: Emscripten.Bind.Type.Registry,

embind_tuple_registry: Emscripten.Bind.TupleRegistry,

embind_awaiting_dependencies: Emscripten.Bind.AwaitingDependencies,

embind_invoker_contexts: [cached_function_indices.kvs.len]?*Emscripten.Bind.InvokerContext = @splat(null),

embind_invoker_function_indices: std.StringHashMap(usize),

shape: Shape,
debug_geometry: struct {
    physics: *HavokPhysics,

    // zig fmt: off

    pub fn getInfo(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_DebugGeometry_GetInfo", .{a, b}); }
    pub fn release(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_DebugGeometry_Release", .{a}); }

    // zig fmt: on
},
body: struct {
    physics: *HavokPhysics,

    // zig fmt: off

    pub fn create(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_Body_Create", .{a}); }
    pub fn release(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_Body_Release", .{a}); }

    pub fn setShape(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetShape", .{a, b}); }
    pub fn getShape(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetShape", .{a, b}); }

    pub fn setMotionType(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetMotionType", .{a, b}); }
    pub fn getMotionType(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetMotionType", .{a, b}); }

    pub fn setEventMask(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetEventMask", .{a, b}); }
    pub fn getEventMask(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetEventMask", .{a, b}); }

    pub fn setMassProperties(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetMassProperties", .{a, b}); }
    pub fn getMassProperties(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetMassProperties", .{a, b}); }

    pub fn setLinearDamping(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetLinearDamping", .{a, b}); }
    pub fn getLinearDamping(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetLinearDamping", .{a, b}); }

    pub fn setAngularDamping(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetAngularDamping", .{a, b}); }
    pub fn getAngularDamping(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetAngularDamping", .{a, b}); }

    pub fn setGravityFactor(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetGravityFactor", .{a, b}); }
    pub fn getGravityFactor(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetGravityFactor", .{a, b}); }

    pub fn getWorld(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetWorld", .{a, b}); }
    pub fn getWorldTransformOffset(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetWorldTransformOffset", .{a, b}); }

    pub fn setPosition(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetPosition", .{a, b}); }
    pub fn getPosition(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetPosition", .{a, b}); }

    pub fn setOrientation(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetOrientation", .{a, b}); }
    pub fn getOrientation(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetOrientation", .{a, b}); }

    pub fn setQTransform(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetQTransform", .{a, b}); }
    pub fn getQTransform(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetQTransform", .{a, b}); }

    pub fn setTargetQTransform(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetTargetQTransform", .{a, b}); }

    pub fn setLinearVelocity(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetLinearVelocity", .{a, b}); }
    pub fn getLinearVelocity(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetLinearVelocity", .{a, b}); }

    pub fn setAngularVelocity(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetAngularVelocity", .{a, b}); }
    pub fn getAngularVelocity(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetAngularVelocity", .{a, b}); }

    pub fn applyImpulse(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Body_ApplyImpulse", .{a, b, c}); }
    pub fn applyAngularImpulse(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_ApplyAngularImpulse", .{a, b}); }

    pub fn setActivationState(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetActivationState", .{a, b}); }
    pub fn getActivationState(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_GetActivationState", .{a, b}); }

    pub fn setActivationControl(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetActivationControl", .{a, b}); }
    pub fn setActivationPriority(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Body_SetActivationPriority", .{a, b}); }

    // zig fmt: on
},
constraint: struct {
    physics: *HavokPhysics,

    // zig fmt: off

    pub fn create(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_Constraint_Create", .{a}); }
    pub fn release(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_Constraint_Release", .{a}); }

    pub fn setParentBody(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_SetParentBody", .{a, b}); }
    pub fn getParentBody(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_GetParentBody", .{a, b}); }

    pub fn setChildBody(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_SetChildBody", .{a, b}); }
    pub fn getChildBody(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_GetChildBody", .{a, b}); }

    pub fn setAnchorInParent(self: *@This(), a: u32, b: u32, c: u32, d: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAnchorInParent", .{a, b, c, d}); }
    pub fn setAnchorInChild(self: *@This(), a: u32, b: u32, c: u32, d: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAnchorInChild", .{a, b, c, d}); }

    pub fn setCollisionsEnabled(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_SetCollisionsEnabled", .{a, b}); }
    pub fn getCollisionsEnabled(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_GetCollisionsEnabled", .{a, b}); }
    
    pub fn getAppliedImpulses(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAppliedImpulses", .{a, b, c}); }

    pub fn setEnabled(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_SetEnabled", .{a, b}); }
    pub fn getEnabled(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Constraint_GetEnabled", .{a, b}); }

    pub fn setAxisMinLimit(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMinLimit", .{a, b, c}); }
    pub fn getAxisMinLimit(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMinLimit", .{a, b, c}); }

    pub fn setAxisMaxLimit(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMaxLimit", .{a, b, c}); }
    pub fn getAxisMaxLimit(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMaxLimit", .{a, b, c}); }

    pub fn setAxisMode(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMode", .{a, b, c}); }
    pub fn getAxisMode(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMode", .{a, b, c}); }
    
    pub fn setAxisFriction(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisFriction", .{a, b, c}); }
    pub fn getAxisFriction(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisFriction", .{a, b, c}); }

    pub fn setAxisMotorType(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMotorType", .{a, b, c}); }
    pub fn getAxisMotorType(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMotorType", .{a, b, c}); }

    pub fn setAxisMotorPositionTarget(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMotorPositionTarget", .{a, b, c}); }
    pub fn getAxisMotorPositionTarget(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMotorPositionTarget", .{a, b, c}); }

    pub fn setAxisMotorVelocityTarget(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMotorVelocityTarget", .{a, b, c}); }
    pub fn getAxisMotorVelocityTarget(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMotorVelocityTarget", .{a, b, c}); }

    pub fn setAxisMotorMaxForce(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMotorMaxForce", .{a, b, c}); }
    pub fn getAxisMotorMaxForce(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMotorMaxForce", .{a, b, c}); }

    pub fn setAxisMotorStiffness(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMotorStiffness", .{a, b, c}); }
    pub fn getAxisMotorStiffness(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMotorStiffness", .{a, b, c}); }

    pub fn setAxisMotorDamping(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMotorDamping", .{a, b, c}); }
    pub fn getAxisMotorDamping(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMotorDamping", .{a, b, c}); }

    pub fn setAxisMotorTarget(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisMotorTarget", .{a, b, c}); }
    pub fn getAxisMotorTarget(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_GetAxisMotorTarget", .{a, b, c}); }

    pub fn setAxisStiffness(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisStiffness", .{a, b, c}); }
    pub fn setAxisDamping(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_Constraint_SetAxisDamping", .{a, b, c}); }

    // zig fmt: on
},
world: struct {
    physics: *HavokPhysics,

    // zig fmt: off

    pub fn create(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_World_Create", .{a}); }
    pub fn release(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_World_Release", .{a}); }

    pub fn getBodyBuffer(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_GetBodyBuffer", .{a, b}); }

    pub fn setGravity(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_SetGravity", .{a, b}); }
    pub fn getGravity(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_GetGravity", .{a, b}); }

    pub fn addBody(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_AddBody", .{a, b, c}); }
    pub fn removeBody(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_RemoveBody", .{a, b}); }

    pub fn getNumBodies(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_GetNumBodies", .{a, b}); }

    pub fn castRay(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_CastRay", .{a, b, c}); }
    pub fn castRayWithCollector(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_CastRayWithCollector", .{a, b, c}); }
    pub fn pointProximityWithCollector(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_PointProximityWithCollector", .{a, b, c}); }
    pub fn shapeProximityWithCollector(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_ShapeProximityWithCollector", .{a, b, c}); }
    pub fn shapeCastWithCollector(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_ShapeCastWithCollector", .{a, b, c}); }

    pub fn step(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_Step", .{a, b}); }
    pub fn setIdealStepTime(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_SetIdealStepTime", .{a, b}); }

    pub fn setSpeedLimit(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_SetSpeedLimit", .{a, b, c}); }
    pub fn getSpeedLimit(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_World_GetSpeedLimit", .{a, b, c}); }

    pub fn getNextCollisionEvent(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_GetNextCollisionEvent", .{a, b}); }
    pub fn getNextTriggerEvent(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_World_GetNextTriggerEvent", .{a, b}); }

    // zig fmt: on
},
query_collector: struct {
    physics: *HavokPhysics,

    // zig fmt: off

    pub fn create(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_QueryCollector_Create", .{a, b}); }
    pub fn release(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_QueryCollector_Release", .{a}); }

    pub fn getNumHits(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_QueryCollector_GetNumHits", .{a, b}); }

    pub fn getCastRayResult(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_QueryCollector_GetCastRayResult", .{a, b, c}); }
    pub fn getPointProximityResult(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_QueryCollector_GetPointProximityResult", .{a, b, c}); }
    pub fn getShapeProximityResult(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_QueryCollector_GetShapeProximityResult", .{a, b, c}); }
    pub fn getShapeCastResult(self: *@This(), a: u32, b: u32, c: u32) !u32 { return self.physics.callExported("HP_QueryCollector_GetShapeCastResult", .{a, b, c}); }

    // zig fmt: on
},
debug: struct {
    physics: *HavokPhysics,

    // zig fmt: off

    pub fn startRecordingStats(self: *@This(), a: u32) !u32 { return self.physics.callExported("HP_Debug_StartRecordingStats", .{a}); }
    pub fn stopRecordingStats(self: *@This(), a: u32, b: u32) !u32 { return self.physics.callExported("HP_Debug_StopRecordingStats", .{a, b}); }

    // zig fmt: on
},

const raw_buf = @embedFile("binary/x86_64/HavokPhysics.aot");
const buf align(8) = raw_buf.*; // Change alignment for WAMR

const heap_size: u32 = 10 * 1024 * 1024;

var heap_buf: [heap_size]u8 align(8) = undefined;

const heap_ptr: [*]align(8) u8 = &heap_buf;

var heap_i8: []i8 = @as([*]i8, @ptrCast(heap_ptr))[0..heap_size];
var heap_u8: []u8 = heap_ptr[0..heap_size];

var heap_i16: []i16 = @as([*]i16, @ptrCast(heap_ptr))[0 .. heap_size / 2];
var heap_u16: []u16 = @as([*]u16, @ptrCast(heap_ptr))[0 .. heap_size / 2];

var heap_i32: []i32 = @as([*]i32, @ptrCast(heap_ptr))[0 .. heap_size / 4];
var heap_u32: []u32 = @as([*]u32, @ptrCast(heap_ptr))[0 .. heap_size / 4];

var heap_i64: []i64 = @as([*]i64, @ptrCast(heap_ptr))[0 .. heap_size / 8];
var heap_u64: []u64 = @as([*]u64, @ptrCast(heap_ptr))[0 .. heap_size / 8];

var heap_f32: []f32 = @as([*]f32, @ptrCast(heap_ptr))[0 .. heap_size / 4];
var heap_f64: []f64 = @as([*]f64, @ptrCast(heap_ptr))[0 .. heap_size / 8];

const stack_size: u32 = 64 * 1024;

fn abort_js(_: wamr.wasm_exec_env_t) callconv(.c) void {
    log.err("aborted", .{});
}

fn emscripten_get_heap_max(_: wamr.wasm_exec_env_t) callconv(.c) i32 {
    return 2147483647;
}

fn emscripten_resize_heap(_: wamr.wasm_exec_env_t, _: i32) callconv(.c) i32 {
    return 1;
}

fn emscripten_date_now(_: wamr.wasm_exec_env_t) callconv(.c) f64 {
    return @floatFromInt(time.milliTimestamp());
}

fn emscripten_get_now(exec_env: wamr.wasm_exec_env_t) callconv(.c) f64 {
    return emscripten_date_now(exec_env) / 1000000.0;
}

fn emscripten_get_now_is_monotonic(_: wamr.wasm_exec_env_t) callconv(.c) i32 {
    return 1;
}

fn unimplemented(src: builtin.SourceLocation) void {
    log.err("unimplemented function called: {s}", .{src.fn_name});
}

fn readLatin1String(physics: *HavokPhysics, ptr: u32) ![]u8 {
    const allocator = physics.embind_allocator;

    const raw_ptr = wamr.wasm_runtime_addr_app_to_native(physics.module_inst, ptr);
    if (raw_ptr == null)
        return error.InvalidMemoryAccess;

    const native_ptr: [*]u8 = @ptrCast(@alignCast(raw_ptr));

    var list: std.ArrayList(u8) = .empty;
    errdefer list.deinit(allocator);

    var i: usize = 0;

    while (true) {
        const byte = native_ptr[i];
        if (byte == 0)
            break;

        var utf8_buf: [4]u8 = undefined;

        const len = try unicode.utf8Encode(byte, &utf8_buf);

        try list.appendSlice(allocator, utf8_buf[0..len]);

        i += 1;
    }

    return list.toOwnedSlice(allocator);
}

const RegisteredCounter = atomic.Value(u16);

fn whenDependentTypesAreResolvedOnComplete(
    self: *HavokPhysics,
    context: *anyopaque,
    ids: []const Emscripten.Bind.Type.Id,
    converters_handler: Emscripten.Bind.Type.ConvertersHandler,
    free_converters_handler_return: bool,
    converters: Emscripten.Bind.Type.Converters,
) anyerror!void {
    const allocator = self.embind_allocator;

    const handled_converters = converters_handler(self, context, converters);
    defer if (free_converters_handler_return)
        allocator.free(handled_converters);

    if (ids.len != handled_converters.len)
        return error.MismatchLength;

    for (ids, handled_converters) |id, converter|
        try self.registerType(id, converter);
}

fn whenDependentTypesAreResolved(
    self: *HavokPhysics,
    context: *anyopaque,
    ids: []const Emscripten.Bind.Type.Id,
    dependent_ids: []const Emscripten.Bind.Type.Id,
    converters_handler: Emscripten.Bind.Type.ConvertersHandler,
    free_converters: bool,
    free_converters_handler_return: bool,
) !void {
    const allocator = self.embind_allocator;

    const converters = allocator.alloc(*const Emscripten.Bind.Type.Instance, dependent_ids.len) catch return;
    errdefer allocator.free(converters);

    const registered = try allocator.create(RegisteredCounter);
    errdefer allocator.destroy(registered);

    const unregistered = try allocator.create(RegisteredCounter);
    errdefer allocator.destroy(unregistered);

    registered.* = .init(0);
    unregistered.* = .init(0);

    const ids_dupe = try allocator.dupe(Emscripten.Bind.Type.Id, ids);
    errdefer allocator.free(ids_dupe);

    for (dependent_ids, 0..) |id, i| {
        if (self.embind_type_registry.get(id)) |instance|
            converters[i] = instance
        else {
            _ = unregistered.fetchAdd(1, .monotonic);

            const entry = try self.embind_awaiting_dependencies.getOrPut(id);

            if (!entry.found_existing)
                entry.value_ptr.* = .init(allocator);

            const waiter = try allocator.create(Emscripten.Bind.DependencyWaiter);
            errdefer allocator.destroy(waiter);

            waiter.* = .{
                .context = context,

                .i = i,

                .type_ids = ids_dupe,

                .type_converters = converters,

                .type_converters_handler = converters_handler,
                .free_type_converters_handler_return = free_converters_handler_return,

                .registered = registered,
                .unregistered = unregistered,
            };

            try entry.value_ptr.append(waiter);
        }
    }

    if (unregistered.load(.monotonic) == 0) {
        allocator.destroy(registered);
        allocator.destroy(unregistered);

        allocator.free(ids_dupe);

        try self.whenDependentTypesAreResolvedOnComplete(
            context,
            ids,
            converters_handler,
            free_converters_handler_return,
            converters,
        );

        if (free_converters)
            allocator.free(converters);
    }
}

fn getPhysics(exec_env: wamr.wasm_exec_env_t) ?*HavokPhysics {
    return if (wamr.wasm_runtime_get_function_attachment(exec_env)) |attachment|
        @ptrCast(@alignCast(attachment))
    else
        null;
}

fn embind_register_void(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .void,
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_bool(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
    true_value: i32,
    false_value: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .bool,

            .true_value = @intCast(true_value),
            .false_value = @intCast(false_value),
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_integer(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
    size: i32,
    min_range: i32,
    _: i32, // max_range (unused)
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .int,

            .size = @intCast(size),

            .is_signed = min_range != 0,
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_float(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
    size: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .float,

            .size = @intCast(size),
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_std_string(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .std_string,

            .destructor = @ptrCast(&freeDesturctor),
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_std_wstring(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    char_size: i32,
    name_ptr: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .std_wstring,

            .destructor = @ptrCast(&freeDesturctor),

            .size = @intCast(char_size),
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_emval(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics|
        physics.registerType(type_id, .emval) catch return;
}

fn embind_register_memory_view(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    data_type: i32,
    name_ptr: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        if (physics.embind_type_registry.contains(type_id))
            return;

        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .memory_view,

            .data_type = @intCast(data_type),
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn usesDestructorStack(type_instances: []?*const Emscripten.Bind.Type.Instance) bool {
    for (type_instances[1..]) |item|
        if (item) |type_instance|
            if (type_instance.destructor == null)
                return true;

    return false;
}

const MethodSignature = enum {
    ftf,
    ftfn,
    fffn,
    ftfnnnn,
    ftfnn,
    ftftt,
    ftft,
    ftftn,
    ftftnn,
    ftfttn,
    ftfttt,
    ftfnntn,
    ftftttt,
};

fn createMethodSignature(
    type_instances: []?*const Emscripten.Bind.Type.Instance,
    returns: bool,
    is_async: bool,
) !MethodSignature {
    var buffer: [8]u8 = undefined;
    var len: usize = 0;

    buffer[len] = 'f';
    len += 1;

    buffer[len] = if (returns) 't' else 'f';
    len += 1;

    buffer[len] = if (is_async) 't' else 'f';
    len += 1;

    for (type_instances[2..]) |item|
        if (item) |type_instance| {
            buffer[len] = if (type_instance.destructor != null) 't' else 'n';
            len += 1;
        };

    return meta.stringToEnum(MethodSignature, buffer[0..len]) orelse error.InvalidSignature;
}

fn createMethodImplInner(comptime Return: type, signature: MethodSignature) MethodImpl(Return) {
    return switch (signature) {
        .ftf => @ptrCast(&struct {
            fn impl(physics: *HavokPhysics, context_index: u8) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const return_wired = physics.call(context.invoker, .{context.function}) catch unreachable;

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftfn => @ptrCast(&struct {
            fn impl(physics: *HavokPhysics, context_index: u8, arg_0: *const anyopaque) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired }) catch unreachable;

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .fffn => @ptrCast(&struct { // We here want to constantly change Return to void, but it may be broken in later changes
            fn impl(physics: *HavokPhysics, context_index: u8, arg_0: *const anyopaque) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);

                _ = physics.call(context.invoker, .{ context.function, arg_0_wired }) catch unreachable;

                return mem.zeroes(Return);
            }
        }.impl),
        .ftfnnnn => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
                arg_2: *const anyopaque,
                arg_3: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2);
                const arg_3_wired = context.arg_type_instances[3].toWire(arg_3);

                const return_wired = physics.call(context.invoker, .{
                    context.function,
                    arg_0_wired,
                    arg_1_wired,
                    arg_2_wired,
                    arg_3_wired,
                }) catch unreachable;

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftfnn => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired, arg_1_wired }) catch unreachable;

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftftt => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired, arg_1_wired }) catch unreachable;

                { // Destruct arg_0_wired
                    const is_destructor_wasm, const destructor = context.destructors[0];

                    if (is_destructor_wasm)
                        destructor(arg_0_wired)
                    else
                        destructor(physics, arg_0_wired);
                }

                { // Destruct arg_1_wired
                    const is_destructor_wasm, const destructor = context.destructors[1];

                    if (is_destructor_wasm)
                        destructor(arg_1_wired)
                    else
                        destructor(physics, arg_1_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftft => @ptrCast(&struct {
            fn impl(physics: *HavokPhysics, context_index: u8, arg_0: *const anyopaque) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired }) catch unreachable;

                { // Destruct arg_0_wired
                    const is_destructor_wasm, const destructor = context.destructors[0];

                    if (is_destructor_wasm)
                        destructor(arg_0_wired)
                    else
                        destructor(physics, arg_0_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftftn => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired, arg_1_wired }) catch unreachable;

                { // Destruct arg_0_wired
                    const is_destructor_wasm, const destructor = context.destructors[0];

                    if (is_destructor_wasm)
                        destructor(arg_0_wired)
                    else
                        destructor(physics, arg_0_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftftnn => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
                arg_2: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired, arg_1_wired, arg_2_wired }) catch unreachable;

                { // Destruct arg_0_wired
                    const is_destructor_wasm, const destructor = context.destructors[0];

                    if (is_destructor_wasm)
                        destructor(arg_0_wired)
                    else
                        destructor(physics, arg_0_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftfttn => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
                arg_2: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired, arg_1_wired, arg_2_wired }) catch unreachable;

                { // Destruct arg_0_wired
                    const is_destructor_wasm, const destructor = context.destructors[0];

                    if (is_destructor_wasm)
                        destructor(arg_0_wired)
                    else
                        destructor(physics, arg_0_wired);
                }

                { // Destruct arg_1_wired
                    const is_destructor_wasm, const destructor = context.destructors[1];

                    if (is_destructor_wasm)
                        destructor(arg_1_wired)
                    else
                        destructor(physics, arg_1_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftfttt => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
                arg_2: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2);

                const return_wired = physics.call(context.invoker, .{ context.function, arg_0_wired, arg_1_wired, arg_2_wired }) catch unreachable;

                { // Destruct arg_0_wired
                    const is_destructor_wasm, const destructor = context.destructors[0];

                    if (is_destructor_wasm)
                        destructor(arg_0_wired)
                    else
                        destructor(physics, arg_0_wired);
                }

                { // Destruct arg_1_wired
                    const is_destructor_wasm, const destructor = context.destructors[1];

                    if (is_destructor_wasm)
                        destructor(arg_1_wired)
                    else
                        destructor(physics, arg_1_wired);
                }

                { // Destruct arg_2_wired
                    const is_destructor_wasm, const destructor = context.destructors[2];

                    if (is_destructor_wasm)
                        destructor(arg_2_wired)
                    else
                        destructor(physics, arg_2_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftfnntn => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
                arg_2: *const anyopaque,
                arg_3: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2);
                const arg_3_wired = context.arg_type_instances[3].toWire(arg_3);

                const return_wired = physics.call(context.invoker, .{
                    context.function,
                    arg_0_wired,
                    arg_1_wired,
                    arg_2_wired,
                    arg_3_wired,
                }) catch unreachable;

                { // Destruct arg_2_wired
                    const is_destructor_wasm, const destructor = context.destructors[2];

                    if (is_destructor_wasm)
                        destructor(arg_2_wired)
                    else
                        destructor(physics, arg_2_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
        .ftftttt => @ptrCast(&struct {
            fn impl(
                physics: *HavokPhysics,
                context_index: u8,
                arg_0: *const anyopaque,
                arg_1: *const anyopaque,
                arg_2: *const anyopaque,
                arg_3: *const anyopaque,
            ) callconv(.c) Return {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2);
                const arg_3_wired = context.arg_type_instances[3].toWire(arg_3);

                const return_wired = physics.call(context.invoker, .{
                    context.function,
                    arg_0_wired,
                    arg_1_wired,
                    arg_2_wired,
                    arg_3_wired,
                }) catch unreachable;

                { // Destruct arg_0_wired
                    const is_destructor_wasm, const destructor = context.destructors[0];

                    if (is_destructor_wasm)
                        destructor(arg_0_wired)
                    else
                        destructor(physics, arg_0_wired);
                }

                { // Destruct arg_1_wired
                    const is_destructor_wasm, const destructor = context.destructors[1];

                    if (is_destructor_wasm)
                        destructor(arg_1_wired)
                    else
                        destructor(physics, arg_1_wired);
                }

                { // Destruct arg_2_wired
                    const is_destructor_wasm, const destructor = context.destructors[2];

                    if (is_destructor_wasm)
                        destructor(arg_2_wired)
                    else
                        destructor(physics, arg_2_wired);
                }

                { // Destruct arg_3_wired
                    const is_destructor_wasm, const destructor = context.destructors[3];

                    if (is_destructor_wasm)
                        destructor(arg_3_wired)
                    else
                        destructor(physics, arg_3_wired);
                }

                return @as(*const Return, @ptrCast(@alignCast(context.return_type_instance.fromWire(return_wired)))).*;
            }
        }.impl),
    };
}

/// type_instances must not be deinited after this function called.
/// type_instances's parent array list deinited when arena's deinit called.
fn createMethodImpl(
    self: *HavokPhysics,
    comptime Return: type,
    name: []u8,
    type_instances: []?*const Emscripten.Bind.Type.Instance,
    invoker: wamr.wasm_function_inst_t,
    function: u32,
    is_async: bool,
) !MethodImpl(Return) {
    const allocator = self.embind_allocator;

    const type_instances_len = type_instances.len;
    if (type_instances_len < 2)
        return error.NotEnoughArgumentsCount;

    const returns =
        if (type_instances[0]) |item|
            !mem.eql(u8, item.name, "void")
        else
            false;

    const context = try allocator.create(Emscripten.Bind.InvokerContext);

    context.* = .{
        .invoker = invoker,

        .function = function,

        .return_type_instance = type_instances[0] orelse return error.MissingReturnType,
    };

    for (type_instances[2..], 0..) |type_instance, i|
        context.arg_type_instances[i] = type_instance orelse return error.MissingArgumentType;

    if (!usesDestructorStack(type_instances))
        for (type_instances[2..], 0..) |item, i|
            if (item) |type_instance| {
                if (type_instance.destructor) |destructor|
                    context.destructors[i] = .{ type_instance.is_destructor_wasm, destructor };
            };

    if (self.embind_invoker_function_indices.get(name)) |invoker_index|
        self.embind_invoker_contexts[invoker_index] = context;

    return createMethodImplInner(Return, try createMethodSignature(type_instances, returns, is_async));
}

fn embind_register_function(
    exec_env: wamr.wasm_exec_env_t,
    name_ptr: i32,
    dependent_type_ids_count: i32,
    dependent_type_ids_ptr: i32,
    _: i32,
    invoker_idx: i32,
    function: i32,
    is_async: i32,
    _: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const allocator = physics.embind_allocator;

        const dependent_type_ids = allocator.alloc(Emscripten.Bind.Type.Id, @intCast(dependent_type_ids_count)) catch return;
        defer allocator.free(dependent_type_ids);

        { // Read dependent type ids
            const raw_ptr = wamr.wasm_runtime_addr_app_to_native(physics.module_inst, @intCast(dependent_type_ids_ptr));
            if (raw_ptr == null)
                return;

            const native_i32_ptr: [*]i32 = @ptrCast(@alignCast(raw_ptr));

            @memcpy(dependent_type_ids, native_i32_ptr[0..dependent_type_ids.len]);
        }

        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const RegisterContext = struct {
            name: []u8,
            invoker: wamr.wasm_function_inst_t,
            function: u32,
            is_async: bool,
        };

        const register_context = allocator.create(RegisterContext) catch return;

        register_context.* = .{
            .name = name,
            .invoker = physics.getFunctionIndirect(@intCast(invoker_idx)) catch {
                allocator.free(name);

                allocator.destroy(register_context);

                return;
            },
            .function = @intCast(function),
            .is_async = is_async != 0,
        };

        physics.whenDependentTypesAreResolved(
            register_context,
            &.{},
            dependent_type_ids,
            struct {
                fn impl(
                    physics_inner: *HavokPhysics,
                    context: *anyopaque,
                    converters: Emscripten.Bind.Type.Converters,
                ) Emscripten.Bind.Type.Converters {
                    const allocator_inner = physics_inner.embind_allocator;

                    const register_context_inner: *RegisterContext = @ptrCast(@alignCast(context));
                    defer {
                        allocator_inner.free(register_context_inner.name);

                        allocator_inner.destroy(register_context_inner);
                    }

                    var invoker_type_instances: array_list.Managed(?*const Emscripten.Bind.Type.Instance) = .init(allocator_inner);

                    invoker_type_instances.append(converters[0]) catch {
                        invoker_type_instances.deinit();

                        return &.{};
                    };
                    invoker_type_instances.append(null) catch {
                        invoker_type_instances.deinit();

                        return &.{};
                    };
                    invoker_type_instances.appendSlice(@ptrCast(converters[1..])) catch {
                        invoker_type_instances.deinit();

                        return &.{};
                    };

                    const method_impl = physics_inner.createMethodImpl(
                        struct {},
                        register_context_inner.name,
                        invoker_type_instances.toOwnedSlice() catch {
                            invoker_type_instances.deinit();

                            return &.{};
                        },
                        register_context_inner.invoker,
                        register_context_inner.function,
                        register_context_inner.is_async,
                    ) catch {
                        invoker_type_instances.deinit();

                        return &.{};
                    };

                    _ = method_impl;

                    return &.{};
                }
            }.impl,
            true,
            false,
        ) catch return;
    }
}

fn embind_register_value_array(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
    _: i32,
    constructor_idx: i32,
    _: i32,
    destructor_idx: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        physics.embind_tuple_registry.put(type_id, .{
            .name = name,

            .constructor = physics.getFunctionIndirect(@intCast(constructor_idx)) catch {
                allocator.free(name);

                return;
            },
            .destructor = physics.getFunctionIndirect(@intCast(destructor_idx)) catch {
                allocator.free(name);

                return;
            },

            .elements = .init(allocator),
        }) catch allocator.free(name);
    }
}

fn embind_register_value_array_element(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    getter_return_type: i32,
    _: i32,
    getter_idx: i32,
    getter_context: i32,
    setter_arg_type: i32,
    _: i32,
    setter_idx: i32,
    setter_context: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics|
        if (physics.embind_tuple_registry.getPtr(type_id)) |tuple|
            tuple.elements.append(.{
                .getter_return_type = getter_return_type,
                .getter = physics.getFunctionIndirect(@intCast(getter_idx)) catch return,
                .getter_context = getter_context,

                .setter_arg_type = setter_arg_type,
                .setter = physics.getFunctionIndirect(@intCast(setter_idx)) catch return,
                .setter_context = setter_context,
            }) catch return;
}

fn embind_register_bigint(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
    size: i32, // 1, 2, 4, 8
    _: i64,
    _: i64,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch {
            allocator.free(name);

            return;
        };

        instance.* = .{
            .name = name,
            .kind = .bigint,

            .size = @intCast(size),
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_enum(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
    size: i32, // 1, 2, 4
    is_signed: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch return;

        instance.* = .{
            .name = name,
            .kind = .@"enum",

            .size = @intCast(size),

            .is_signed = is_signed != 0,
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_enum_value(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
    name_ptr: i32,
    enum_value: i32,
) callconv(.c) void {
    _ = exec_env;
    _ = type_id;
    _ = name_ptr;
    _ = enum_value;
}

fn embind_finalize_value_array(
    exec_env: wamr.wasm_exec_env_t,
    type_id: Emscripten.Bind.Type.Id,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics|
        if (physics.embind_tuple_registry.fetchRemove(type_id)) |tuple_kv| {
            const allocator = physics.embind_allocator;

            const tuple_ptr = allocator.create(Emscripten.Bind.Tuple) catch return;

            tuple_ptr.* = tuple_kv.value;

            var element_type_ids: array_list.Managed(Emscripten.Bind.Type.Id) = .init(allocator);
            defer element_type_ids.deinit();

            for (tuple_ptr.elements.items) |element|
                element_type_ids.append(element.getter_return_type) catch {
                    allocator.free(tuple_ptr.name);

                    tuple_ptr.elements.deinit();

                    allocator.destroy(tuple_ptr);

                    return;
                };

            for (tuple_ptr.elements.items) |element|
                element_type_ids.append(element.setter_arg_type) catch {
                    allocator.free(tuple_ptr.name);

                    tuple_ptr.elements.deinit();

                    allocator.destroy(tuple_ptr);

                    return;
                };

            physics.whenDependentTypesAreResolved(
                tuple_ptr,
                &.{type_id},
                element_type_ids.toOwnedSlice() catch {
                    allocator.free(tuple_ptr.name);

                    tuple_ptr.elements.deinit();

                    allocator.destroy(tuple_ptr);

                    return;
                },
                struct {
                    fn impl(
                        physics_inner: *HavokPhysics,
                        context: *anyopaque,
                        converters: Emscripten.Bind.Type.Converters,
                    ) Emscripten.Bind.Type.Converters {
                        const allocator_inner = physics_inner.embind_allocator;

                        const tuple_inner: *Emscripten.Bind.Tuple = @ptrCast(@alignCast(context));

                        const instance = allocator_inner.create(Emscripten.Bind.Type.Instance) catch return &.{};

                        instance.* = .{
                            .name = tuple_inner.name,
                            .kind = .tuple,

                            .destructor = @ptrCast(tuple_inner.destructor),
                            .is_destructor_wasm = true,

                            .tuple_constructor = @ptrCast(tuple_inner.constructor),
                            .tuple_elements = tuple_inner.elements,
                            .tuple_converters = converters,
                        };

                        allocator_inner.destroy(tuple_inner);

                        const result_converters = allocator_inner.alloc(*const Emscripten.Bind.Type.Instance, 1) catch return &.{};

                        result_converters[0] = instance;

                        return result_converters;
                    }
                }.impl,
                false,
                true,
            ) catch {
                allocator.free(tuple_ptr.name);

                tuple_ptr.elements.deinit();

                allocator.destroy(tuple_ptr);
            };
        };
}

fn emval_get_method_caller(_: wamr.wasm_exec_env_t, _: i32, _: i32, _: i32) callconv(.c) void {
    unimplemented(@src());
}

fn emval_call_method(_: wamr.wasm_exec_env_t, _: i32, _: i32) callconv(.c) void {
    unimplemented(@src());
}

fn emval_decref(_: wamr.wasm_exec_env_t, _: i32) callconv(.c) void {
    unimplemented(@src());
}

fn emval_run_destructors(_: wamr.wasm_exec_env_t, _: i32) callconv(.c) void {
    unimplemented(@src());
}

fn fd_write(_: wamr.wasm_exec_env_t, _: i32, _: i32, _: i32, _: i32) callconv(.c) void {
    unimplemented(@src());
}

pub fn init(allocator: mem.Allocator) !*HavokPhysics {
    var physics = try allocator.create(HavokPhysics);
    errdefer allocator.destroy(physics);

    var emval_handles: Emscripten.Val.Handles = .init(allocator);

    try emval_handles.append(.{ .value = null, .ref_count = 1 });
    try emval_handles.append(.{ .value = null, .ref_count = 1 });
    try emval_handles.append(.{ .value = null, .ref_count = 1 });
    try emval_handles.append(.{ .value = null, .ref_count = 1 });
    try emval_handles.append(.{ .value = null, .ref_count = 1 });

    physics.* = .{
        .allocator = allocator,

        .cached_indirect_functions = .init(allocator),

        .emval_handles = emval_handles,
        .emval_free_list = .init(allocator),

        .embind_arena = undefined,
        .embind_allocator = undefined,

        .embind_type_registry = .init(allocator),

        .embind_tuple_registry = .init(allocator),

        .embind_awaiting_dependencies = .init(allocator),

        .embind_invoker_function_indices = .init(allocator),

        .shape = .{ .physics = physics },
        .debug_geometry = .{ .physics = physics },
        .body = .{ .physics = physics },
        .constraint = .{ .physics = physics },
        .world = .{ .physics = physics },
        .query_collector = .{ .physics = physics },
        .debug = .{ .physics = physics },
    };

    physics.embind_arena = .init(allocator);
    physics.embind_allocator = physics.embind_arena.allocator();

    for (function_names, 0..) |function_name, i| // Add function indices
        try physics.embind_invoker_function_indices.put(function_name, i);

    var init_args = mem.zeroes(wamr.RuntimeInitArgs);

    { // Allocate with pool
        init_args.mem_alloc_type = wamr.Alloc_With_Pool;

        init_args.mem_alloc_option.pool.heap_buf = &heap_buf;
        init_args.mem_alloc_option.pool.heap_size = heap_buf.len;
    }

    if (!wamr.wasm_runtime_full_init(&init_args))
        return error.FullInitFailed;

    { // Add native symbols
        var native_symbols_emscripten = [_]wamr.NativeSymbol{
            .{ .symbol = "_abort_js", .func_ptr = @ptrCast(@constCast(&abort_js)), .signature = "()", .attachment = null },

            .{ .symbol = "emscripten_get_heap_max", .func_ptr = @ptrCast(@constCast(&emscripten_get_heap_max)), .signature = "()i", .attachment = null },
            .{ .symbol = "emscripten_resize_heap", .func_ptr = @ptrCast(@constCast(&emscripten_resize_heap)), .signature = "(i)i", .attachment = null },
            .{ .symbol = "emscripten_date_now", .func_ptr = @ptrCast(@constCast(&emscripten_date_now)), .signature = "()F", .attachment = null },
            .{ .symbol = "emscripten_get_now", .func_ptr = @ptrCast(@constCast(&emscripten_get_now)), .signature = "()F", .attachment = null },
            .{ .symbol = "_emscripten_get_now_is_monotonic", .func_ptr = @ptrCast(@constCast(&emscripten_get_now_is_monotonic)), .signature = "()i", .attachment = null },

            .{ .symbol = "_embind_register_void", .func_ptr = @ptrCast(@constCast(&embind_register_void)), .signature = "(ii)", .attachment = physics },
            .{ .symbol = "_embind_register_bool", .func_ptr = @ptrCast(@constCast(&embind_register_bool)), .signature = "(iiii)", .attachment = physics },
            .{ .symbol = "_embind_register_integer", .func_ptr = @ptrCast(@constCast(&embind_register_integer)), .signature = "(iiiii)", .attachment = physics },
            .{ .symbol = "_embind_register_float", .func_ptr = @ptrCast(@constCast(&embind_register_float)), .signature = "(iii)", .attachment = physics },
            .{ .symbol = "_embind_register_std_string", .func_ptr = @ptrCast(@constCast(&embind_register_std_string)), .signature = "(ii)", .attachment = physics },
            .{ .symbol = "_embind_register_std_wstring", .func_ptr = @ptrCast(@constCast(&embind_register_std_wstring)), .signature = "(iii)", .attachment = physics },
            .{ .symbol = "_embind_register_emval", .func_ptr = @ptrCast(@constCast(&embind_register_emval)), .signature = "(i)", .attachment = physics },
            .{ .symbol = "_embind_register_memory_view", .func_ptr = @ptrCast(@constCast(&embind_register_memory_view)), .signature = "(iii)", .attachment = physics },
            .{ .symbol = "_embind_register_function", .func_ptr = @ptrCast(@constCast(&embind_register_function)), .signature = "(iiiiiiii)", .attachment = physics },
            .{ .symbol = "_embind_register_value_array", .func_ptr = @ptrCast(@constCast(&embind_register_value_array)), .signature = "(iiiiii)", .attachment = physics },
            .{ .symbol = "_embind_register_value_array_element", .func_ptr = @ptrCast(@constCast(&embind_register_value_array_element)), .signature = "(iiiiiiiii)", .attachment = physics },
            .{ .symbol = "_embind_register_bigint", .func_ptr = @ptrCast(@constCast(&embind_register_bigint)), .signature = "(iiiII)", .attachment = physics },
            .{ .symbol = "_embind_register_enum", .func_ptr = @ptrCast(@constCast(&embind_register_enum)), .signature = "(iiii)", .attachment = physics },
            .{ .symbol = "_embind_register_enum_value", .func_ptr = @ptrCast(@constCast(&embind_register_enum_value)), .signature = "(iii)", .attachment = physics },
            .{ .symbol = "_embind_finalize_value_array", .func_ptr = @ptrCast(@constCast(&embind_finalize_value_array)), .signature = "(i)", .attachment = physics },
            .{ .symbol = "_emval_get_method_caller", .func_ptr = @ptrCast(@constCast(&emval_get_method_caller)), .signature = "(iii)i", .attachment = physics },
            .{ .symbol = "_emval_call_method", .func_ptr = @ptrCast(@constCast(&emval_call_method)), .signature = "(iiiii)F", .attachment = physics },
            .{ .symbol = "_emval_decref", .func_ptr = @ptrCast(@constCast(&emval_decref)), .signature = "(i)", .attachment = physics },
            .{ .symbol = "_emval_run_destructors", .func_ptr = @ptrCast(@constCast(&emval_run_destructors)), .signature = "(i)", .attachment = physics },
        };

        var native_symbols_wasi = [_]wamr.NativeSymbol{
            .{ .symbol = "fd_write", .func_ptr = @ptrCast(@constCast(&fd_write)), .signature = "(iiii)i", .attachment = null },
        };

        try physics.registerNativeSymbols("env", &native_symbols_emscripten);
        try physics.registerNativeSymbols("wasi_snapshot_preview1", &native_symbols_wasi);
    }

    var error_buf = mem.zeroes([128]u8);

    physics.module = wamr.wasm_runtime_load(
        @constCast(&buf),
        buf.len,
        &error_buf,
        error_buf.len,
    );
    if (physics.module == null)
        return error.LoadFailed;

    _ = wamr.wasm_runtime_set_wasi_args_ex(
        physics.module,
        null,
        0,
        null,
        0,
        null,
        0,
        null,
        0,
        -1,
        -1,
        -1,
    );

    physics.module_inst = wamr.wasm_runtime_instantiate(physics.module, stack_size, heap_size, &error_buf, error_buf.len);
    if (physics.module_inst == null)
        return error.InstantiateFailed;

    physics.exec_env = wamr.wasm_runtime_create_exec_env(physics.module_inst, stack_size);
    if (physics.exec_env == null)
        return error.ExecEnvCreateFailed;

    if (!wamr.wasm_runtime_get_export_table_inst(physics.module_inst, "__indirect_function_table", &physics.table_inst))
        return error.TableInstGetFailed;

    return physics;
}

pub fn deinit(self: *HavokPhysics) void {
    if (self.exec_env) |exec_env| wamr.wasm_runtime_destroy_exec_env(exec_env);
    if (self.module_inst) |module_inst| wamr.wasm_runtime_deinstantiate(module_inst);
    if (self.module) |module| wamr.wasm_runtime_unload(module);

    wamr.wasm_runtime_destroy();

    self.cached_indirect_functions.deinit();

    { // Free emval's
        self.emval_handles.deinit();
        self.emval_free_list.deinit();
    }

    { // Free embind's
        self.embind_arena.deinit();

        self.embind_type_registry.deinit();

        self.embind_tuple_registry.deinit();

        self.embind_awaiting_dependencies.deinit();

        self.embind_invoker_function_indices.deinit();
    }

    self.allocator.destroy(self);
}

pub fn start(self: *HavokPhysics) !u32 {
    _ = try self.callExported("__wasm_call_ctors", .{});

    if (wamr.wasm_application_execute_main(self.module_inst, 0, null))
        return wamr.wasm_runtime_get_wasi_exit_code(self.module_inst)
    else {
        const exception = wamr.wasm_runtime_get_exception(self.module_inst);

        log.err("wasm execution failed: {s}", .{exception});

        return error.MainExecutionFailed;
    }
}

fn registerNativeSymbols(
    _: *const HavokPhysics,
    comptime module_name: [*c]const u8,
    native_symbols: []wamr.NativeSymbol,
) !void {
    if (!wamr.wasm_runtime_register_natives(
        module_name,
        native_symbols.ptr,
        @intCast(native_symbols.len),
    ))
        return error.NativeRegistrationFailed;
}

pub fn registerType(
    self: *HavokPhysics,
    id: Emscripten.Bind.Type.Id,
    instance: *const Emscripten.Bind.Type.Instance,
) anyerror!void {
    if (self.embind_type_registry.contains(id))
        // Handled in embind_register_memory_view
        // if (instance.kind == .memory_view)
        //     return
        // else
        return error.DuplicatedRegistration;

    try self.embind_type_registry.put(id, instance);

    if (self.embind_awaiting_dependencies.fetchRemove(id)) |entry| {
        const allocator = self.embind_allocator;

        var awaiting_dependency = entry.value;
        defer awaiting_dependency.deinit();

        for (awaiting_dependency.items) |dependency_waiter| {
            defer allocator.destroy(dependency_waiter);

            dependency_waiter.type_converters[dependency_waiter.i] = instance;

            if (dependency_waiter.registered.fetchAdd(1, .monotonic) + 1 ==
                dependency_waiter.unregistered.load(.monotonic))
                try self.whenDependentTypesAreResolvedOnComplete(
                    dependency_waiter.context,
                    dependency_waiter.type_ids,
                    dependency_waiter.type_converters_handler,
                    dependency_waiter.free_type_converters_handler_return,
                    dependency_waiter.type_converters,
                );
        }
    }

    log.debug("registered type id: {d}, name: {s}, kind: {any}", .{ id, instance.name, instance.kind });
}

const function_names = [_][]const u8{
    "HP_GetStatistics",

    "HP_Shape_CreateSphere",
    "HP_Shape_CreateCapsule",
    "HP_Shape_CreateCylinder",
    "HP_Shape_CreateBox",
    "HP_Shape_CreateConvexHull",
    "HP_Shape_CreateMesh",
    "HP_Shape_CreateHeightField",
    "HP_Shape_CreateContainer",
    "HP_Shape_Release",
    "HP_Shape_GetType",
    "HP_Shape_AddChild",
    "HP_Shape_RemoveChild",
    "HP_Shape_GetNumChildren",
    "HP_Shape_GetChildShape",
    "HP_Shape_SetChildQSTransform",
    "HP_Shape_GetChildQSTransform",
    "HP_Shape_SetFilterInfo",
    "HP_Shape_GetFilterInfo",
    "HP_Shape_SetMaterial",
    "HP_Shape_GetMaterial",
    "HP_Shape_SetDensity",
    "HP_Shape_GetDensity",
    "HP_Shape_GetBoundingBox",
    "HP_Shape_CastRay",
    "HP_Shape_BuildMassProperties",
    "HP_Shape_SetTrigger",
    "HP_Shape_PathIterator_GetNext",
    "HP_Shape_CreateDebugDisplayGeometry",

    "HP_DebugGeometry_GetInfo",
    "HP_DebugGeometry_Release",

    "HP_Body_Create",
    "HP_Body_Release",
    "HP_Body_SetShape",
    "HP_Body_GetShape",
    "HP_Body_SetMotionType",
    "HP_Body_GetMotionType",
    "HP_Body_SetEventMask",
    "HP_Body_GetEventMask",
    "HP_Body_SetMassProperties",
    "HP_Body_GetMassProperties",
    "HP_Body_SetLinearDamping",
    "HP_Body_GetLinearDamping",
    "HP_Body_SetAngularDamping",
    "HP_Body_GetAngularDamping",
    "HP_Body_SetGravityFactor",
    "HP_Body_GetGravityFactor",
    "HP_Body_GetWorld",
    "HP_Body_GetWorldTransformOffset",
    "HP_Body_SetPosition",
    "HP_Body_GetPosition",
    "HP_Body_SetOrientation",
    "HP_Body_GetOrientation",
    "HP_Body_SetQTransform",
    "HP_Body_GetQTransform",
    "HP_Body_SetTargetQTransform",
    "HP_Body_SetLinearVelocity",
    "HP_Body_GetLinearVelocity",
    "HP_Body_SetAngularVelocity",
    "HP_Body_GetAngularVelocity",
    "HP_Body_ApplyImpulse",
    "HP_Body_ApplyAngularImpulse",
    "HP_Body_SetActivationState",
    "HP_Body_GetActivationState",
    "HP_Body_SetActivationControl",
    "HP_Body_SetActivationPriority",

    "HP_Constraint_Create",
    "HP_Constraint_Release",
    "HP_Constraint_SetParentBody",
    "HP_Constraint_GetParentBody",
    "HP_Constraint_SetChildBody",
    "HP_Constraint_GetChildBody",
    "HP_Constraint_SetAnchorInParent",
    "HP_Constraint_SetAnchorInChild",
    "HP_Constraint_SetCollisionsEnabled",
    "HP_Constraint_GetCollisionsEnabled",
    "HP_Constraint_GetAppliedImpulses",
    "HP_Constraint_SetEnabled",
    "HP_Constraint_GetEnabled",
    "HP_Constraint_SetAxisMinLimit",
    "HP_Constraint_GetAxisMinLimit",
    "HP_Constraint_SetAxisMaxLimit",
    "HP_Constraint_GetAxisMaxLimit",
    "HP_Constraint_SetAxisMode",
    "HP_Constraint_GetAxisMode",
    "HP_Constraint_SetAxisFriction",
    "HP_Constraint_GetAxisFriction",
    "HP_Constraint_SetAxisMotorType",
    "HP_Constraint_GetAxisMotorType",
    "HP_Constraint_SetAxisMotorPositionTarget",
    "HP_Constraint_GetAxisMotorPositionTarget",
    "HP_Constraint_SetAxisMotorVelocityTarget",
    "HP_Constraint_GetAxisMotorVelocityTarget",
    "HP_Constraint_SetAxisMotorMaxForce",
    "HP_Constraint_GetAxisMotorMaxForce",
    "HP_Constraint_SetAxisMotorStiffness",
    "HP_Constraint_GetAxisMotorStiffness",
    "HP_Constraint_SetAxisMotorDamping",
    "HP_Constraint_GetAxisMotorDamping",
    "HP_Constraint_SetAxisMotorTarget",
    "HP_Constraint_GetAxisMotorTarget",
    "HP_Constraint_SetAxisStiffness",
    "HP_Constraint_SetAxisDamping",

    "HP_World_Create",
    "HP_World_Release",
    "HP_World_GetBodyBuffer",
    "HP_World_SetGravity",
    "HP_World_GetGravity",
    "HP_World_AddBody",
    "HP_World_RemoveBody",
    "HP_World_GetNumBodies",
    "HP_World_CastRay",
    "HP_World_CastRayWithCollector",
    "HP_World_PointProximityWithCollector",
    "HP_World_ShapeProximityWithCollector",
    "HP_World_ShapeCastWithCollector",
    "HP_World_Step",
    "HP_World_SetIdealStepTime",
    "HP_World_SetSpeedLimit",
    "HP_World_GetSpeedLimit",
    "HP_World_GetCollisionEvents",
    "HP_World_GetNextCollisionEvent",
    "HP_Event_AsCollision",
    "HP_World_GetTriggerEvents",
    "HP_World_GetNextTriggerEvent",
    "HP_Event_AsTrigger",

    "HP_QueryCollector_Create",
    "HP_QueryCollector_Release",
    "HP_QueryCollector_GetNumHits",
    "HP_QueryCollector_GetCastRayResult",
    "HP_QueryCollector_GetPointProximityResult",
    "HP_QueryCollector_GetShapeProximityResult",
    "HP_QueryCollector_GetShapeCastResult",

    "HP_Debug_StartRecordingStats",
    "HP_Debug_StopRecordingStats",

    "main",

    "malloc",
    "free",

    "_malloc",
    "_free",

    "__getTypeName",

    "__wasm_call_ctors",
};

const CachedFunctionIndices = comptime_string_map.ComptimeStringMap(comptime_int);

const cached_function_indices: CachedFunctionIndices = blk: {
    var kvs: CachedFunctionIndices.KeyValues = &.{};

    for (function_names, 0..) |name, i|
        kvs = kvs ++ .{.{ name, i }};

    break :blk .initComptime(kvs);
};

pub fn call(self: *HavokPhysics, function: wamr.wasm_function_inst_t, args: anytype) !u32 {
    const args_info = @typeInfo(@TypeOf(args));

    var argv: [args_info.@"struct".fields.len]u32 = undefined;

    inline for (args_info.@"struct".fields, 0..) |field, i|
        argv[i] = @intCast(@field(args, field.name));

    if (!wamr.wasm_runtime_call_wasm(
        self.exec_env,
        function,
        @intCast(argv.len),
        &argv,
    ))
        return error.FunctionCallFailed;

    return if (argv.len > 0)
        argv[0]
    else
        0;
}

pub fn callExported(self: *HavokPhysics, comptime name: [:0]const u8, args: anytype) !u32 {
    const function_index = comptime cached_function_indices.get(name) orelse
        @compileError(fmt.comptimePrint("uncached function name: {s}", .{name}));

    const function = self.cached_functions[function_index] orelse blk: {
        const function = wamr.wasm_runtime_lookup_function(self.module_inst, name.ptr);
        if (function == null)
            return error.FunctionNotFound;

        self.cached_functions[function_index] = function;

        break :blk function;
    };

    return self.call(function, args);
}

pub fn callIndirect(self: *HavokPhysics, idx: u32, args: anytype) !u32 {
    const function = try self.getFunctionIndirect(idx);

    return self.call(function, args);
}

pub fn getFunctionIndirect(self: *HavokPhysics, idx: u32) !wamr.wasm_function_inst_t {
    return self.cached_indirect_functions.get(idx) orelse blk: {
        const function = wamr.wasm_table_get_func_inst(self.module_inst, &self.table_inst, idx);

        try self.cached_indirect_functions.put(idx, function);

        break :blk function;
    };
}

// zig fmt: off

pub fn getStatistics(self: *HavokPhysics, a: u32) !u32 { return self.callExported("HP_GetStatistics", .{a}); }

// zig fmt: on

const std = @import("std");
const mem = std.mem;
const log = std.log;
const time = std.time;
const unicode = std.unicode;
const builtin = std.builtin;
const fmt = std.fmt;
const array_list = std.array_list;
const atomic = std.atomic;
const math = std.math;
const heap = std.heap;
const meta = std.meta;

const wamr = @import("wamr").wasm_export;

const comptime_string_map = @import("ComptimeStringMap.zig");
