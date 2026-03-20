const Physics = @This();

pub const Result = enum(u32) {
    ok,
    fail,
    invalid_handle,
    invalid_args,
    not_implemented,

    /// Resultizes a child type.
    pub fn Wrap(comptime Child: type) type {
        return struct { Result, Child };
    }

    /// Resultizes a two child types in order.
    pub fn WrapPair(comptime Child1: type, comptime Child2: type) type {
        return struct { Result, Child1, Child2 };
    }
};

pub const BoolResult = Result.Wrap(bool);

pub const IntResult = Result.Wrap(i32);
pub const UintResult = Result.Wrap(u32);

/// Shared float type between physical methods and such.
pub const Float = f32;

pub const FloatResult = Result.Wrap(Float);
pub const FloatPairResult = Result.WrapPair(Float, Float);

pub const InternalHandleResult = UintResult;

pub const Vector3 = @Vector(3, Float);

pub const Vector3Result = Result.Wrap(Vector3);
pub const Vector3PairResult = Result.WrapPair(Vector3, Vector3);

pub const Quaternion = @Vector(4, Float);
pub const QuaternionResult = Result.Wrap(Quaternion);

pub const Rotation = @Vector(3, Vector3);
pub const RotationResult = Result.Wrap(Rotation);

pub const QTransform = struct { Vector3, Quaternion };
pub const QTransformResult = Result.Wrap(QTransform);

pub const QSTransform = struct { Vector3, Quaternion, Vector3 };
pub const QSTransformResult = Result.Wrap(QSTransform);

pub const Transform = struct { Vector3, Rotation };
pub const TransformResult = Result.Wrap(Transform);

pub const Aabb = struct { Vector3, Vector3 };
pub const AabbResult = Result.Wrap(Aabb);

fn opacifyVectorElements(
    comptime len: comptime_int,
    comptime Element: type,
    vector: *const @Vector(len, Element),
) [len]Emscripten.Bind.Type.Instance.Opaque {
    const Instance = Emscripten.Bind.Type.Instance;

    var result: [len]Instance.Opaque = undefined;

    inline for (0..len) |i|
        result[i] = Instance.opacify(&vector[i]);

    return result;
}

fn opacifyTupleElements(
    comptime Tuple: type,
    tuple: *const Tuple,
) [@typeInfo(Tuple).@"struct".fields.len]Emscripten.Bind.Type.Instance.Opaque {
    const Instance = Emscripten.Bind.Type.Instance;

    const len = @typeInfo(Tuple).@"struct".fields.len;

    var result: [len]Instance.Opaque = undefined;

    inline for (0..len) |i|
        result[i] = Instance.opacify(&tuple[i]);

    return result;
}

pub const ShapeId = struct { u64 };
pub const ShapeResult = Result.Wrap(ShapeId);

pub const DebugGeometryId = struct { u64 };
pub const DebugGeometryResult = Result.Wrap(DebugGeometryId);

pub const BodyId = struct { u64 };
pub const BodyResult = Result.Wrap(BodyId);

pub const ConstraintId = struct { u64 };
pub const ConstraintResult = Result.Wrap(ConstraintId);

pub const WorldId = struct { u64 };
pub const WorldResult = Result.Wrap(WorldId);

pub const CollectorId = struct { u64 };
pub const CollectorResult = Result.Wrap(CollectorId);

pub const ShapePathIterator = struct {
    /// Shape id.
    u64,
    /// Path data.
    u64,
};

pub const ShapePathIterResult = Result.WrapPair(ShapePathIterator, i32);

pub const ShapeType = enum(u32) {
    collider,
    container,
};

pub const ShapeTypeResult = Result.Wrap(ShapeType);

/// Indicates how the body will behave.
pub const MotionType = enum(u32) {
    static,
    kinematic,
    dynamic,
};

pub const MotionTypeResult = Result.Wrap(MotionType);

pub const EventType = enum(u32) {
    collision_started,
    collision_continued,
    collision_finished,
    trigger_entered,
    trigger_exited,
};

/// Optional motor which attempts to move a body at a specific velocity, or at a specific position.
pub const ConstraintMotorType = enum(u32) {
    none,
    velocity,
    position,
    spring_force,
    spring_acceleration,
};

pub const ConstraintMotorTypeResult = Result.Wrap(ConstraintMotorType);

/// How a specific axis can be constrained.
pub const ConstraintAxisLimitMode = enum(u32) {
    /// The axis is not restricted at all.
    free,
    /// The axis has a minimum/maximum limit.
    limited,
    /// The axis allows no relative movement of the pivots.
    locked,
};

pub const ConstraintAxisLimitResult = Result.Wrap(ConstraintAxisLimitMode);

/// The constraint specific axis to use when setting friction, limit mode, max force, ...
pub const ConstraintAxis = enum(u32) {
    /// Translation along the primary axis of the constraint.
    linear_x,
    /// Translation along the second axis of the constraint.
    linear_y,
    /// Translation along the third axis of the constraint.
    linear_z,
    /// Rotation around the primary axis of the constraint.
    angular_x,
    /// Rotation around the second axis of the constraint.
    angular_y,
    /// Rotation around the third axis of the constraint.
    angular_z,
    /// A 3D distance limit; similar to specifying the `linear_*` axes
    /// individually, but the distance calculation uses all three axes
    /// simultaneously, instead of individually.
    linear_distance,
};

pub const MaterialCombine = enum(u32) {
    /// The final value will be the geometric mean of the two values: `sqrt(AB)`.
    geometric_mean,
    /// The final value will be the smaller of the two: `min(A, B)`.
    minimum,
    /// The final value will be the larger of the two: `max(A, B)`.
    maximum,
    /// The final value will be the arithmetic mean of the two values: `(A + B) / 2`.
    artihemic_mean,
    /// The final value will be the product of the two values: `AB`.
    multiply,
};

pub const ActivationState = enum(u32) {
    active,
    inactive,
};

pub const ActivationStateResult = Result.Wrap(ActivationState);

/// Controls the body sleep mode.
pub const ActivationControl = enum(u32) {
    simulation_controlled,
    always_active,
    always_inactive,
};

pub const MassProperties = struct {
    /// The center of mass, in local space. This is the
    /// point the body will rotate around when applying
    /// an angular velocity.
    Vector3,
    /// The total mass of this object, in kilograms. This
    /// affects how easy it is to move the body. A value
    /// of zero will be used as an infinite mass.
    Float,
    /// The principal moments of inertia of this object
    /// for a unit mass. This determines how easy it is
    /// for the body to rotate. A value of zero on any
    /// axis will be used as infinite inertia about that
    /// axis.
    Vector3,
    /// The rotation rotating from inertia major axis space
    /// to parent space (i.e., the rotation which, when
    /// applied to the 3x3 inertia tensor causes the inertia
    /// tensor to become a diagonal matrix). This determines
    /// how the values of inertia are aligned with the parent
    /// object.
    Quaternion,
};

pub const MassPropertiesResult = Result.Wrap(MassProperties);

pub const Material = struct {
    /// Static friction. Must be overcome before a pair of objects can start sliding
    /// relative to each other; for physically-realistic behaviour, it should be at least as high as the
    /// normal friction value.
    Float,
    /// Dynamic friction. Determines how much an object will slow down when it is in contact with another object.
    /// This is important for simulating realistic physics, such as when an object slides across a surface.
    Float,
    /// Restitution. A factor which describes, the amount of energy that is retained after a collision,
    /// which should be a number between 0 and 1.
    ///
    /// A restitution of 0 means that no energy is retained and the objects will not bounce off each other,
    /// while a restitution of 1 means that all energy is retained and the objects will bounce.
    ///
    /// Note, though, due that due to the simulation implementation, an object with a restitution of 1 may
    /// still lose energy over time.
    Float,
    /// Friction combine mode. Describes how two different friction values should be combined.
    MaterialCombine,
    /// Restitution combine mode. Describes how two different restitution values should be combined.
    MaterialCombine,
};

pub const MaterialResult = Result.Wrap(Material);

pub const FilterInfo = struct {
    /// Membership mask.
    u32,
    /// Collision mask.
    u32,
};

pub const FilterInfoResult = Result.Wrap(FilterInfo);

pub const ContactPoint = struct {
    /// Body id.
    BodyId,
    /// Collider id.
    ShapeId,
    /// Shape hierarchy.
    ShapePathIterator,
    /// Position.
    Vector3,
    /// Normal.
    Vector3,
    /// Triangle index.
    i32,
};

pub const RayCastInput = struct {
    /// Start.
    Vector3,
    /// End.
    Vector3,
    /// Collision filter info.
    FilterInfo,
    /// Should hit triggers.
    bool,
    /// Optional body id to ignore.
    BodyId,
};

pub const RayCastResult = struct {
    /// Fraction.
    Float,
    /// Contact point.
    ContactPoint,
};

pub const RayCastResultResult = Result.Wrap(RayCastResult);

pub const PointProximityInput = struct {
    /// Point position.
    Vector3,
    /// Max distance.
    Float,
    /// Collision filter info.
    FilterInfo,
    /// Should hit triggers.
    bool,
    /// Optional body id to ignore.
    BodyId,
};

pub const PointProximityResult = struct {
    /// Distance.
    Float,
    /// Contact point.
    ContactPoint,
};

pub const PointProximityResultResult = Result.Wrap(PointProximityResult);

pub const ShapeProximityInput = struct {
    /// Shape id.
    ShapeId,
    /// Shape position.
    Vector3,
    /// Shape orientation.
    Quaternion,
    /// Max distance.
    Float,
    /// Should hit triggers.
    bool,
    /// Optional body id to ignore.
    BodyId,
};

pub const ShapeProximityResult = struct {
    /// Distance.
    Float,
    /// Contact point on input shape, in input shape space.
    ContactPoint,
    /// Contact point on hit shape, in world space.
    ContactPoint,
};

pub const ShapeProximityResultResult = Result.Wrap(ShapeProximityResult);

pub const ShapeCastInput = struct {
    /// Shape id.
    ShapeId,
    /// Shape orientation.
    Quaternion,
    /// Cast start position.
    Vector3,
    /// Cast end position.
    Vector3,
    /// Should hit triggers.
    bool,
    /// Optional body id to ignore.
    BodyId,
};

pub const ShapeCastResult = struct {
    /// Fraction.
    Float,
    /// Contact point on input shape, in input shape space.
    ContactPoint,
    /// Contact point on hit shape, in world space.
    ContactPoint,
};

pub const ShapeCastResultResult = Result.Wrap(ShapeCastResult);

pub const CollisionEvent = struct {
    /// Type.
    EventType,
    /// Contact point on body A.
    ContactPoint,
    /// Contact point on body B.
    ContactPoint,
    /// Impulse applied.
    Float,
};

pub const CollisionEventResult = Result.Wrap(CollisionEvent);

pub const TriggerEvent = struct {
    /// Type.
    EventType,
    /// Body A id.
    BodyId,
    /// Body A shape id.
    ShapeId,
    /// Body B id.
    BodyId,
    /// Body B shape id.
    ShapeId,
};

pub const TriggerEventResult = Result.Wrap(TriggerEvent);

pub const DebugGeometryInfo = struct {
    /// Address of vertex (float3) buffer in plugin.
    u32,
    /// Number of vertices in the buffer.
    i32,
    /// Address of triangle (int, int, int) buffer in plugin.
    u32,
    /// Number of triangle in the buffer.
    i32,
};

pub const DebugGeometryInfoResult = Result.Wrap(DebugGeometryInfo);

pub const ObjectStatistics = struct {
    /// Num bodies.
    i32,
    /// Num shapes.
    i32,
    /// Num constraints.
    i32,
    /// Num debug geometries.
    i32,
    /// Num worlds.
    i32,
    /// Num query collectors.
    i32,
};

pub const ObjectStatisticsResult = Result.Wrap(ObjectStatistics);

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
                // std_string,  // Not used in Havok Physics
                // std_wstring, // Not used in Havok Physics
                @"enum",
                tuple,
                // emval,       // NOTE: Used in Havok Physics, implement later
                // memory_view, // Not used in Havok Physics
            };

            pub const Instance = struct {
                // pub const Destructor = union(enum) {
                //     native: *const fn (physics: *Physics, ...) callconv(.c) void,
                //     wasm: wamr.wasm_function_inst_t,
                // };

                /// In Havok Physics we can assume we only use wasm destructor.
                pub const Destructor = wamr.wasm_function_inst_t;

                pub const Wire = struct {
                    value: u64,
                    is_multiple: bool = false,

                    pub fn simplify(self: Wire) u32 {
                        return @intCast(self.value);
                    }
                };

                pub const Opaque = *allowzero const anyopaque;

                name: []const u8,
                kind: Kind,

                /// Physics instance to use in WAMR.
                physics: *Physics,

                destructor: Destructor = null,

                /// Not only purposive for one kind. General-meant size.
                size: ?u32 = null,

                // These are definitely and respectively 1 and 0
                // true_value: ?u32 = null,
                // false_value: ?u32 = null,

                is_signed: bool = false,

                tuple_constructor: wamr.wasm_function_inst_t = null,
                tuple_elements: ?Tuple.Elements.Slice = null,
                tuple_converters: ?Converters = null,

                // pub const emval: *const Instance = &.{
                //     .name = "emscripten::val",
                //     .kind = .emval,
                // };

                fn castOpaqueInner(comptime T: type, @"opaque": Opaque) T {
                    return switch (@typeInfo(T)) {
                        .bool => @intFromPtr(@"opaque") != 0,
                        .int => |info| if (comptime (info.bits <= 32))
                            @bitCast(@as(u32, @truncate(@intFromPtr(@"opaque"))))
                        else if (comptime (info.bits == 64 and @sizeOf(usize) == 8))
                            @bitCast(@as(u64, @intCast(@intFromPtr(@"opaque"))))
                        else
                            castOpaqueSimplex(T, @"opaque"),
                        .float => @bitCast(@as(u32, @truncate(@intFromPtr(@"opaque")))),
                        .@"enum" => @enumFromInt(@as(u32, @truncate(@intFromPtr(@"opaque")))),
                        else => castOpaqueSimplex(T, @"opaque"),
                    };
                }

                pub fn castOpaqueSimplex(comptime T: type, @"opaque": Opaque) T {
                    return @as(*allowzero const T, @ptrCast(@alignCast(@"opaque"))).*;
                }

                /// NOTE: this function must be called by only outside of this struct.
                /// The simplex version of this are not implemented.
                pub fn castOpaque(comptime T: type, @"opaque": Opaque) T {
                    return switch (@typeInfo(T)) {
                        .@"struct" => |struct_info| if (struct_info.is_tuple) blk: {
                            const opaques_ptr: [*]allowzero const Opaque = @ptrCast(@alignCast(@"opaque"));

                            var result: T = undefined;

                            inline for (struct_info.fields, 0..) |field, i|
                                @field(result, field.name) = castOpaque(field.type, opaques_ptr[i]);

                            break :blk result;
                        } else castOpaqueInner(T, @"opaque"),
                        .vector => |vector_info| blk: {
                            const opaques_ptr: [*]allowzero const Opaque = @ptrCast(@alignCast(@"opaque"));

                            var result: T = undefined;

                            inline for (0..vector_info.len) |i|
                                result[i] = castOpaque(vector_info.child, opaques_ptr[i]);

                            break :blk result;
                        },
                        else => castOpaqueInner(T, @"opaque"),
                    };
                }

                pub fn opacify(ptr: anytype) Opaque {
                    return @ptrCast(@constCast(ptr));
                }

                fn opacifyAlloc(
                    comptime T: type,
                    allocator: mem.Allocator,
                    arg: T,
                ) Opaque {
                    const ptr = allocator.create(T) catch unreachable;

                    ptr.* = arg;

                    return @ptrCast(ptr);
                }

                /// Caller owns the return.
                pub fn fromWire(self: *const Instance, wire: Wire) Opaque {
                    const physics = self.physics;

                    return switch (self.kind) {
                        .void => @ptrFromInt(0),
                        .bool => @ptrFromInt(@as(usize, @intCast(wire.value))),
                        .int => blk: {
                            const value_u32: u32 = @truncate(wire.value);

                            const size = self.size.?;

                            const final_u32: u32 =
                                if (self.is_signed) blk_inner: {
                                    const value_i32: i32 = @bitCast(value_u32);

                                    break :blk_inner @bitCast(@as(i32, switch (size) {
                                        1 => @intCast(@as(i8, @truncate(value_i32))),
                                        2 => @intCast(@as(i16, @truncate(value_i32))),
                                        else => value_i32,
                                    }));
                                } else switch (size) {
                                    1 => value_u32 & 0xFF,
                                    2 => value_u32 & 0xFFFF,
                                    else => value_u32,
                                };

                            break :blk @ptrFromInt(@as(usize, final_u32));
                        },
                        .float, .@"enum" => @ptrFromInt(@as(usize, @truncate(wire.value))),
                        .bigint => if (comptime (@sizeOf(usize) == 8))
                            @ptrFromInt(@as(usize, @intCast(wire.value)))
                        else
                            opacifyAlloc(u64, physics.embind_temp_allocator, wire.value),
                        // .std_string => @panic("std_string features are not implemented"),
                        // .std_wstring => @panic("std_wstring features are not implemented"),
                        .tuple => blk: {
                            const ptr: u32 = @intCast(wire.value);

                            const elements = self.tuple_elements.?;
                            const elements_len = elements.len;

                            const converters = self.tuple_converters.?;

                            const allocator = physics.embind_temp_allocator;

                            const opaques = allocator.alloc(Opaque, elements_len) catch unreachable;
                            errdefer allocator.free(opaques);

                            for (elements, 0..) |element, i| {
                                const getter_return_type = converters[i];
                                const getter = element.getter;
                                const getter_context = element.getter_context;

                                opaques[i] = getter_return_type.fromWire(.{
                                    .value = physics.callSimple(getter, .{ getter_context, ptr }) catch unreachable,
                                });
                            }

                            _ = physics.callSimple(self.destructor, .{ptr}) catch unreachable;

                            break :blk @ptrCast(opaques.ptr);
                        },
                        // .emval => @panic("emval features are not implemented"),
                        // .memory_view => @panic("memory_view features are not implemented"),
                    };
                }

                pub fn toWire(self: *const Instance, @"opaque": Opaque, comptime free_ptr: bool) Wire {
                    const physics = self.physics;

                    return switch (self.kind) {
                        .void => .{ .value = 0 },
                        .bool => .{ .value = @intFromBool(castOpaqueSimplex(bool, @"opaque")) },
                        .int, .float, .@"enum" => .{ .value = castOpaqueSimplex(u32, @"opaque") },
                        .bigint => .{ .value = @bitCast(castOpaqueSimplex(u64, @"opaque")), .is_multiple = true },
                        // .std_string => @panic("std_string features are not implemented"),
                        // .std_wstring => @panic("std_wstring features are not implemented"),
                        .tuple => blk: {
                            const elements = self.tuple_elements.?;
                            const elements_len = elements.len;

                            const converters = self.tuple_converters.?;

                            const ptr = physics.callVoid(self.tuple_constructor) catch unreachable;

                            const opaques = castOpaqueSimplex([]const Opaque, @"opaque");

                            for (elements, 0..) |element, i| {
                                const setter_arg_type = converters[elements_len + i];
                                const setter = element.setter;
                                const setter_context = element.setter_context;

                                _ = physics.call(setter, &.{
                                    .{ .value = setter_context },
                                    .{ .value = ptr },
                                    setter_arg_type.toWire(opaques[i], true),
                                }) catch unreachable;
                            }

                            if (free_ptr)
                                _ = physics.callSimple(self.destructor, .{ptr}) catch unreachable;

                            break :blk .{ .value = ptr };
                        },
                        // .emval => @panic("emval features are not implemented"),
                        // .memory_view => @panic("memory_view are not able to convert opaque to wire"),
                    };
                }
            };

            pub const Registry = std.AutoHashMap(Id, *const Instance);

            pub const Converters = []*const Instance;

            pub const ConvertersHandler = *const fn (
                physics: *Physics,
                context: *anyopaque,
                converters: Converters,
            ) Converters;
        };

        pub const Tuple = struct {
            pub const Element = struct {
                getter_return_type: i32,
                getter: wamr.wasm_function_inst_t,
                getter_context: u32,

                setter_arg_type: i32,
                setter: wamr.wasm_function_inst_t,
                setter_context: u32,
            };

            pub const Elements = array_list.Managed(Element);

            name: []u8,

            constructor: wamr.wasm_function_inst_t,
            destructor: wamr.wasm_function_inst_t,

            elements: Elements,
        };

        pub const TupleRegistry = std.AutoHashMap(Type.Id, Tuple);

        pub const Dependency = struct {
            context: *anyopaque,

            i: usize,

            type_ids: []const Type.Id,

            type_converters: Type.Converters,

            type_converters_handler: Type.ConvertersHandler,
            free_type_converters_handler_return: bool,

            registered: *RegisteredCounter,
            unregistered: *RegisteredCounter,
        };

        pub const AwaitingDependencies = std.AutoHashMap(Type.Id, array_list.Managed(*Dependency));

        pub const InvokerContext = struct {
            const max_args = 8;

            invoker: wamr.wasm_function_inst_t,

            function_wire: Type.Instance.Wire,

            return_type_instance: *const Type.Instance,

            arg_type_instances: [max_args]*const Type.Instance = undefined,

            destructors: [max_args]Type.Instance.Destructor = undefined,
        };
    };
};

const MethodImpl = *const fn (physics: *Physics, invoker_context_index: u8, ...) callconv(.c) Emscripten.Bind.Type.Instance.Opaque;

fn noopImpl(_: *Physics, _: u8, ...) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
    return @ptrFromInt(0); // Undefined
}

/// You must get function_index from `cached_function_indices` as we need to ensure the index matches.
fn replaceMethodImpl(
    function_index: usize,
    function: MethodImpl,
) void {
    switch (function_index) {
        cached_function_indices.getUnoptional("HP_GetStatistics") => get_statistics_impl = function,

        cached_function_indices.getUnoptional("_malloc") => _malloc_impl = function,
        cached_function_indices.getUnoptional("_free") => _free_impl = function,

        // Begin shape

        cached_function_indices.getUnoptional("HP_Shape_CreateSphere") => Shape.create_sphere_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_CreateCapsule") => Shape.create_capsule_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_CreateCylinder") => Shape.create_cylinder_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_CreateBox") => Shape.create_box_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_CreateConvexHull") => Shape.create_convex_hull_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_CreateMesh") => Shape.create_mesh_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_CreateHeightField") => Shape.create_height_field_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_SetFilterInfo") => Shape.set_filter_info_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_GetFilterInfo") => Shape.get_filter_info_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_SetMaterial") => Shape.set_material_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_GetMaterial") => Shape.get_material_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_GetDensity") => Shape.get_density_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_SetDensity") => Shape.set_density_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_CreateContainer") => Shape.create_container_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_AddChild") => Shape.add_child_impl = function,
        cached_function_indices.getUnoptional("HP_Shape_RemoveChild") => Shape.remove_child_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_GetNumChildren") => Shape.get_num_children_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_GetChildShape") => Shape.get_child_shape_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_GetType") => Shape.get_type_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_GetBoundingBox") => Shape.get_bounding_box_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_Release") => Shape.release_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_BuildMassProperties") => Shape.build_mass_properties_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_PathIterator_GetNext") => Shape.path_iterator_get_next_impl = function,

        cached_function_indices.getUnoptional("HP_Shape_SetTrigger") => Shape.set_trigger_impl = function,

        // End shape

        // Begin debug geometry

        cached_function_indices.getUnoptional("HP_Shape_CreateDebugDisplayGeometry") => DebugGeometry.create_impl = function,

        cached_function_indices.getUnoptional("HP_DebugGeometry_GetInfo") => DebugGeometry.get_info_impl = function,

        cached_function_indices.getUnoptional("HP_DebugGeometry_Release") => DebugGeometry.release_impl = function,

        // End debug geometry

        // Begin body

        cached_function_indices.getUnoptional("HP_Body_Create") => Body.create_impl = function,

        cached_function_indices.getUnoptional("HP_Body_Release") => Body.release_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetShape") => Body.set_shape_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetShape") => Body.get_shape_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetMotionType") => Body.set_motion_type_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetMotionType") => Body.get_motion_type_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetEventMask") => Body.set_event_mask_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetEventMask") => Body.get_event_mask_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetMassProperties") => Body.set_mass_properties_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetMassProperties") => Body.get_mass_properties_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetLinearDamping") => Body.set_linear_damping_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetLinearDamping") => Body.get_linear_damping_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetAngularDamping") => Body.set_angular_damping_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetAngularDamping") => Body.get_angular_damping_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetGravityFactor") => Body.set_gravity_factor_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetGravityFactor") => Body.get_gravity_factor_impl = function,

        cached_function_indices.getUnoptional("HP_Body_GetWorldTransformOffset") => Body.get_world_transform_offset_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetQTransform") => Body.set_q_transform_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetQTransform") => Body.get_q_transform_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetPosition") => Body.set_position_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetPosition") => Body.get_position_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetOrientation") => Body.set_orientation_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetOrientation") => Body.get_orientation_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetLinearVelocity") => Body.set_linear_velocity_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetLinearVelocity") => Body.get_linear_velocity_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetAngularVelocity") => Body.set_angular_velocity_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetAngularVelocity") => Body.get_angular_velocity_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetTargetQTransform") => Body.set_target_q_transform_impl = function,

        cached_function_indices.getUnoptional("HP_Body_ApplyImpulse") => Body.apply_impulse_impl = function,
        cached_function_indices.getUnoptional("HP_Body_ApplyAngularImpulse") => Body.apply_angular_impulse_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetActivationState") => Body.set_activation_state_impl = function,
        cached_function_indices.getUnoptional("HP_Body_GetActivationState") => Body.get_activation_state_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetActivationControl") => Body.set_activation_control_impl = function,

        cached_function_indices.getUnoptional("HP_Body_SetActivationPriority") => Body.set_activation_priority_impl = function,

        // End body

        // Begin constraint

        cached_function_indices.getUnoptional("HP_Constraint_Create") => Constraint.create_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_Release") => Constraint.release_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetParentBody") => Constraint.set_parent_body_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetParentBody") => Constraint.set_parent_body_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetChildBody") => Constraint.set_child_body_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetChildBody") => Constraint.get_child_body_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetEnabled") => Constraint.set_enabled_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetEnabled") => Constraint.get_enabled_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetCollisionsEnabled") => Constraint.set_collisions_enabled_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetCollisionsEnabled") => Constraint.get_collisions_enabled_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMode") => Constraint.set_axis_mode_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMode") => Constraint.get_axis_mode_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorType") => Constraint.set_axis_motor_type_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorType") => Constraint.get_axis_motor_type_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_GetAppliedImpulses") => Constraint.get_applied_impulses_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetAnchorInParent") => Constraint.set_anchor_in_parent_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAnchorInChild") => Constraint.set_anchor_in_child_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_GetAxisFriction") => Constraint.get_axis_friction_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMinLimit") => Constraint.get_axis_min_limit_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMaxLimit") => Constraint.get_axis_max_limit_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorTarget") => Constraint.get_axis_motor_target_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorMaxForce") => Constraint.get_axis_motor_max_force_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorPositionTarget") => Constraint.get_axis_motor_position_target_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorVelocityTarget") => Constraint.get_axis_motor_velocity_target_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorStiffness") => Constraint.get_axis_motor_stiffness_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorDamping") => Constraint.get_axis_motor_damping_impl = function,

        cached_function_indices.getUnoptional("HP_Constraint_SetAxisFriction") => Constraint.set_axis_friction_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMinLimit") => Constraint.set_axis_min_limit_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMaxLimit") => Constraint.set_axis_max_limit_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisStiffness") => Constraint.set_axis_stiffness = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisDamping") => Constraint.set_axis_damping = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorTarget") => Constraint.set_axis_motor_target_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorMaxForce") => Constraint.set_axis_motor_max_force_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorPositionTarget") => Constraint.set_axis_motor_position_target_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorVelocityTarget") => Constraint.set_axis_motor_velocity_target_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorStiffness") => Constraint.set_axis_motor_stiffness_impl = function,
        cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorDamping") => Constraint.set_axis_motor_damping_impl = function,

        // End constraint

        // Begin world

        cached_function_indices.getUnoptional("HP_World_Create") => World.create_impl = function,

        cached_function_indices.getUnoptional("HP_World_Release") => World.release_impl = function,

        cached_function_indices.getUnoptional("HP_World_GetBodyBuffer") => World.get_body_buffer_impl = function,

        cached_function_indices.getUnoptional("HP_World_SetGravity") => World.set_gravity_impl = function,

        cached_function_indices.getUnoptional("HP_World_AddBody") => World.add_body_impl = function,
        cached_function_indices.getUnoptional("HP_World_RemoveBody") => World.remove_body_impl = function,

        cached_function_indices.getUnoptional("HP_World_GetNumBodies") => World.get_num_bodies_impl = function,

        cached_function_indices.getUnoptional("HP_World_CastRay") => World.cast_ray_impl = function,

        cached_function_indices.getUnoptional("HP_World_CastRayWithCollector") => World.cast_ray_with_collector_impl = function,

        cached_function_indices.getUnoptional("HP_World_PointProximityWithCollector") => World.point_proximity_with_collector_impl = function,
        cached_function_indices.getUnoptional("HP_World_ShapeProximityWithCollector") => World.shape_proximity_with_collector_impl = function,

        cached_function_indices.getUnoptional("HP_World_ShapeCastWithCollector") => World.shape_cast_with_collector_impl = function,

        cached_function_indices.getUnoptional("HP_World_Step") => World.step_impl = function,
        cached_function_indices.getUnoptional("HP_World_SetIdealStepTime") => World.set_ideal_step_time_impl = function,

        cached_function_indices.getUnoptional("HP_World_SetSpeedLimit") => World.set_speed_limit_impl = function,
        cached_function_indices.getUnoptional("HP_World_GetSpeedLimit") => World.get_speed_limit_impl = function,

        cached_function_indices.getUnoptional("HP_World_GetCollisionEvents") => World.get_collision_events_impl = function,
        cached_function_indices.getUnoptional("HP_World_GetNextCollisionEvent") => World.get_next_collision_event_impl = function,

        cached_function_indices.getUnoptional("HP_World_GetTriggerEvents") => World.get_trigger_events_impl = function,
        cached_function_indices.getUnoptional("HP_World_GetNextTriggerEvent") => World.get_next_trigger_event_impl = function,

        // End world

        // Begin event

        cached_function_indices.getUnoptional("HP_Event_AsTrigger") => Event.as_trigger = function,

        cached_function_indices.getUnoptional("HP_Event_AsCollision") => Event.as_collision = function,

        // End event

        // Begin query collector

        cached_function_indices.getUnoptional("HP_QueryCollector_Create") => QueryCollector.create_impl = function,

        cached_function_indices.getUnoptional("HP_QueryCollector_Release") => QueryCollector.release_impl = function,

        cached_function_indices.getUnoptional("HP_QueryCollector_GetNumHits") => QueryCollector.get_num_hits_impl = function,

        cached_function_indices.getUnoptional("HP_QueryCollector_GetCastRayResult") => QueryCollector.get_cast_ray_result_impl = function,

        cached_function_indices.getUnoptional("HP_QueryCollector_GetPointProximityResult") => QueryCollector.get_point_proximity_result_impl = function,
        cached_function_indices.getUnoptional("HP_QueryCollector_GetShapeProximityResult") => QueryCollector.get_shape_proximity_result_impl = function,

        cached_function_indices.getUnoptional("HP_QueryCollector_GetShapeCastResult") => QueryCollector.get_shape_cast_result_impl = function,

        // End query collector

        // Begin debug

        cached_function_indices.getUnoptional("HP_Debug_StartRecordingStats") => Debug.start_recording_stats_impl = function,
        cached_function_indices.getUnoptional("HP_Debug_StopRecordingStats") => Debug.stop_recording_stats_impl = function,

        // End debug

        else => undefined,
    }
}

// Begin flats

var get_statistics_impl = &noopImpl;

/// Return statistics on the number of allocated objects.
pub fn getStatistics(self: *Physics) ObjectStatisticsResult {
    const TypeInstance = Emscripten.Bind.Type.Instance;

    const castOpaque = TypeInstance.castOpaque;

    return castOpaque(ObjectStatisticsResult, get_statistics_impl(self, comptime @intCast(cached_function_indices.getUnoptional("HP_GetStatistics"))));
}

// End flats

pub const MemoryReference = struct {
    /// The offset from the beginning of the heap.
    u32,
    /// The number of identically-sized objects the buffer contains.
    u32,
};

var _malloc_impl = &noopImpl;
var _free_impl = &noopImpl;

/// Mallocs a memory.
pub fn wasmMalloc(self: *Physics, len: u32) u32 {
    const TypeInstance = Emscripten.Bind.Type.Instance;

    const castOpaque = TypeInstance.castOpaque;

    return castOpaque(u32, _malloc_impl(
        self,
        comptime @intCast(cached_function_indices.getUnoptional("_malloc")),
        &len,
    ));
}

/// Frees a memory.
pub fn wasmFree(self: *Physics, offset: u32) void {
    _ = _free_impl(
        self,
        comptime @intCast(cached_function_indices.getUnoptional("_free")),
        &offset,
    );
}

pub fn transformVertices(self: *Physics, vertices: Shape.Vertices) !MemoryReference {
    const vertices_len_u32: u32 = @intCast(vertices.len);

    const num_floats = 3 * vertices_len_u32;

    const wasm_ptr = self.wasmMalloc(@sizeOf(Float) * num_floats);

    const raw_ptr = wamr.wasm_runtime_addr_app_to_native(self.module_inst, wasm_ptr);
    if (raw_ptr == null)
        return error.InvalidMemoryAccess;

    const dst: [*]Float = @ptrCast(@alignCast(raw_ptr));
    const src: [*]const Float = @ptrCast(@alignCast(vertices.ptr));

    @memcpy(dst[0..num_floats], src[0..num_floats]);

    return .{ wasm_ptr, num_floats };
}

pub const Shape = struct {
    physics: *Physics,

    pub const Vertices = []const Vector3;

    var create_sphere_impl = &noopImpl;
    var create_capsule_impl = &noopImpl;
    var create_cylinder_impl = &noopImpl;
    var create_box_impl = &noopImpl;
    var create_convex_hull_impl = &noopImpl;
    var create_mesh_impl = &noopImpl;
    var create_height_field_impl = &noopImpl;
    var create_container_impl = &noopImpl;

    var set_filter_info_impl = &noopImpl;
    var get_filter_info_impl = &noopImpl;

    var set_material_impl = &noopImpl;
    var get_material_impl = &noopImpl;

    var set_density_impl = &noopImpl;
    var get_density_impl = &noopImpl;

    var add_child_impl = &noopImpl;
    var remove_child_impl = &noopImpl;

    var get_num_children_impl = &noopImpl;

    var get_child_shape_impl = &noopImpl;

    var get_type_impl = &noopImpl;

    var get_bounding_box_impl = &noopImpl;

    var release_impl = &noopImpl;

    var build_mass_properties_impl = &noopImpl;

    var path_iterator_get_next_impl = &noopImpl;

    var set_trigger_impl = &noopImpl;

    /// Creates geometry representing a sphere.
    pub fn createSphere(self: *const @This(), center: Vector3, radius: Float) ShapeResult {
        const center_opaque_array = opacifyVectorElements(3, Float, &center);
        const center_opaque_vector: []const Opaque = &center_opaque_array;

        return castOpaque(ShapeResult, create_sphere_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Shape_CreateSphere"),
            &center_opaque_vector,
            &radius,
        ));
    }

    /// Creates a geometry representing a capsule.
    pub fn createCapsule(self: *const @This(), point_a: Vector3, point_b: Vector3, radius: Float) ShapeResult {
        const point_a_opaque_array = opacifyVectorElements(3, Float, &point_a);
        const point_a_opaque_vector: []const Opaque = &point_a_opaque_array;

        const point_b_opaque_array = opacifyVectorElements(3, Float, &point_b);
        const point_b_opaque_vector: []const Opaque = &point_b_opaque_array;

        return castOpaque(ShapeResult, create_capsule_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Shape_CreateCapsule"),
            &point_a_opaque_vector,
            &point_b_opaque_vector,
            &radius,
        ));
    }

    /// Creates a geometry representing a cylinder.
    pub fn createCylinder(self: *const @This(), point_a: Vector3, point_b: Vector3, radius: Float) ShapeResult {
        const point_a_opaque_array = opacifyVectorElements(3, Float, &point_a);
        const point_a_opaque_vector: []const Opaque = &point_a_opaque_array;

        const point_b_opaque_array = opacifyVectorElements(3, Float, &point_b);
        const point_b_opaque_vector: []const Opaque = &point_b_opaque_array;

        return castOpaque(ShapeResult, create_cylinder_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Shape_CreateCylinder"),
            &point_a_opaque_vector,
            &point_b_opaque_vector,
            &radius,
        ));
    }

    /// Creates a geometry representing a box.
    pub fn createBox(
        self: *const @This(),
        /// Position of the box center (in shape space).
        center: Vector3,
        /// Orientation of the box (in shape space).
        rotation: Quaternion,
        /// Total size of the box.
        extents: Vector3,
    ) ShapeResult {
        const center_opaque_array = opacifyVectorElements(3, Float, &center);
        const center_opaque_vector: []const Opaque = &center_opaque_array;

        const rotation_opaque_array = opacifyVectorElements(4, Float, &rotation);
        const rotation_opaque_vector: []const Opaque = &rotation_opaque_array;

        const extents_opaque_array = opacifyVectorElements(3, Float, &extents);
        const extents_opaque_vector: []const Opaque = &extents_opaque_array;

        return castOpaque(ShapeResult, create_box_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Shape_CreateBox"),
            &center_opaque_vector,
            &rotation_opaque_vector,
            &extents_opaque_vector,
        ));
    }

    /// Creates a geometry which encloses all the `vertices`.
    pub fn createConvexHull(
        self: *const @This(),
        /// Need to be allocated within the WASM memory using `wasmMalloc` and should refer to a buffer populated with `Vector`.
        /// Zig's is `Vertices`.
        vertices: u32,
        num_vertices: i32,
    ) ShapeResult {
        return castOpaque(ShapeResult, create_convex_hull_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Shape_CreateConvexHull"),
            &vertices,
            &num_vertices,
        ));
    }

    /// Creates a geometry representing the surface of a mesh.
    pub fn createMesh(
        self: *const @This(),
        /// Need to be allocated within the WASM memory using `wasmMalloc` and should refer to a buffer populated with `Vector`.
        /// Zig's is `Vertices`.
        vertices: u32,
        num_vertices: i32,
        /// Should be triples of 32-bit integers which index into `vertices` (need to be allocated).
        triangles: u32,
        num_triangles: i32,
    ) ShapeResult {
        return castOpaque(ShapeResult, create_mesh_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Shape_CreateMesh"),
            &vertices,
            &num_vertices,
            &triangles,
            &num_triangles,
        ));
    }

    /// Creates a geometry representing a height map.
    pub fn createHeightField(
        self: *const @This(),
        num_x_samples: usize,
        num_z_samples: usize,
        /// X and Z components should be converted from integer space to shape space.
        /// Y supplies a scaling factor for the height.
        scale: Vector3,
        /// Should be a buffer of floats, of size (num_x_samples * num_z_samples), describing heights at (x, z) of
        /// [(0, 0), (1, 0), ... (num_x_samples - 1, 0), (0, 1), (1, 1) ... (num_x_samples - 1, 1) ... (num_x_samples - 1, num_z_samples - 1)].
        heights: u32,
    ) ShapeResult {
        const scale_opaque_array = opacifyVectorElements(3, Float, &scale);
        const scale_opaque_vector: []const Opaque = &scale_opaque_array;

        return castOpaque(ShapeResult, create_height_field_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Shape_CreateHeightField"),
            &num_x_samples,
            &num_z_samples,
            &scale_opaque_vector,
            &heights,
        ));
    }

    /// Creates a "container" shape - this shape does not have any inherent geometry, but it can contain other shapes.
    pub fn createContainer(self: *const @This()) ShapeResult {
        return castOpaque(ShapeResult, create_container_impl(self.physics, comptime cached_function_indices.getUnoptional("HP_Shape_CreateContainer")));
    }

    /// Release a shape, freeing memory if it is unused.
    pub fn release(self: *const @This(), id: ShapeId) Result {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, release_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_Release")),
            &id_opaque_tuple,
        ));
    }

    /// Get the type of the shape.
    pub fn getType(self: *const @This(), id: ShapeId) ShapeTypeResult {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ShapeTypeResult, get_type_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_GetType")),
            &id_opaque_tuple,
        ));
    }

    /// Adds the `child_id` to the container `container` at the transform `container_from_child`.
    pub fn addChild(self: *const @This(), container_id: ShapeId, child_id: ShapeId, container_from_child: QSTransform) Result {
        const container_id_opaque_array = opacifyTupleElements(ShapeId, &container_id);
        const container_id_opaque_tuple: []const Opaque = &container_id_opaque_array;

        const child_id_opaque_array = opacifyTupleElements(ShapeId, &child_id);
        const child_id_opaque_tuple: []const Opaque = &child_id_opaque_array;

        const container_from_child_1_opaque_array = opacifyVectorElements(3, Float, &container_from_child[0]);
        const container_from_child_2_opaque_array = opacifyVectorElements(4, Float, &container_from_child[1]);
        const container_from_child_3_opaque_array = opacifyVectorElements(3, Float, &container_from_child[2]);

        const container_from_child_1_opaque_vector: []const Opaque = &container_from_child_1_opaque_array;
        const container_from_child_2_opaque_vector: []const Opaque = &container_from_child_2_opaque_array;
        const container_from_child_3_opaque_vector: []const Opaque = &container_from_child_3_opaque_array;

        const container_from_child_opaque_tuple: []const Opaque = &.{
            opacify(&container_from_child_1_opaque_vector),
            opacify(&container_from_child_2_opaque_vector),
            opacify(&container_from_child_3_opaque_vector),
        };

        return castOpaque(Result, add_child_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_AddChild")),
            &container_id_opaque_tuple,
            &child_id_opaque_tuple,
            &container_from_child_opaque_tuple,
        ));
    }

    /// Removes the child at index `child_index` inside `container`.
    pub fn removeChild(self: *const @This(), container_id: ShapeId, child_index: u32) Result {
        const container_id_opaque_array = opacifyTupleElements(ShapeId, &container_id);
        const container_id_opaque_tuple: []const Opaque = &container_id_opaque_array;

        return castOpaque(Result, remove_child_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_RemoveChild")),
            &container_id_opaque_tuple,
            &child_index,
        ));
    }

    /// Get the number of children of the container.
    pub fn getNumChildren(self: *const @This(), container_id: ShapeId) IntResult {
        const container_id_opaque_array = opacifyTupleElements(ShapeId, &container_id);
        const container_id_opaque_tuple: []const Opaque = &container_id_opaque_array;

        return castOpaque(IntResult, get_num_children_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_GetNumChildren")),
            &container_id_opaque_tuple,
        ));
    }

    /// Returns the shape id of the child shape at index `child_index` in the container.
    pub fn getChildShape(self: *const @This(), container_id: ShapeId, child_index: u32) ShapeResult {
        const container_id_opaque_array = opacifyTupleElements(ShapeId, &container_id);
        const container_id_opaque_tuple: []const Opaque = &container_id_opaque_array;

        return castOpaque(ShapeResult, get_child_shape_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_GetChildShape")),
            &container_id_opaque_tuple,
            &child_index,
        ));
    }

    // pub fn setChildQSTransform(self: *const @This(), a: u32, b: u32, c: u32) !u32 {
    //     return self.physics.callExported("HP_Shape_SetChildQSTransform", .{ a, b, c });
    // }
    // pub fn getChildQSTransform(self: *const @This(), a: u32, b: u32, c: u32) !u32 {
    //     return self.physics.callExported("HP_Shape_GetChildQSTransform", .{ a, b, c });
    // }

    /// Sets the collision info for the shape to the information in `filter_info`.
    /// This can prevent collisions between shapes and queries, depending on how you have configured the filter.
    pub fn setFilterInfo(self: *const @This(), id: ShapeId, filter_info: FilterInfo) Result {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const filter_info_opaque_array = opacifyTupleElements(FilterInfo, &filter_info);
        const filter_info_opaque_tuple: []const Opaque = &filter_info_opaque_array;

        return castOpaque(Result, set_filter_info_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_SetFilterInfo")),
            &id_opaque_tuple,
            &filter_info_opaque_tuple,
        ));
    }

    /// Get the collision filter info for a shape.
    pub fn getFilterInfo(self: *const @This(), id: ShapeId) FilterInfoResult {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(FilterInfoResult, get_filter_info_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_GetFilterInfo")),
            &id_opaque_tuple,
        ));
    }

    /// Sets the material of the shape to the provided material.
    pub fn setMaterial(self: *const @This(), id: ShapeId, material: Material) Result {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const material_opaque_array = opacifyTupleElements(Material, &material);
        const material_opaque_tuple: []const Opaque = &material_opaque_array;

        return castOpaque(Result, set_material_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_SetMaterial")),
            &id_opaque_tuple,
            &material_opaque_tuple,
        ));
    }

    /// Get the material associated with the shape.
    pub fn getMaterial(self: *const @This(), id: ShapeId) MaterialResult {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(MaterialResult, get_material_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_GetMaterial")),
            &id_opaque_tuple,
        ));
    }

    /// Set the density of the shape. Used when calling `buildMassProperties`.
    pub fn setDensity(self: *const @This(), id: ShapeId, density: Float) Result {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_density_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_SetDensity")),
            &id_opaque_tuple,
            &density,
        ));
    }

    /// Get the density of the shape.
    pub fn getDensity(self: *const @This(), id: ShapeId) FloatResult {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(FloatResult, get_density_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_GetDensity")),
            &id_opaque_tuple,
        ));
    }

    /// Retrieve the axis aligned bounding box of the shape located at `world_from_shape`.
    pub fn getBoundingBox(self: *const @This(), id: ShapeId, world_from_shape: QTransform) AabbResult {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const world_from_shape_1_opaque_array = opacifyVectorElements(3, Float, &world_from_shape[0]);
        const world_from_shape_2_opaque_array = opacifyVectorElements(4, Float, &world_from_shape[1]);

        const world_from_shape_1_opaque_vector: []const Opaque = &world_from_shape_1_opaque_array;
        const world_from_shape_2_opaque_vector: []const Opaque = &world_from_shape_2_opaque_array;

        const world_from_shape_opaque_tuple: []const Opaque = &.{
            opacify(&world_from_shape_1_opaque_vector),
            opacify(&world_from_shape_2_opaque_vector),
        };

        return castOpaque(AabbResult, get_bounding_box_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_GetBoundingBox")),
            &id_opaque_tuple,
            &world_from_shape_opaque_tuple,
        ));
    }

    // pub fn castRay(self: *const @This(), a: u32, b: u32, c: u32, d: u32, e: u32) !u32 {
    //     return self.physics.callExported("HP_Shape_CastRay", .{ a, b, c, d, e });
    // }

    /// Calculates the mass properties of the shape.
    pub fn buildMassProperties(self: *const @This(), id: ShapeId) MassPropertiesResult {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(MassPropertiesResult, build_mass_properties_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_BuildMassProperties")),
            &id_opaque_tuple,
        ));
    }

    /// Allows descending a hierarchy of shape containers, advancing `current_item` to the next entry.
    pub fn pathIteratorGetNext(self: *const @This(), current_item: ShapePathIterator) ShapePathIterResult {
        const current_item_opaque_array = opacifyTupleElements(ShapePathIterator, &current_item);
        const current_item_opaque_tuple: []const Opaque = &current_item_opaque_array;

        return castOpaque(ShapePathIterResult, path_iterator_get_next_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_PathIterator_GetNext")),
            &current_item_opaque_tuple,
        ));
    }

    /// Mark this shape as a trigger. A trigger will generate events, rather than applying impulses to prevent overlap.
    /// Any material set on this shape will be unused. This has no effect on container shapes, as they don't have any
    /// geometry themselves.
    ///
    /// NOTE: Currently, when one of the shapes overlapping a trigger is a mesh shape, one event will be raised per
    /// overlapping triangle. This is subject to change, as it can cause performance issues.
    pub fn setTrigger(self: *const @This(), id: ShapeId, is_trigger: bool) Result {
        const id_opaque_array = opacifyTupleElements(ShapeId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_trigger_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_SetTrigger")),
            &id_opaque_tuple,
            &is_trigger,
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

pub const Body = struct {
    physics: *Physics,

    var create_impl = &noopImpl;

    var release_impl = &noopImpl;

    var set_shape_impl = &noopImpl;
    var get_shape_impl = &noopImpl;

    var set_motion_type_impl = &noopImpl;
    var get_motion_type_impl = &noopImpl;

    var set_event_mask_impl = &noopImpl;
    var get_event_mask_impl = &noopImpl;

    var set_mass_properties_impl = &noopImpl;
    var get_mass_properties_impl = &noopImpl;

    var set_linear_damping_impl = &noopImpl;
    var get_linear_damping_impl = &noopImpl;

    var set_angular_damping_impl = &noopImpl;
    var get_angular_damping_impl = &noopImpl;

    var set_gravity_factor_impl = &noopImpl;
    var get_gravity_factor_impl = &noopImpl;

    var get_world_transform_offset_impl = &noopImpl;

    var set_q_transform_impl = &noopImpl;
    var get_q_transform_impl = &noopImpl;

    var set_position_impl = &noopImpl;
    var get_position_impl = &noopImpl;

    var set_orientation_impl = &noopImpl;
    var get_orientation_impl = &noopImpl;

    var set_linear_velocity_impl = &noopImpl;
    var get_linear_velocity_impl = &noopImpl;

    var set_angular_velocity_impl = &noopImpl;
    var get_angular_velocity_impl = &noopImpl;

    var set_target_q_transform_impl = &noopImpl;

    var apply_impulse_impl = &noopImpl;
    var apply_angular_impulse_impl = &noopImpl;

    var set_activation_state_impl = &noopImpl;
    var get_activation_state_impl = &noopImpl;

    var set_activation_control_impl = &noopImpl;

    var set_activation_priority_impl = &noopImpl;

    /// Allocates a new body.
    pub fn create(self: *const @This()) BodyResult {
        return castOpaque(BodyResult, create_impl(self.physics, comptime cached_function_indices.getUnoptional("HP_Body_Create")));
    }

    /// Releases a body, potentially freeing the memory. Will not remove from the world if body is in use.
    pub fn release(self: *const @This(), id: BodyId) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, release_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_Release"),
            &id_opaque_tuple,
        ));
    }

    /// Sets body to use a particular shape.
    /// A body can only have a single shape; multiple shapes should be wrapped in a container shape.
    pub fn setShape(self: *const @This(), id: BodyId, shape_id: ShapeId) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const shape_id_opaque_array = opacifyTupleElements(ShapeId, &shape_id);
        const shape_id_opaque_tuple: []const Opaque = &shape_id_opaque_array;

        return castOpaque(Result, set_shape_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetShape"),
            &id_opaque_tuple,
            &shape_id_opaque_tuple,
        ));
    }

    /// Get the shape in use by a body.
    pub fn getShape(self: *const @This(), id: BodyId) Shape.OurResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Shape.OurResult, get_shape_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetShape"),
            &id_opaque_tuple,
        ));
    }

    /// Set the body to behave according to the motion type.
    pub fn setMotionType(self: *const @This(), id: BodyId, motion_type: MotionType) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_motion_type_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetMotionType"),
            &id_opaque_tuple,
            &motion_type,
        ));
    }

    /// Get the current motion type of the body.
    pub fn getMotionType(self: *const @This(), id: BodyId) MotionTypeResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(MotionTypeResult, get_motion_type_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetMotionType"),
            &id_opaque_tuple,
        ));
    }

    /// Configure a body to raise events, based on `event_mask`. Bodies will not raise events by default.
    /// The event mask should be the integer value of the `EventType` enum for all the events you wish to opt into, ORed together.
    pub fn setEventMask(self: *const @This(), id: BodyId, event_mask: u32) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_event_mask_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetEventMask"),
            &id_opaque_tuple,
            &event_mask,
        ));
    }

    /// Get the event mask of a body.
    pub fn getEventMask(self: *const @This(), id: BodyId) UintResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(UintResult, get_event_mask_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetEventMask"),
            &id_opaque_tuple,
        ));
    }

    /// Configures the body to use the supplied mass properties.
    pub fn setMassProperties(self: *const @This(), id: BodyId, mass_properties: MassProperties) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const mass_properties_1_opaque_array = opacifyVectorElements(3, Float, &mass_properties[0]);
        const mass_properties_3_opaque_array = opacifyVectorElements(3, Float, &mass_properties[2]);
        const mass_properties_4_opaque_array = opacifyVectorElements(4, Float, &mass_properties[3]);

        const mass_properties_1_opaque_vector: []const Opaque = &mass_properties_1_opaque_array;
        const mass_properties_3_opaque_vector: []const Opaque = &mass_properties_3_opaque_array;
        const mass_properties_4_opaque_vector: []const Opaque = &mass_properties_4_opaque_array;

        const mass_properties_opaque_tuple: []const Opaque = &.{
            opacify(&mass_properties_1_opaque_vector),
            opacify(&mass_properties[1]),
            opacify(&mass_properties_3_opaque_vector),
            opacify(&mass_properties_4_opaque_vector),
        };

        return castOpaque(Result, set_mass_properties_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetMassProperties"),
            &id_opaque_tuple,
            &mass_properties_opaque_tuple,
        ));
    }

    /// Get the mass properties currently used by the body.
    pub fn getMassProperties(self: *const @This(), id: BodyId) MassPropertiesResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(MassPropertiesResult, get_mass_properties_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetMassProperties"),
            &id_opaque_tuple,
        ));
    }

    /// Sets the linear damping of a body.
    /// This will reduce the linear velocity of the body by some fraction every step,
    /// even when the body is not in collision.
    pub fn setLinearDamping(self: *const @This(), id: BodyId, damping: Float) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_linear_damping_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetLinearDamping"),
            &id_opaque_tuple,
            &damping,
        ));
    }

    /// Get the linear damping of a body.
    pub fn getLinearDamping(self: *const @This(), id: BodyId) FloatResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(FloatResult, get_linear_damping_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetLinearDamping"),
            &id_opaque_tuple,
        ));
    }

    /// Sets the angular damping of a body.
    /// This will reduce the angular velocity of the body by some fraction every step,
    /// event when the body is not in collision.
    pub fn setAngularDamping(self: *const @This(), id: BodyId, damping: Float) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_angular_damping_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetAngularDamping"),
            &id_opaque_tuple,
            &damping,
        ));
    }

    /// Get the angular damping of a body.
    pub fn getAngularDamping(self: *const @This(), id: BodyId) FloatResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(FloatResult, get_angular_damping_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetAngularDamping"),
            &id_opaque_tuple,
        ));
    }

    /// Set the gravity factor of the body. This will scale the effect of gravity on the body during the simulation step.
    pub fn setGravityFactor(self: *const @This(), id: BodyId, factor: Float) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_gravity_factor_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetGravityFactor"),
            &id_opaque_tuple,
            &factor,
        ));
    }

    /// Get the gravity factor of a body.
    pub fn getGravityFactor(self: *const @This(), id: BodyId) FloatResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(FloatResult, get_gravity_factor_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetGravityFactor"),
            &id_opaque_tuple,
        ));
    }

    // pub fn getWorld(self: *@This(), a: u32, b: u32) !u32 {
    //     return self.physics.callExported("HP_Body_GetWorld", .{ a, b });
    // }

    /// Return the offet of the memory containing the body's transform from the
    /// world's body buffer. This is only valid if the body is in the world.
    pub fn getWorldTransformOffset(self: *const @This(), id: BodyId) InternalHandleResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(InternalHandleResult, get_world_transform_offset_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetWorldTransformOffset"),
            &id_opaque_tuple,
        ));
    }

    /// Set the transform of the body (in world space).
    pub fn setQTransform(self: *const @This(), id: BodyId, transform: QTransform) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const transform_1_opaque_array = opacifyVectorElements(3, Float, &transform[0]);
        const transform_2_opaque_array = opacifyVectorElements(4, Float, &transform[1]);

        const transform_1_opaque_vector: []const Opaque = &transform_1_opaque_array;
        const transform_2_opaque_vector: []const Opaque = &transform_2_opaque_array;

        const transform_opaque_tuple: []const Opaque = &.{
            opacify(&transform_1_opaque_vector),
            opacify(&transform_2_opaque_vector),
        };

        return castOpaque(Result, set_q_transform_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetQTransform"),
            &id_opaque_tuple,
            &transform_opaque_tuple,
        ));
    }

    /// Get the transform of the body (in world space).
    pub fn getQTransform(self: *const @This(), id: BodyId) QTransformResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(QTransformResult, get_q_transform_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetQTransform"),
            &id_opaque_tuple,
        ));
    }

    /// Sets the position of the body (in world space).
    /// If you wish to also set the orientation, you should use `setQTransform`.
    pub fn setPosition(self: *const @This(), id: BodyId, position: Vector3) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const position_opaque_array = opacifyVectorElements(3, Float, &position);
        const position_opaque_vector: []const Opaque = &position_opaque_array;

        return castOpaque(Result, set_position_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetPosition"),
            &id_opaque_tuple,
            &position_opaque_vector,
        ));
    }

    /// Gets the position of the body (in world space).
    /// Consider using `getQTransform` instead.
    pub fn getPosition(self: *const @This(), id: BodyId) Vector3Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Vector3Result, get_position_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetPosition"),
            &id_opaque_tuple,
        ));
    }

    /// Sets the orientation of the body (in world space).
    /// If you wish to also set the position, you should use `setQTransform`.
    pub fn setOrientation(self: *const @This(), id: BodyId, orientation: Quaternion) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const orientation_opaque_array = opacifyVectorElements(4, Float, &orientation);
        const orientation_opaque_vector: []const Opaque = &orientation_opaque_array;

        return castOpaque(Result, set_orientation_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetOrientation"),
            &id_opaque_tuple,
            &orientation_opaque_vector,
        ));
    }

    /// Gets the orientation of the body (in world space).
    /// Consider using `getQTransform` instead.
    pub fn getOrientation(self: *const @This(), id: BodyId) QuaternionResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(QuaternionResult, get_orientation_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetOrientation"),
            &id_opaque_tuple,
        ));
    }

    /// Set the linear velocity of a body. No effect on `static` bodies.
    pub fn setLinearVelocity(self: *const @This(), id: BodyId, velocity: Vector3) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const velocity_opaque_array = opacifyVectorElements(3, Float, &velocity);
        const velocity_opaque_vector: []const Opaque = &velocity_opaque_array;

        return castOpaque(Result, set_linear_velocity_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetLinearVelocity"),
            &id_opaque_tuple,
            &velocity_opaque_vector,
        ));
    }

    /// Get the linear velocity of a body.
    pub fn getLinearVelocity(self: *const @This(), id: BodyId) Vector3Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Vector3Result, get_linear_velocity_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetLinearVelocity"),
            &id_opaque_tuple,
        ));
    }

    /// Set the angular velocity of a body. No effect on `static` bodies.
    pub fn setAngularVelocity(self: *const @This(), id: BodyId, velocity: Vector3) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const velocity_opaque_array = opacifyVectorElements(3, Float, &velocity);
        const velocity_opaque_vector: []const Opaque = &velocity_opaque_array;

        return castOpaque(Result, set_angular_velocity_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetAngularVelocity"),
            &id_opaque_tuple,
            &velocity_opaque_vector,
        ));
    }

    /// Get the angular velocity of a body.
    pub fn getAngularVelocity(self: *const @This(), id: BodyId) Vector3Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Vector3Result, get_angular_velocity_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetAngularVelocity"),
            &id_opaque_tuple,
        ));
    }

    /// Change the body velocity such that next step, it would reach the target transform.
    /// Dynamic bodies can still be prevented from reaching that transform by collisions and constraints.
    pub fn setTargetQTransform(self: *const @This(), id: BodyId, transform: QTransform) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const transform_1_opaque_array = opacifyVectorElements(3, Float, &transform[0]);
        const transform_2_opaque_array = opacifyVectorElements(4, Float, &transform[1]);

        const transform_1_opaque_vector: []const Opaque = &transform_1_opaque_array;
        const transform_2_opaque_vector: []const Opaque = &transform_2_opaque_array;

        const transform_opaque_tuple: []const Opaque = &.{
            opacify(&transform_1_opaque_vector),
            opacify(&transform_2_opaque_vector),
        };

        return castOpaque(Result, set_target_q_transform_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetTargetQTransform"),
            &id_opaque_tuple,
            &transform_opaque_tuple,
        ));
    }

    /// Apply the impulse `impulse` to the body at the position `location` in world space.
    pub fn applyImpulse(self: *const @This(), id: BodyId, location: Vector3, impulse: Vector3) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const location_opaque_array = opacifyVectorElements(3, Float, &location);
        const location_opaque_vector: []const Opaque = &location_opaque_array;

        const impulse_opaque_array = opacifyVectorElements(3, Float, &impulse);
        const impulse_opaque_vector: []const Opaque = &impulse_opaque_array;

        return castOpaque(Result, apply_impulse_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_ApplyImpulse"),
            &id_opaque_tuple,
            &location_opaque_vector,
            &impulse_opaque_vector,
        ));
    }

    /// Applies the angular impulse `impulse` to the body around its center of mass.
    pub fn applyAngularImpulse(self: *const @This(), id: BodyId, impulse: Vector3) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const impulse_opaque_array = opacifyVectorElements(3, Float, &impulse);
        const impulse_opaque_vector: []const Opaque = &impulse_opaque_array;

        return castOpaque(Result, apply_angular_impulse_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_ApplyAngularImpulse"),
            &id_opaque_tuple,
            &impulse_opaque_vector,
        ));
    }

    /// Try to set the activation state of a body.
    /// NOTE: this seemed to not work.
    pub fn setActivationState(self: *const @This(), id: BodyId, state: ActivationState) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_activation_state_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetActivationState"),
            &id_opaque_tuple,
            &state,
        ));
    }

    /// Get the current activation state of a body.
    pub fn getActivationState(self: *const @This(), id: BodyId) ActivationStateResult {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ActivationStateResult, get_activation_state_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_GetActivationState"),
            &id_opaque_tuple,
        ));
    }

    /// Set the activation behavior of a body. See `ActivationControl` for more details.
    pub fn setActivationControl(self: *const @This(), id: BodyId, control: ActivationControl) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_activation_control_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetActivationControl"),
            &id_opaque_tuple,
            &control,
        ));
    }

    /// Set the activation priority of a body. Defaults to 0.
    /// A body with simulation controlled activation will only be activated by interactions from other bodies
    /// whose priority is >= `priority`
    pub fn setActivationPriority(self: *const @This(), id: BodyId, priority: i7) Result {
        const id_opaque_array = opacifyTupleElements(BodyId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_activation_priority_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Body_SetActivationPriority"),
            &id_opaque_tuple,
            &priority,
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

pub const Constraint = struct {
    physics: *Physics,

    var create_impl = &noopImpl;

    var release_impl = &noopImpl;

    var set_parent_body_impl = &noopImpl;
    var get_parent_body_impl = &noopImpl;

    var set_child_body_impl = &noopImpl;
    var get_child_body_impl = &noopImpl;

    var set_enabled_impl = &noopImpl;
    var get_enabled_impl = &noopImpl;

    var set_collisions_enabled_impl = &noopImpl;
    var get_collisions_enabled_impl = &noopImpl;

    var set_axis_mode_impl = &noopImpl;
    var get_axis_mode_impl = &noopImpl;

    var set_axis_motor_type_impl = &noopImpl;
    var get_axis_motor_type_impl = &noopImpl;

    var get_applied_impulses_impl = &noopImpl;

    var set_anchor_in_parent_impl = &noopImpl;
    var set_anchor_in_child_impl = &noopImpl;

    var get_axis_friction_impl = &noopImpl;
    var get_axis_min_limit_impl = &noopImpl;
    var get_axis_max_limit_impl = &noopImpl;
    var get_axis_motor_target_impl = &noopImpl;
    var get_axis_motor_max_force_impl = &noopImpl;
    var get_axis_motor_position_target_impl = &noopImpl;
    var get_axis_motor_velocity_target_impl = &noopImpl;
    var get_axis_motor_stiffness_impl = &noopImpl;
    var get_axis_motor_damping_impl = &noopImpl;

    var set_axis_friction_impl = &noopImpl;
    var set_axis_min_limit_impl = &noopImpl;
    var set_axis_max_limit_impl = &noopImpl;
    var set_axis_stiffness = &noopImpl;
    var set_axis_damping = &noopImpl;
    var set_axis_motor_target_impl = &noopImpl;
    var set_axis_motor_max_force_impl = &noopImpl;
    var set_axis_motor_position_target_impl = &noopImpl;
    var set_axis_motor_velocity_target_impl = &noopImpl;
    var set_axis_motor_stiffness_impl = &noopImpl;
    var set_axis_motor_damping_impl = &noopImpl;

    /// Allocates a new handle for a constraint object, which limits the relative movement between two bodies.
    pub fn create(self: *const @This()) ConstraintResult {
        return castOpaque(ConstraintResult, create_impl(self.physics, comptime cached_function_indices.getUnoptional("HP_Constraint_Create")));
    }

    /// Release the constraint handle, freeing it's memory if not in use.
    pub fn release(self: *const @This(), id: ConstraintId) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, release_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_Release"),
            &id_opaque_tuple,
        ));
    }

    /// Set the `parent` body of the constraint. Limits are defined with respect to this body.
    pub fn setParentBody(self: *const @This(), id: ConstraintId, body_id: BodyId) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const body_id_opaque_array = opacifyTupleElements(BodyId, &body_id);
        const body_id_opaque_tuple: []const Opaque = &body_id_opaque_array;

        return castOpaque(Result, set_parent_body_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetParentBody"),
            &id_opaque_tuple,
            &body_id_opaque_tuple,
        ));
    }

    /// Get the parent body for a particular constraint.
    pub fn getParentBody(self: *const @This(), id: ConstraintId) BodyResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(BodyResult, get_parent_body_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetParentBody"),
            &id_opaque_tuple,
        ));
    }

    /// Set the "child" body of the constraint. This is the other body which the constraint is attached to.
    pub fn setChildBody(self: *const @This(), id: ConstraintId, body_id: BodyId) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const body_id_opaque_array = opacifyTupleElements(BodyId, &body_id);
        const body_id_opaque_tuple: []const Opaque = &body_id_opaque_array;

        return castOpaque(Result, set_child_body_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetChildBody"),
            &id_opaque_tuple,
            &body_id_opaque_tuple,
        ));
    }

    /// Get the child body for a particular constraint.
    pub fn getChildBody(self: *const @This(), id: ConstraintId) BodyResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(BodyResult, get_child_body_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetChildBody"),
            &id_opaque_tuple,
        ));
    }

    /// Enables a constraint if isEnabled is non-zero. Requires both bodies to be in the same world to have an effect.
    pub fn setEnabled(self: *const @This(), id: ConstraintId, is_enabled: bool) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_enabled_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetEnabled"),
            &id_opaque_tuple,
            &is_enabled,
        ));
    }

    /// Whether the constraint is enabled or not.
    pub fn getEnabled(self: *const @This(), id: ConstraintId) BoolResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(BoolResult, get_enabled_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetEnabled"),
            &id_opaque_tuple,
        ));
    }

    /// By default, a constraint will not allow collisions to occur between the two connected bodies.
    /// A non-zero value for `is_enabled` will change that behaviour.
    pub fn setCollisionsEnabled(self: *const @This(), id: ConstraintId, is_enabled: bool) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_collisions_enabled_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetCollisionsEnabled"),
            &id_opaque_tuple,
            &is_enabled,
        ));
    }

    /// Retrieve whether collisions are enabled for an individual constraint.
    pub fn getCollisionsEnabled(self: *const @This(), id: ConstraintId) BoolResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(BoolResult, get_collisions_enabled_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetCollisionsEnabled"),
            &id_opaque_tuple,
        ));
    }

    /// Set the limit behaviour of the specified axis. See `ConstraintAxisLimitMode` for each effect.
    pub fn setAxisMode(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, limit_mode: ConstraintAxisLimitMode) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_mode_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMode"),
            &id_opaque_tuple,
            &axis,
            &limit_mode,
        ));
    }

    /// Get the limit behaviour for the specified axis.
    pub fn getAxisMode(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) ConstraintAxisLimitResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintAxisLimitResult, get_axis_mode_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMode"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Set the motor type for a constraint axis. Controls how the motor applies forces along the axis.
    pub fn setAxisMotorType(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, motor_type: ConstraintMotorType) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_motor_type_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorType"),
            &id_opaque_tuple,
            &axis,
            &motor_type,
        ));
    }

    /// Get the type of motor used on the specified axis.
    pub fn getAxisMotorType(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) ConstraintMotorTypeResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_motor_type_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorType"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Get the linear and angular impulses applied by a constraint, in world space.
    pub fn getAppliedImpulses(self: *const @This(), id: ConstraintId) Vector3PairResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Vector3PairResult, get_applied_impulses_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAppliedImpulses"),
            &id_opaque_tuple,
        ));
    }

    /// Configure the constraint space of the parent body (in parent body space).
    /// A third basis vector is calculated perpendicular to `axis_*`.
    /// You must also configure the anchor in child space.
    pub fn setAnchorInParent(
        self: *const @This(),
        id: ConstraintId,
        /// Origin of constraint space (in parent body space).
        pivot: Vector3,
        /// Basis vector for constraint space X (in parent body space).
        axis_x: Vector3,
        /// Basis vector for constraint space Y (in parent body space).
        axis_y: Vector3,
    ) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const pivot_opaque_array = opacifyVectorElements(3, Float, &pivot);
        const pivot_opaque_vector: []const Opaque = &pivot_opaque_array;

        const axis_x_opaque_array = opacifyVectorElements(3, Float, &axis_x);
        const axis_x_opaque_vector: []const Opaque = &axis_x_opaque_array;

        const axis_y_opaque_array = opacifyVectorElements(3, Float, &axis_y);
        const axis_y_opaque_vector: []const Opaque = &axis_y_opaque_array;

        return castOpaque(Result, set_anchor_in_parent_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAnchorInParent"),
            &id_opaque_tuple,
            &pivot_opaque_vector,
            &axis_x_opaque_vector,
            &axis_y_opaque_vector,
        ));
    }

    /// Configure the constraint space of the child body (in parent body space).
    /// A third basis vector is calculated perpendicular to `axis_*`.
    /// You must also configure the anchor in parent space.
    pub fn setAnchorInChild(
        self: *const @This(),
        id: ConstraintId,
        /// Origin of constraint space (in child body space).
        pivot: Vector3,
        /// Basis vector for constraint space X (in child body space).
        axis_x: Vector3,
        /// Basis vector for constraint space Y (in child body space).
        axis_y: Vector3,
    ) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const pivot_opaque_array = opacifyVectorElements(3, Float, &pivot);
        const pivot_opaque_vector: []const Opaque = &pivot_opaque_array;

        const axis_x_opaque_array = opacifyVectorElements(3, Float, &axis_x);
        const axis_x_opaque_vector: []const Opaque = &axis_x_opaque_array;

        const axis_y_opaque_array = opacifyVectorElements(3, Float, &axis_y);
        const axis_y_opaque_vector: []const Opaque = &axis_y_opaque_array;

        return castOpaque(Result, set_anchor_in_child_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAnchorInChild"),
            &id_opaque_tuple,
            &pivot_opaque_vector,
            &axis_x_opaque_vector,
            &axis_y_opaque_vector,
        ));
    }

    /// Get the friction coefficient associated with a particular axis for a constraint.
    pub fn getAxisFriction(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_friction_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisFriction"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Retrieve the minumum signed movement along the specified axis.
    pub fn getAxisMinLimit(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_min_limit_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMinLimit"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Retrieve the maximum signed movement along the specified axis.
    pub fn getAxisMaxLimit(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_max_limit_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMaxLimit"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Get the target for the constraint motor on the specified axis.
    /// Consider using the more explicit position/velocity target functions.
    pub fn getAxisMotorTarget(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_motor_target_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorTarget"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Get the max force (or torque) for a constraint motor on a particular axis.
    pub fn getAxisMotorMaxForce(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_motor_max_force_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorMaxForce"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Get the position target for a constraint motor on a particular axis.
    pub fn getAxisMotorPositionTarget(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_motor_position_target_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorPositionTarget"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Get the velocity target for a constraint motor on a particular axis.
    pub fn getAxisMotorVelocityTarget(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_motor_velocity_target_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorVelocityTarget"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Get the position target stiffness of a `spring` type motor.
    pub fn getAxisMotorStiffness(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_motor_stiffness_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorStiffness"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Get the velocity target damping of a `spring` type motor.
    pub fn getAxisMotorDamping(self: *const @This(), id: ConstraintId, axis: ConstraintAxis) FloatResult {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ConstraintMotorTypeResult, get_axis_motor_damping_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_GetAxisMotorDamping"),
            &id_opaque_tuple,
            &axis,
        ));
    }

    /// Adds a friction coefficient which resists movement along the specified axis.
    pub fn setAxisFriction(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, friction: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_friction_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisFriction"),
            &id_opaque_tuple,
            &axis,
            &friction,
        ));
    }

    /// Set the minimum allowed signed movement along the specified axis.
    /// For `linear_*` axes, this value is in meters; for angular axes, this is in radians.
    /// Only has an effect when ConstraintAxisLimitMode is `limited`.
    pub fn setAxisMinLimit(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, min_limit: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_min_limit_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMinLimit"),
            &id_opaque_tuple,
            &axis,
            &min_limit,
        ));
    }

    /// Set the maximum allowed signed movement along the specified axis.
    /// For `linear_*` axes, this value is in meters; for angular axes, this is in radians.
    /// Only has an effect when ConstraintAxisLimitMode is `limited`.
    pub fn setAxisMaxLimit(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, max_limit: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_max_limit_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMaxLimit"),
            &id_opaque_tuple,
            &axis,
            &max_limit,
        ));
    }

    /// Sets the stiffness of a constraint axis. This will convert the axis from a hard limit to one which
    /// uses a `spring` model, parameterized on stiffness and damping.
    pub fn setAxisStiffness(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, stiffness: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_stiffness(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisStiffness"),
            &id_opaque_tuple,
            &axis,
            &stiffness,
        ));
    }

    /// Sets the damping of a constraint axis. This will convert the axis from a hard limit to one which
    /// uses a `spring` model, parameterized on stiffness and damping.
    pub fn setAxisDamping(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, damping: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_damping(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisDamping"),
            &id_opaque_tuple,
            &axis,
            &damping,
        ));
    }

    /// Set the target for the constraint motor on the specified axis. The precise meaning of target depends on the type of the constraint motor.
    /// Consider using the more explicit position/velocity target functions.
    pub fn setAxisMotorTarget(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, target: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_motor_target_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorTarget"),
            &id_opaque_tuple,
            &axis,
            &target,
        ));
    }

    /// Set the max force for the constraint motor on the specified axis.
    /// For angular axes, `max_force` is used as a max torque.
    pub fn setAxisMotorMaxForce(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, max_force: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_motor_max_force_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorMaxForce"),
            &id_opaque_tuple,
            &axis,
            &max_force,
        ));
    }

    /// Set the position target for a constraint motor on a particular axis.
    pub fn setAxisMotorPositionTarget(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, target: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_motor_position_target_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorPositionTarget"),
            &id_opaque_tuple,
            &axis,
            &target,
        ));
    }

    /// Set the velocity target for a constraint motor on a particular axis.
    pub fn setAxisMotorVelocityTarget(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, target: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_motor_velocity_target_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorVelocityTarget"),
            &id_opaque_tuple,
            &axis,
            &target,
        ));
    }

    /// Set the position target stiffness of a `spring` type motor.
    pub fn setAxisMotorStiffness(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, stiffness: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_motor_stiffness_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorStiffness"),
            &id_opaque_tuple,
            &axis,
            &stiffness,
        ));
    }

    /// Set the velocity target damping of a `spring` type motor.
    pub fn setAxisMotorDamping(self: *const @This(), id: ConstraintId, axis: ConstraintAxis, stiffness: Float) Result {
        const id_opaque_array = opacifyTupleElements(ConstraintId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_axis_motor_damping_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Constraint_SetAxisMotorDamping"),
            &id_opaque_tuple,
            &axis,
            &stiffness,
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

pub const DebugGeometry = struct {
    physics: *Physics,

    var create_impl = &noopImpl;

    var get_info_impl = &noopImpl;

    var release_impl = &noopImpl;

    /// Generates a visualization of a shape's geometry, suitable for debugging.
    pub fn create(self: *const @This(), shape_id: ShapeId) DebugGeometryResult {
        const shape_id_opaque_array = opacifyTupleElements(ShapeId, &shape_id);
        const shape_id_opaque_tuple: []const Opaque = &shape_id_opaque_array;

        return castOpaque(DebugGeometryResult, create_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_Shape_CreateDebugDisplayGeometry")),
            &shape_id_opaque_tuple,
        ));
    }

    /// Retrieves the vertex and triangle information for a debug geometry id.
    pub fn getInfo(self: *const @This(), id: DebugGeometryId) DebugGeometryInfoResult {
        const id_opaque_array = opacifyTupleElements(DebugGeometryId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(DebugGeometryInfoResult, get_info_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_DebugGeometry_GetInfo")),
            &id_opaque_tuple,
        ));
    }

    /// Release the reference to the debug geometry, freeing memory.
    pub fn release(self: *const @This(), id: DebugGeometryId) Result {
        const id_opaque_array = opacifyTupleElements(DebugGeometryId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, release_impl(
            self.physics,
            comptime @intCast(cached_function_indices.getUnoptional("HP_DebugGeometry_Release")),
            &id_opaque_tuple,
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

pub const World = struct {
    physics: *Physics,

    var create_impl = &noopImpl;

    var release_impl = &noopImpl;

    var get_body_buffer_impl = &noopImpl;

    var set_gravity_impl = &noopImpl;

    var add_body_impl = &noopImpl;
    var remove_body_impl = &noopImpl;

    var get_num_bodies_impl = &noopImpl;

    var cast_ray_impl = &noopImpl;

    var cast_ray_with_collector_impl = &noopImpl;

    var point_proximity_with_collector_impl = &noopImpl;
    var shape_proximity_with_collector_impl = &noopImpl;

    var shape_cast_with_collector_impl = &noopImpl;

    var step_impl = &noopImpl;
    var set_ideal_step_time_impl = &noopImpl;

    var set_speed_limit_impl = &noopImpl;
    var get_speed_limit_impl = &noopImpl;

    var get_collision_events_impl = &noopImpl;
    var get_next_collision_event_impl = &noopImpl;

    var get_trigger_events_impl = &noopImpl;
    var get_next_trigger_event_impl = &noopImpl;

    /// Allocate a new handle for a world, which is the basis of a simulation.
    pub fn create(self: *const @This()) WorldResult {
        return castOpaque(WorldResult, create_impl(self.physics, comptime cached_function_indices.getUnoptional("HP_World_Create")));
    }

    /// Releases a world handle, freeing any memory used.
    pub fn release(self: *const @This(), id: WorldId) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, release_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_Release"),
            &id_opaque_tuple,
        ));
    }

    /// Returns the address of the world's body buffer, for use with
    /// `Body.getWorldTransformOffset`. This result can be invalidated if a
    /// body is added to the world.
    pub fn getBodyBuffer(self: *const @This(), id: WorldId) InternalHandleResult {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(InternalHandleResult, get_body_buffer_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_GetBodyBuffer"),
            &id_opaque_tuple,
        ));
    }

    /// Set the global acceleration due to gravity of a world. This is applied to all `dynamic` bodies each step.
    pub fn setGravity(self: *const @This(), id: WorldId, gravity: Vector3) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const gravity_opaque_array = opacifyVectorElements(3, Float, &gravity);
        const gravity_opaque_vector: []const Opaque = &gravity_opaque_array;

        return castOpaque(Result, set_gravity_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_SetGravity"),
            &id_opaque_tuple,
            &gravity_opaque_vector,
        ));
    }

    // pub fn getGravity(self: *const @This(), a: u32, b: u32) !u32 {
    //     return self.physics.callExported("HP_World_GetGravity", .{ a, b });
    // }

    /// Adds a body to the world, where it will partake in the simulation in the next step. A body can only be in a single world at a time.
    pub fn addBody(self: *const @This(), id: WorldId, body_id: BodyId, start_asleep: bool) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const body_id_opaque_array = opacifyTupleElements(BodyId, &body_id);
        const body_id_opaque_tuple: []const Opaque = &body_id_opaque_array;

        return castOpaque(Result, add_body_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_AddBody"),
            &id_opaque_tuple,
            &body_id_opaque_tuple,
            &start_asleep,
        ));
    }

    /// Remove a body from the world.
    pub fn removeBody(self: *const @This(), id: WorldId, body_id: BodyId) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const body_id_opaque_array = opacifyTupleElements(BodyId, &body_id);
        const body_id_opaque_tuple: []const Opaque = &body_id_opaque_array;

        return castOpaque(Result, remove_body_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_RemoveBody"),
            &id_opaque_tuple,
            &body_id_opaque_tuple,
        ));
    }

    /// Return the number of bodies added to the world.
    pub fn getNumBodies(self: *const @This(), id: WorldId) IntResult {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(IntResult, get_num_bodies_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_GetNumBodies"),
            &id_opaque_tuple,
        ));
    }

    /// Perform the raycast described by `query` against the bodies added to the world, storing the results in the collector.
    /// Collector will be cleared of any previous results.
    pub fn castRayWithCollector(self: *const @This(), id: WorldId, collector_id: CollectorId, query: RayCastInput) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const collector_id_opaque_array = opacifyTupleElements(CollectorId, &collector_id);
        const collector_id_opaque_tuple: []const Opaque = &collector_id_opaque_array;

        const query_1_opaque_array = opacifyVectorElements(3, Float, &query[0]);
        const query_2_opaque_array = opacifyVectorElements(3, Float, &query[1]);
        const query_3_opaque_array = opacifyTupleElements(FilterInfo, &query[2]);
        const query_5_opaque_array = opacifyTupleElements(BodyId, &query[4]);

        const query_1_opaque_vector: []const Opaque = &query_1_opaque_array;
        const query_2_opaque_vector: []const Opaque = &query_2_opaque_array;
        const query_3_opaque_vector: []const Opaque = &query_3_opaque_array;
        const query_5_opaque_vector: []const Opaque = &query_5_opaque_array;

        const query_opaque_tuple: []const Opaque = &.{
            opacify(&query_1_opaque_vector),
            opacify(&query_2_opaque_vector),
            opacify(&query_3_opaque_vector),
            opacify(&query[3]),
            opacify(&query_5_opaque_vector),
        };

        return castOpaque(Result, cast_ray_with_collector_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_CastRayWithCollector"),
            &id_opaque_tuple,
            &collector_id_opaque_tuple,
            &query_opaque_tuple,
        ));
    }

    /// Perform the point proximity described by `query` against the bodies added to the world, storing the results in the collector.
    /// Collector will be cleared of any previous results.
    pub fn pointProximityWithCollector(self: *const @This(), id: WorldId, collector_id: CollectorId, query: PointProximityInput) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const collector_id_opaque_array = opacifyTupleElements(CollectorId, &collector_id);
        const collector_id_opaque_tuple: []const Opaque = &collector_id_opaque_array;

        const query_1_opaque_array = opacifyVectorElements(3, Float, &query[0]);
        const query_3_opaque_array = opacifyTupleElements(FilterInfo, &query[2]);
        const query_5_opaque_array = opacifyTupleElements(BodyId, &query[4]);

        const query_1_opaque_vector: []const Opaque = &query_1_opaque_array;
        const query_3_opaque_vector: []const Opaque = &query_3_opaque_array;
        const query_5_opaque_vector: []const Opaque = &query_5_opaque_array;

        const query_opaque_tuple: []const Opaque = &.{
            opacify(&query_1_opaque_vector),
            opacify(&query[1]),
            opacify(&query_3_opaque_vector),
            opacify(&query[3]),
            opacify(&query_5_opaque_vector),
        };

        return castOpaque(Result, point_proximity_with_collector_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_PointProximityWithCollector"),
            &id_opaque_tuple,
            &collector_id_opaque_tuple,
            &query_opaque_tuple,
        ));
    }

    /// Perform the shape proximity described by `query` against the bodies added to the world, storing the results in the collector.
    /// Collector will be cleared of any previous results.
    pub fn shapeProximityWithCollector(self: *const @This(), id: WorldId, collector_id: CollectorId, query: ShapeProximityInput) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const collector_id_opaque_array = opacifyTupleElements(CollectorId, &collector_id);
        const collector_id_opaque_tuple: []const Opaque = &collector_id_opaque_array;

        const query_1_opaque_array = opacifyTupleElements(ShapeId, &query[0]);
        const query_2_opaque_array = opacifyVectorElements(3, Float, &query[1]);
        const query_3_opaque_array = opacifyVectorElements(4, Float, &query[2]);
        const query_6_opaque_array = opacifyTupleElements(BodyId, &query[5]);

        const query_1_opaque_vector: []const Opaque = &query_1_opaque_array;
        const query_2_opaque_vector: []const Opaque = &query_2_opaque_array;
        const query_3_opaque_vector: []const Opaque = &query_3_opaque_array;
        const query_6_opaque_vector: []const Opaque = &query_6_opaque_array;

        const query_opaque_tuple: []const Opaque = &.{
            opacify(&query_1_opaque_vector),
            opacify(&query_2_opaque_vector),
            opacify(&query_3_opaque_vector),
            opacify(&query[3]),
            opacify(&query[4]),
            opacify(&query_6_opaque_vector),
        };

        return castOpaque(Result, shape_proximity_with_collector_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_ShapeProximityWithCollector"),
            &id_opaque_tuple,
            &collector_id_opaque_tuple,
            &query_opaque_tuple,
        ));
    }

    /// Perform a shape cast, as described by `query` against the bodies added to the world, storing the results in the collector.
    /// Collector will be cleared of any previous results.
    pub fn shapeCastWithCollector(self: *const @This(), id: WorldId, collector_id: CollectorId, query: ShapeCastInput) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        const collector_id_opaque_array = opacifyTupleElements(CollectorId, &collector_id);
        const collector_id_opaque_tuple: []const Opaque = &collector_id_opaque_array;

        const query_1_opaque_array = opacifyTupleElements(ShapeId, &query[0]);
        const query_2_opaque_array = opacifyVectorElements(4, Float, &query[1]);
        const query_3_opaque_array = opacifyVectorElements(3, Float, &query[2]);
        const query_4_opaque_array = opacifyVectorElements(3, Float, &query[3]);
        const query_6_opaque_array = opacifyTupleElements(BodyId, &query[5]);

        const query_1_opaque_vector: []const Opaque = &query_1_opaque_array;
        const query_2_opaque_vector: []const Opaque = &query_2_opaque_array;
        const query_3_opaque_vector: []const Opaque = &query_3_opaque_array;
        const query_4_opaque_vector: []const Opaque = &query_4_opaque_array;
        const query_6_opaque_vector: []const Opaque = &query_6_opaque_array;

        const query_opaque_tuple: []const Opaque = &.{
            opacify(&query_1_opaque_vector),
            opacify(&query_2_opaque_vector),
            opacify(&query_3_opaque_vector),
            opacify(&query_4_opaque_vector),
            opacify(&query[4]),
            opacify(&query_6_opaque_vector),
        };

        return castOpaque(Result, shape_cast_with_collector_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_ShapeCastWithCollector"),
            &id_opaque_tuple,
            &collector_id_opaque_tuple,
            &query_opaque_tuple,
        ));
    }

    /// Simulate the world and advance time by `timestep` seconds.
    pub fn step(self: *const @This(), id: WorldId, timestep: Float) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, step_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_Step"),
            &id_opaque_tuple,
            &timestep,
        ));
    }

    /// Configure the ideal delta time which you intend to call `world.step`. Defaults to 1/60.
    /// If the delta time passed to the world step differs from this amount, the solver parameters
    /// will be automatically adjusted, to attempt to maintain a similar effective solver stiffness.
    /// To disable this behaviour, set this value to zero.
    pub fn setIdealStepTime(self: *const @This(), id: WorldId, delta_time: Float) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_ideal_step_time_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_SetIdealStepTime"),
            &id_opaque_tuple,
            &delta_time,
        ));
    }

    /// Configure the maximum speed an individual body may have.
    pub fn setSpeedLimit(self: *const @This(), id: WorldId, max_linear_velocity: Float, max_angular_velocity: Float) Result {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, set_speed_limit_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_SetSpeedLimit"),
            &id_opaque_tuple,
            &max_linear_velocity,
            &max_angular_velocity,
        ));
    }

    /// Retrieve the maximum speed an individual body may have.
    pub fn getSpeedLimit(self: *const @This(), id: WorldId) FloatPairResult {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(FloatPairResult, get_speed_limit_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_GetSpeedLimit"),
            &id_opaque_tuple,
        ));
    }

    /// Get the next collision event, following `previous_event`.
    pub fn getNextCollisionEvent(self: *const @This(), world: u32, previous_event: u32) u32 {
        return castOpaque(u32, get_next_collision_event_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_GetNextCollisionEvent"),
            &world,
            &previous_event,
        ));
    }

    /// Get the first trigger event generated by the previous world step.
    pub fn getNextTriggerEvent(self: *const @This(), world: u32, previous_event: u32) u32 {
        return castOpaque(u32, get_next_trigger_event_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_GetNextTriggerEvent"),
            &world,
            &previous_event,
        ));
    }

    /// Get the first collision event generated by the previous world step.
    pub fn getCollisionEvents(self: *const @This(), id: WorldId) InternalHandleResult {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(InternalHandleResult, get_collision_events_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_GetCollisionEvents"),
            &id_opaque_tuple,
        ));
    }

    /// Get the first trigger event generated by the previous world step.
    pub fn getTriggerEvents(self: *const @This(), id: WorldId) InternalHandleResult {
        const id_opaque_array = opacifyTupleElements(WorldId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(InternalHandleResult, get_trigger_events_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_GetTriggerEvents"),
            &id_opaque_tuple,
        ));
    }

    /// Advanced use only. Perform a raycast using structures preallocated in the WASM memory.
    /// `world` should be the address of an world, `query` should be the address of a RayCastInput, and
    /// `result` should be the address of a buffer of `max_results` RaycastResult. Returns the number of hits.
    pub fn castRay(self: *const @This(), world: u32, query: u32, result: u32, max_results: i32) i32 {
        return castOpaque(i32, cast_ray_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_World_CastRay"),
            &world,
            &query,
            &result,
            &max_results,
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

pub const Event = struct {
    physics: *Physics,

    var as_trigger = &noopImpl;

    var as_collision = &noopImpl;

    /// Get the first trigger event generated by the previous world step.
    pub fn asTrigger(self: *const @This(), id: u32) TriggerEventResult {
        return castOpaque(TriggerEventResult, as_trigger(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Event_AsTrigger"),
            &id,
        ));
    }

    /// Convert a collision event id to a concrete type.
    pub fn asCollision(self: *const @This(), id: u32) CollisionEventResult {
        return castOpaque(CollisionEventResult, as_collision(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Event_AsCollision"),
            &id,
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

pub const QueryCollector = struct {
    physics: *Physics,

    var create_impl = &noopImpl;

    var release_impl = &noopImpl;

    var get_num_hits_impl = &noopImpl;

    var get_cast_ray_result_impl = &noopImpl;

    var get_point_proximity_result_impl = &noopImpl;
    var get_shape_proximity_result_impl = &noopImpl;

    var get_shape_cast_result_impl = &noopImpl;

    /// Allocates a query collector with sufficient capacity to store the requested number of hits.
    pub fn create(self: *const @This(), hit_capacity: i32) CollectorResult {
        return castOpaque(CollectorResult, create_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_QueryCollector_Create"),
            &hit_capacity,
        ));
    }

    /// Releases a query collector handle, returning the memory to the plugin.
    pub fn release(self: *const @This(), id: CollectorId) Result {
        const id_opaque_array = opacifyTupleElements(CollectorId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(Result, release_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_QueryCollector_Release"),
            &id_opaque_tuple,
        ));
    }

    /// Get the number of hits currently stored in the collector.
    pub fn getNumHits(self: *const @This(), id: CollectorId) IntResult {
        const id_opaque_array = opacifyTupleElements(CollectorId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(IntResult, get_num_hits_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_QueryCollector_GetNumHits"),
            &id_opaque_tuple,
        ));
    }

    /// Get the raycast result stored at `hit_index` in the collector.
    /// Only valid if the last query performed with this collector was a raycast.
    pub fn getCastRayResult(self: *const @This(), id: CollectorId, hit_index: i32) RayCastResultResult {
        const id_opaque_array = opacifyTupleElements(CollectorId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(RayCastResultResult, get_cast_ray_result_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_QueryCollector_GetCastRayResult"),
            &id_opaque_tuple,
            &hit_index,
        ));
    }

    /// Get the proximity result stored at `hit_index` in the collector.
    /// Only valid if the last query performed with this collector was a point proximity.
    pub fn getPointProximityResult(self: *const @This(), id: CollectorId, hit_index: i32) PointProximityResultResult {
        const id_opaque_array = opacifyTupleElements(CollectorId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(PointProximityResultResult, get_point_proximity_result_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_QueryCollector_GetPointProximityResult"),
            &id_opaque_tuple,
            &hit_index,
        ));
    }

    /// Get the proximity result stored at `hit_index` in the collector.
    /// Only valid if the last query performed with this collector was a point proximity.
    pub fn getShapeProximityResult(self: *const @This(), id: CollectorId, hit_index: i32) ShapeProximityResultResult {
        const id_opaque_array = opacifyTupleElements(CollectorId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ShapeProximityResultResult, get_shape_proximity_result_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_QueryCollector_GetShapeProximityResult"),
            &id_opaque_tuple,
            &hit_index,
        ));
    }

    /// Get the shape cast result stored at `hit_index` in the collector.
    /// Only valid if the last query performed with this collector was a point proximity or a shape proximity test.
    pub fn getShapeCastResult(self: *const @This(), id: CollectorId, hit_index: i32) ShapeCastResultResult {
        const id_opaque_array = opacifyTupleElements(CollectorId, &id);
        const id_opaque_tuple: []const Opaque = &id_opaque_array;

        return castOpaque(ShapeCastResultResult, get_shape_cast_result_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_QueryCollector_GetShapeCastResult"),
            &id_opaque_tuple,
            &hit_index,
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

pub const Debug = struct {
    physics: *Physics,

    var start_recording_stats_impl = &noopImpl;
    var stop_recording_stats_impl = &noopImpl;

    /// Start recording performance counters for a single world. Only available in development builds.
    pub fn startRecordingStats(self: *const @This(), world_id: WorldId) Result {
        const world_id_opaque_array = opacifyTupleElements(WorldId, &world_id);
        const world_id_opaque_tuple: []const Opaque = &world_id_opaque_array;

        return castOpaque(Result, start_recording_stats_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Debug_StartRecordingStats"),
            &world_id_opaque_tuple,
        ));
    }

    /// Stop recording performance counters for a world. Only available in development builds.
    /// This will call a method 'timerData' on the callback object, which can
    /// retrieve statistics formatted in XML, for use with hkMonitor.
    pub fn stopRecordingStats(self: *const @This()) Result {
        // TODO: implement this, this is using emval and hard to implement

        return castOpaque(Result, stop_recording_stats_impl(
            self.physics,
            comptime cached_function_indices.getUnoptional("HP_Debug_StopRecordingStats"),
        ));
    }

    const TypeInstance = Emscripten.Bind.Type.Instance;

    const Opaque = TypeInstance.Opaque;

    const castOpaque = TypeInstance.castOpaque;
    const opacify = TypeInstance.opacify;
};

shape: Shape,
body: Body,
constraint: Constraint,
debug_geometry: DebugGeometry,
world: World,
event: Event,
query_collector: QueryCollector,
debug: Debug,

allocator: mem.Allocator,

aot_buf: []align(16) u8,

heap_buf: []align(16) u8,

module: wamr.wasm_module_t = null,
module_inst: wamr.wasm_module_inst_t = null,
exec_env: wamr.wasm_exec_env_t = null,

table_inst: wamr.wasm_table_inst_t = .{},

cached_functions: [cached_function_indices.kvs.len]wamr.wasm_function_inst_t = @splat(null),
cached_indirect_functions: std.AutoHashMap(u32, wamr.wasm_function_inst_t),

embind_arena: heap.ArenaAllocator,
/// Allocator especially for embind. Its custom implementation is `embind_arena`.
embind_allocator: mem.Allocator,

embind_type_registry: Emscripten.Bind.Type.Registry,

embind_tuple_registry: Emscripten.Bind.TupleRegistry,

embind_awaiting_dependencies: Emscripten.Bind.AwaitingDependencies,

embind_invoker_contexts: [cached_function_indices.kvs.len]?*Emscripten.Bind.InvokerContext = @splat(null),

embind_temp_arena: heap.ArenaAllocator,
/// Temp arena allocator for embind. Mainly used in `fromWire` of type, it must be freed instantly.
/// Its custom implementation is `embind_temp_arena`.
embind_temp_allocator: mem.Allocator,

const aot_buf_raw = @embedFile("binary/x86_64/HavokPhysics.aot");

const stack_size: u32 = 64 * 1024;

const heap_size: u32 = 1073741824;

fn abort_js(_: wamr.wasm_exec_env_t) callconv(.c) void {
    @panic("Aborted()");
}

fn emscripten_get_heap_max(_: wamr.wasm_exec_env_t) callconv(.c) i32 {
    return heap_size;
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

fn readLatin1String(physics: *Physics, ptr: u32) ![]u8 {
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
    self: *Physics,
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
    self: *Physics,
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

            const waiter = try allocator.create(Emscripten.Bind.Dependency);
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

fn getPhysics(exec_env: wamr.wasm_exec_env_t) ?*Physics {
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

            .physics = physics,
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
    _: i32,
    _: i32,
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

            .physics = physics,
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

            .physics = physics,

            .size = @intCast(size),

            .is_signed = min_range < 0,
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

            .physics = physics,

            .size = @intCast(size),
        };

        physics.registerType(type_id, instance) catch {
            allocator.free(name);

            allocator.destroy(instance);
        };
    }
}

fn embind_register_std_string(_: wamr.wasm_exec_env_t, _: Emscripten.Bind.Type.Id, _: i32) callconv(.c) void {}

fn embind_register_std_wstring(_: wamr.wasm_exec_env_t, _: Emscripten.Bind.Type.Id, _: i32, _: i32) callconv(.c) void {}

fn embind_register_emval(_: wamr.wasm_exec_env_t, _: Emscripten.Bind.Type.Id) callconv(.c) void {}

fn embind_register_memory_view(_: wamr.wasm_exec_env_t, _: Emscripten.Bind.Type.Id, _: i32, _: i32) callconv(.c) void {}

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

fn createMethodImplInner(signature: MethodSignature) MethodImpl {
    return switch (signature) {
        .ftf => @ptrCast(&struct {
            fn impl(physics: *Physics, context_index: u8) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const return_wire = physics.call(context.invoker, &.{context.function_wire}) catch unreachable;

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftfn => @ptrCast(&struct {
            fn impl(physics: *Physics, context_index: u8, arg_0: Emscripten.Bind.Type.Instance.Opaque) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired }) catch unreachable;

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .fffn => @ptrCast(&struct { // We here want to constantly change Return to void, but it may be broken in later changes
            fn impl(physics: *Physics, context_index: u8, arg_0: Emscripten.Bind.Type.Instance.Opaque) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);

                _ = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired }) catch unreachable;

                return @ptrFromInt(0); // Undefined
            }
        }.impl),
        .ftfnnnn => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
                arg_2: Emscripten.Bind.Type.Instance.Opaque,
                arg_3: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2, false);
                const arg_3_wired = context.arg_type_instances[3].toWire(arg_3, false);

                const return_wire = physics.call(context.invoker, &.{
                    context.function_wire,
                    arg_0_wired,
                    arg_1_wired,
                    arg_2_wired,
                    arg_3_wired,
                }) catch unreachable;

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftfnn => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired, arg_1_wired }) catch unreachable;

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftftt => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired, arg_1_wired }) catch unreachable;

                // { // Destruct arg_0_wired
                //     const destructor = context.destructors[0];

                //     switch (destructor) {
                //         .native => |destructor_inner| destructor_inner(physics, arg_0_wired.u32ize()),
                //         .wasm => |destructor_inner| _ = physics.callSimple(destructor_inner, .{arg_0_wired.u32ize()}) catch unreachable,
                //     }
                // }

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[0], .{arg_0_wired.simplify()}) catch unreachable;
                    _ = physics.callSimple(context.destructors[1], .{arg_1_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftft => @ptrCast(&struct {
            fn impl(physics: *Physics, context_index: u8, arg_0: Emscripten.Bind.Type.Instance.Opaque) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired }) catch unreachable;

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[0], .{arg_0_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftftn => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired, arg_1_wired }) catch unreachable;

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[0], .{arg_0_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftftnn => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
                arg_2: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired, arg_1_wired, arg_2_wired }) catch unreachable;

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[0], .{arg_0_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftfttn => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
                arg_2: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired, arg_1_wired, arg_2_wired }) catch unreachable;

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[0], .{arg_0_wired.simplify()}) catch unreachable;
                    _ = physics.callSimple(context.destructors[1], .{arg_1_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftfttt => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
                arg_2: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2, false);

                const return_wire = physics.call(context.invoker, &.{ context.function_wire, arg_0_wired, arg_1_wired, arg_2_wired }) catch unreachable;

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[0], .{arg_0_wired.simplify()}) catch unreachable;
                    _ = physics.callSimple(context.destructors[1], .{arg_1_wired.simplify()}) catch unreachable;
                    _ = physics.callSimple(context.destructors[2], .{arg_2_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftfnntn => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
                arg_2: Emscripten.Bind.Type.Instance.Opaque,
                arg_3: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2, false);
                const arg_3_wired = context.arg_type_instances[3].toWire(arg_3, false);

                const return_wire = physics.call(context.invoker, &.{
                    context.function_wire,
                    arg_0_wired,
                    arg_1_wired,
                    arg_2_wired,
                    arg_3_wired,
                }) catch unreachable;

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[2], .{arg_2_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
        .ftftttt => @ptrCast(&struct {
            fn impl(
                physics: *Physics,
                context_index: u8,
                arg_0: Emscripten.Bind.Type.Instance.Opaque,
                arg_1: Emscripten.Bind.Type.Instance.Opaque,
                arg_2: Emscripten.Bind.Type.Instance.Opaque,
                arg_3: Emscripten.Bind.Type.Instance.Opaque,
            ) callconv(.c) Emscripten.Bind.Type.Instance.Opaque {
                const context = physics.embind_invoker_contexts[context_index] orelse unreachable;

                const arg_0_wired = context.arg_type_instances[0].toWire(arg_0, false);
                const arg_1_wired = context.arg_type_instances[1].toWire(arg_1, false);
                const arg_2_wired = context.arg_type_instances[2].toWire(arg_2, false);
                const arg_3_wired = context.arg_type_instances[3].toWire(arg_3, false);

                const return_wire = physics.call(context.invoker, &.{
                    context.function_wire,
                    arg_0_wired,
                    arg_1_wired,
                    arg_2_wired,
                    arg_3_wired,
                }) catch unreachable;

                { // Destruct wired arguments
                    _ = physics.callSimple(context.destructors[0], .{arg_0_wired.simplify()}) catch unreachable;
                    _ = physics.callSimple(context.destructors[1], .{arg_1_wired.simplify()}) catch unreachable;
                    _ = physics.callSimple(context.destructors[2], .{arg_2_wired.simplify()}) catch unreachable;
                    _ = physics.callSimple(context.destructors[3], .{arg_3_wired.simplify()}) catch unreachable;
                }

                return context.return_type_instance.fromWire(.{ .value = return_wire });
            }
        }.impl),
    };
}

fn createMethodImpl(
    self: *Physics,
    /// Must not be deinited after this function called.
    /// Parent's array list will be deinited when arena's deinit is called.
    type_instances: []?*const Emscripten.Bind.Type.Instance,
    invoker: wamr.wasm_function_inst_t,
    function_index: usize,
    function: u32,
    is_async: bool,
) !MethodImpl {
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

        .function_wire = .{ .value = function },

        .return_type_instance = type_instances[0] orelse return error.MissingReturnType,
    };

    var destructors_index: usize = 0;

    for (type_instances[2..], 0..) |item, i|
        if (item) |type_instance| {
            context.arg_type_instances[i] = type_instance;

            if (type_instance.destructor) |destructor| {
                context.destructors[destructors_index] = destructor;

                destructors_index += 1;
            }
        } else return error.MissingArgumentType;

    // Register context
    self.embind_invoker_contexts[function_index] = context;

    return createMethodImplInner(try createMethodSignature(type_instances, returns, is_async));
}

fn embind_register_function(
    exec_env: wamr.wasm_exec_env_t,
    name_ptr: i32,
    dependent_type_ids_count: i32,
    dependent_type_ids_ptr: i32,
    _: i32,
    invoker_index: i32,
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
            .invoker = physics.getIndirectFunction(@intCast(invoker_index)) catch {
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
                    physics_inner: *Physics,
                    context: *anyopaque,
                    converters: Emscripten.Bind.Type.Converters,
                ) Emscripten.Bind.Type.Converters {
                    const allocator_inner = physics_inner.embind_allocator;

                    const register_context_inner: *RegisterContext = @ptrCast(@alignCast(context));
                    defer {
                        allocator_inner.free(register_context_inner.name);

                        allocator_inner.destroy(register_context_inner);
                    }

                    // Freed after arena deinited
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

                    const function_index = cached_function_indices.getRuntime(register_context_inner.name) orelse {
                        invoker_type_instances.deinit();

                        return &.{};
                    };

                    replaceMethodImpl(function_index, physics_inner.createMethodImpl(
                        invoker_type_instances.items,
                        register_context_inner.invoker,
                        function_index,
                        register_context_inner.function,
                        register_context_inner.is_async,
                    ) catch {
                        invoker_type_instances.deinit();

                        return &.{};
                    });

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
    constructor_index: i32,
    _: i32,
    destructor_index: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        physics.embind_tuple_registry.put(type_id, .{
            .name = name,

            .constructor = physics.getIndirectFunction(@intCast(constructor_index)) catch {
                allocator.free(name);

                return;
            },
            .destructor = physics.getIndirectFunction(@intCast(destructor_index)) catch {
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
    getter_index: i32,
    getter_context: i32,
    setter_arg_type: i32,
    _: i32,
    setter_index: i32,
    setter_context: i32,
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics|
        if (physics.embind_tuple_registry.getPtr(type_id)) |tuple|
            tuple.elements.append(.{
                .getter_return_type = getter_return_type,
                .getter = physics.getIndirectFunction(@intCast(getter_index)) catch return,
                .getter_context = @intCast(getter_context),

                .setter_arg_type = setter_arg_type,
                .setter = physics.getIndirectFunction(@intCast(setter_index)) catch return,
                .setter_context = @intCast(setter_context),
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

            .physics = physics,

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
) callconv(.c) void {
    if (getPhysics(exec_env)) |physics| {
        const name = readLatin1String(physics, @intCast(name_ptr)) catch return;

        const allocator = physics.embind_allocator;

        const instance = allocator.create(Emscripten.Bind.Type.Instance) catch return;

        instance.* = .{
            .name = name,
            .kind = .@"enum",

            .physics = physics,

            .size = @intCast(size),
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
                        physics_inner: *Physics,
                        context: *anyopaque,
                        converters: Emscripten.Bind.Type.Converters,
                    ) Emscripten.Bind.Type.Converters {
                        const allocator_inner = physics_inner.embind_allocator;

                        const tuple_inner: *Emscripten.Bind.Tuple = @ptrCast(@alignCast(context));

                        const instance = allocator_inner.create(Emscripten.Bind.Type.Instance) catch return &.{};

                        instance.* = .{
                            .name = tuple_inner.name,
                            .kind = .tuple,

                            .physics = physics_inner,

                            .destructor = tuple_inner.destructor,

                            .tuple_constructor = tuple_inner.constructor,
                            .tuple_elements = tuple_inner.elements.items,
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

fn emval_get_method_caller(_: wamr.wasm_exec_env_t, _: i32, _: i32, _: i32) callconv(.c) void {}

fn emval_call_method(_: wamr.wasm_exec_env_t, _: i32, _: i32) callconv(.c) void {}

fn emval_decref(_: wamr.wasm_exec_env_t, _: i32) callconv(.c) void {}

fn emval_run_destructors(_: wamr.wasm_exec_env_t, _: i32) callconv(.c) void {}

fn fd_write(_: wamr.wasm_exec_env_t, _: i32, _: i32, _: i32, _: i32) callconv(.c) void {}

pub fn init(allocator: mem.Allocator) !*Physics {
    var physics = try allocator.create(Physics);
    errdefer allocator.destroy(physics);

    physics.* = .{
        .allocator = allocator,

        .aot_buf = try allocator.alignedAlloc(u8, .@"16", aot_buf_raw.len), // Change alignment for WAMR

        .heap_buf = try allocator.alignedAlloc(u8, .@"16", heap_size),

        .cached_indirect_functions = .init(allocator),

        .embind_arena = undefined,
        .embind_allocator = undefined,

        .embind_type_registry = undefined,

        .embind_tuple_registry = undefined,

        .embind_awaiting_dependencies = undefined,

        .embind_temp_arena = undefined,
        .embind_temp_allocator = undefined,

        .shape = .{ .physics = physics },
        .body = .{ .physics = physics },
        .constraint = .{ .physics = physics },
        .debug_geometry = .{ .physics = physics },
        .world = .{ .physics = physics },
        .event = .{ .physics = physics },
        .query_collector = .{ .physics = physics },
        .debug = .{ .physics = physics },
    };

    // Copy buf onto `aot_buf`
    @memcpy(physics.aot_buf, aot_buf_raw);

    {
        physics.embind_arena = .init(allocator);
        physics.embind_allocator = physics.embind_arena.allocator();

        physics.embind_type_registry = .init(physics.embind_allocator);

        physics.embind_tuple_registry = .init(physics.embind_allocator);

        physics.embind_awaiting_dependencies = .init(physics.embind_allocator);
    }

    physics.embind_temp_arena = .init(allocator);
    physics.embind_temp_allocator = physics.embind_temp_arena.allocator();

    var init_args: wamr.RuntimeInitArgs = .{};

    { // Allocate with pool
        init_args.mem_alloc_type = wamr.Alloc_With_Pool;

        init_args.mem_alloc_option.pool.heap_buf = physics.heap_buf.ptr;
        init_args.mem_alloc_option.pool.heap_size = heap_size;
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

    var error_buf: [128]u8 = undefined;

    physics.module = wamr.wasm_runtime_load(
        physics.aot_buf.ptr,
        comptime @intCast(aot_buf_raw.len),
        &error_buf[0],
        comptime @intCast(error_buf.len),
    );
    if (physics.module == null) {
        log.err("wasm runtime load failed: {s}", .{mem.sliceTo(&error_buf, 0)});

        return error.LoadFailed;
    }

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
        return error.ExecEnvCreationFailed;

    if (!wamr.wasm_runtime_get_export_table_inst(physics.module_inst, "__indirect_function_table", &physics.table_inst))
        return error.TableInstGetFailed;

    { // Start main
        _ = try physics.callExported("__wasm_call_ctors", &.{});

        if (!wamr.wasm_application_execute_main(physics.module_inst, 0, null)) {
            const exception = wamr.wasm_runtime_get_exception(physics.module_inst);

            log.err("wasm execution failed: {s}", .{exception});

            return error.MainExecutionFailed;
        }
    }

    return physics;
}

pub fn deinit(self: *Physics) void {
    // Dump before destroy
    // self.dumpPGOProfData("HavokPhysics.profraw") catch unreachable;

    if (self.exec_env) |exec_env| wamr.wasm_runtime_destroy_exec_env(exec_env);
    if (self.module_inst) |module_inst| wamr.wasm_runtime_deinstantiate(module_inst);
    if (self.module) |module| wamr.wasm_runtime_unload(module);

    wamr.wasm_runtime_destroy();

    const allocator = self.allocator;

    { // Free buffers
        allocator.free(self.aot_buf);

        allocator.free(self.heap_buf);
    }

    self.cached_indirect_functions.deinit();

    { // Free embind's
        self.embind_arena.deinit();
        self.embind_temp_arena.deinit();
    }

    allocator.destroy(self);
}

/// Dumps PGO profile data to the path `path`.
/// The original of this is `dump_pgo_prof_data`.
fn dumpPGOProfData(self: *Physics, path: []const u8) !void {
    const len = wamr.wasm_runtime_get_pgo_prof_data_size(self.module_inst);
    if (len == 0)
        return error.PgoSizeZero;

    const allocator = self.allocator;

    const buf = try allocator.alloc(u8, len);
    defer allocator.free(buf);

    if (len != wamr.wasm_runtime_dump_pgo_prof_data_to_buf(self.module_inst, buf.ptr, len))
        return error.PgoDumpFailed;

    const file = try fs.cwd().createFile(path, .{});
    defer file.close();

    try file.writeAll(buf[0..len]);

    log.debug("LLVM raw profile file {s} was generated", .{path});
}

/// Resets temp values. This does not affect the return values the physical methods returned.
/// NOTE: This should not be called for each physical call (like `world.create`) because of arena's characteristics,
/// so you should blockize your method calls, then call this in the block as defer.
pub fn free(self: *Physics) void {
    _ = self.embind_temp_arena.reset(.retain_capacity);
}

fn registerNativeSymbols(
    _: *const Physics,
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

fn registerType(
    self: *Physics,
    id: Emscripten.Bind.Type.Id,
    instance: *const Emscripten.Bind.Type.Instance,
) anyerror!void {
    if (self.embind_type_registry.contains(id))
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
}

const function_names = [_][]const u8{
    "HP_GetStatistics",

    "malloc",
    "free",

    "_malloc",
    "_free",

    "HP_World_CastRay",
    "HP_World_GetNextCollisionEvent",
    "HP_World_GetNextTriggerEvent",
    "HP_Shape_CreateConvexHull",
    "HP_Shape_CreateMesh",
    "HP_Shape_CreateContainer",
    "HP_Shape_SetFilterInfo",
    "HP_Shape_GetFilterInfo",
    "HP_Shape_RemoveChild",
    "HP_Shape_GetNumChildren",
    "HP_Shape_GetChildShape",
    "HP_Shape_GetType",
    "HP_Shape_Release",
    "HP_Shape_SetTrigger",
    "HP_Event_AsTrigger",
    "HP_Body_Create",
    "HP_Body_Release",
    "HP_Body_SetShape",
    "HP_Body_GetShape",
    "HP_Body_SetMotionType",
    "HP_Body_GetMotionType",
    "HP_Body_SetEventMask",
    "HP_Body_GetEventMask",
    "HP_Body_GetWorldTransformOffset",
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
    "HP_Constraint_SetEnabled",
    "HP_Constraint_GetEnabled",
    "HP_Constraint_SetCollisionsEnabled",
    "HP_Constraint_GetCollisionsEnabled",
    "HP_Constraint_SetAxisMode",
    "HP_Constraint_GetAxisMode",
    "HP_Constraint_SetAxisMotorType",
    "HP_Constraint_GetAxisMotorType",
    "HP_World_Create",
    "HP_World_Release",
    "HP_World_GetBodyBuffer",
    "HP_World_AddBody",
    "HP_World_RemoveBody",
    "HP_World_GetNumBodies",
    "HP_World_GetCollisionEvents",
    "HP_World_GetTriggerEvents",
    "HP_Debug_StartRecordingStats",
    "HP_QueryCollector_Create",
    "HP_QueryCollector_Release",
    "HP_QueryCollector_GetNumHits",
    "HP_Shape_CreateDebugDisplayGeometry",
    "HP_DebugGeometry_GetInfo",
    "HP_DebugGeometry_Release",
    "HP_Shape_PathIterator_GetNext",
    "HP_World_CastRayWithCollector",
    "HP_Body_GetPosition",
    "HP_Body_GetLinearVelocity",
    "HP_Body_GetAngularVelocity",
    "HP_Constraint_GetAppliedImpulses",
    "HP_Shape_CreateHeightField",
    "HP_Body_SetPosition",
    "HP_Body_SetLinearVelocity",
    "HP_Body_SetAngularVelocity",
    "HP_Body_ApplyImpulse",
    "HP_Body_ApplyAngularImpulse",
    "HP_Constraint_SetAnchorInParent",
    "HP_Constraint_SetAnchorInChild",
    "HP_World_SetGravity",
    "HP_Body_GetQTransform",
    "HP_Shape_GetBoundingBox",
    "HP_Body_SetQTransform",
    "HP_Body_SetTargetQTransform",
    "HP_Shape_AddChild",
    "HP_World_ShapeCastWithCollector",
    "HP_Body_GetOrientation",
    "HP_Shape_CreateBox",
    "HP_Body_SetOrientation",
    "HP_Shape_GetMaterial",
    "HP_Shape_SetMaterial",
    "HP_Shape_BuildMassProperties",
    "HP_Body_GetMassProperties",
    "HP_Body_SetMassProperties",
    "HP_QueryCollector_GetCastRayResult",
    "HP_World_PointProximityWithCollector",
    "HP_QueryCollector_GetPointProximityResult",
    "HP_World_ShapeProximityWithCollector",
    "HP_QueryCollector_GetShapeProximityResult",
    "HP_QueryCollector_GetShapeCastResult",
    "HP_Event_AsCollision",
    "HP_Shape_GetDensity",
    "HP_Body_GetLinearDamping",
    "HP_Body_GetAngularDamping",
    "HP_Body_GetGravityFactor",
    "HP_Constraint_GetAxisFriction",
    "HP_Constraint_GetAxisMinLimit",
    "HP_Constraint_GetAxisMaxLimit",
    "HP_Constraint_GetAxisMotorTarget",
    "HP_Constraint_GetAxisMotorMaxForce",
    "HP_Constraint_GetAxisMotorPositionTarget",
    "HP_Constraint_GetAxisMotorVelocityTarget",
    "HP_Constraint_GetAxisMotorStiffness",
    "HP_Constraint_GetAxisMotorDamping",
    "HP_World_GetSpeedLimit",
    "HP_Shape_CreateSphere",
    "HP_Shape_CreateCapsule",
    "HP_Shape_CreateCylinder",
    "HP_Shape_SetDensity",
    "HP_Body_SetLinearDamping",
    "HP_Body_SetAngularDamping",
    "HP_Body_SetGravityFactor",
    "HP_Constraint_SetAxisFriction",
    "HP_Constraint_SetAxisMinLimit",
    "HP_Constraint_SetAxisMaxLimit",
    "HP_Constraint_SetAxisStiffness",
    "HP_Constraint_SetAxisDamping",
    "HP_Constraint_SetAxisMotorTarget",
    "HP_Constraint_SetAxisMotorMaxForce",
    "HP_Constraint_SetAxisMotorPositionTarget",
    "HP_Constraint_SetAxisMotorVelocityTarget",
    "HP_Constraint_SetAxisMotorStiffness",
    "HP_Constraint_SetAxisMotorDamping",
    "HP_World_Step",
    "HP_World_SetIdealStepTime",
    "HP_World_SetSpeedLimit",
    "HP_Debug_StopRecordingStats",

    "main",

    "__getTypeName",

    "__wasm_call_ctors",
};

const CachedFunctionIndices = comptime_string_map.ComptimeStringMap(usize);

const cached_function_indices: CachedFunctionIndices = blk: {
    var kvs: CachedFunctionIndices.KeyValues = &.{};

    for (function_names, 0..) |name, i|
        kvs = kvs ++ .{.{ name, i }};

    break :blk .initComptime(kvs);
};

fn call(self: *Physics, function: wamr.wasm_function_inst_t, args: []const Emscripten.Bind.Type.Instance.Wire) !u32 {
    var argv: [16]u32 = undefined;
    var argv_len: u32 = 0;

    for (args) |arg| {
        const is_multiple = arg.is_multiple;

        argv[argv_len] = @truncate(arg.value);

        if (is_multiple)
            argv[argv_len + 1] = @truncate(arg.value >> 32);

        argv_len +=
            if (is_multiple)
                2
            else
                1;
    }

    if (!wamr.wasm_runtime_call_wasm(
        self.exec_env,
        function,
        argv_len,
        &argv,
    )) {
        const exception = wamr.wasm_runtime_get_exception(self.module_inst);

        log.err("wasm execution failed: {s}", .{exception});

        return error.FunctionCallFailed;
    }

    return argv[0];
}

fn callExported(self: *Physics, comptime name: [:0]const u8, args: []const Emscripten.Bind.Type.Instance.Wire) !u32 {
    return self.call(try self.getExportedFunction(name), args);
}

fn callSimple(self: *Physics, function: wamr.wasm_function_inst_t, args: anytype) !u32 {
    const args_info_struct_fields = @typeInfo(@TypeOf(args)).@"struct".fields;

    var argv: [args_info_struct_fields.len]u32 = undefined;

    inline for (args_info_struct_fields, 0..) |field, i|
        argv[i] = @intCast(@field(args, field.name));

    if (!wamr.wasm_runtime_call_wasm(
        self.exec_env,
        function,
        comptime @intCast(args_info_struct_fields.len),
        &argv,
    )) {
        const exception = wamr.wasm_runtime_get_exception(self.module_inst);

        log.err("wasm execution failed: {s}", .{exception});

        return error.FunctionCallFailed;
    }

    return argv[0];
}

fn callExportedSimple(self: *Physics, comptime name: [:0]const u8, args: anytype) !u32 {
    return self.callSimple(try self.getExportedFunction(name), args);
}

/// You must use this function if your arguments is empty.
fn callVoid(self: *Physics, function: wamr.wasm_function_inst_t) !u32 {
    var argv: [1]u32 = undefined;

    if (!wamr.wasm_runtime_call_wasm(
        self.exec_env,
        function,
        0, // argc = 0
        &argv,
    )) {
        const exception = wamr.wasm_runtime_get_exception(self.module_inst);

        log.err("wasm execution failed: {s}", .{exception});

        return error.FunctionCallFailed;
    }

    return argv[0];
}

/// You must use this function if your function is exported and arguments is empty.
fn callExportedVoid(self: *Physics, comptime name: [:0]const u8) !u32 {
    return self.callVoid(try self.getExportedFunction(name));
}

fn getExportedFunction(self: *Physics, comptime name: [:0]const u8) !wamr.wasm_function_inst_t {
    const function_index = comptime cached_function_indices.get(name) orelse
        @compileError(fmt.comptimePrint("uncached function name: {s}", .{name}));

    return self.cached_functions[function_index] orelse blk: {
        const function = wamr.wasm_runtime_lookup_function(self.module_inst, name.ptr);
        if (function == null)
            return error.FunctionNotFound;

        self.cached_functions[function_index] = function;

        break :blk function;
    };
}

fn getIndirectFunction(self: *Physics, index: u32) !wamr.wasm_function_inst_t {
    return self.cached_indirect_functions.get(index) orelse blk: {
        const function = wamr.wasm_table_get_func_inst(self.module_inst, &self.table_inst, index);

        try self.cached_indirect_functions.put(index, function);

        break :blk function;
    };
}

comptime {
    testing.refAllDeclsRecursive(Physics);
}

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
const fs = std.fs;
const os = std.os;
const testing = std.testing;

const wamr = @import("wamr").wasm_export;

const comptime_string_map = @import("ComptimeStringMap.zig");
