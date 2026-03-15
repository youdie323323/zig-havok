/// Comptime mapping between strings and values (of type Value).
/// All key/value pairs must be known at compile-time.
pub fn ComptimeStringMap(comptime Value: type) type {
    return ComptimeStringMapWithEql(Value, staticEql);
}

/// Same as StaticStringMap, except keys are compared case-insensitively.
/// Just as a little reminder, this is slower than normal StaticStringMap.
pub fn ComptimeStringMapIgnoreCaseAscii(comptime Value: type) type {
    return ComptimeStringMapWithEql(Value, staticEqlIgnoreCaseAscii);
}

/// String to value mapping for comptime-known key/value pairs.
/// Branches on the key length, then compares each string.
/// `Value` must not be comptime type as to ensure `getRuntime` works properly.
fn ComptimeStringMapWithEql(comptime Value: type, comptime eql: anytype) type {
    return struct {
        const Key = []const u8;

        pub const KeyValue =
            if (Value == void)
                struct { Key, comptime Value = {} }
            else
                struct { Key, Value };

        pub const KeyValues = []const KeyValue;

        /// KeyValues where grouped with same key length.
        const KeyValuesByLength = struct { len: usize, kvs: KeyValues };

        kvs: KeyValues,

        /// Initializes the map at comptime with a list of key/value pairs.
        /// The "value" in a key/value pair is optional if type V is void.
        pub inline fn initComptime(comptime kvs: KeyValues) @This() {
            comptime return .{ .kvs = kvs };
        }

        /// Returns the list of all the keys in the map.
        pub fn keys(comptime self: @This()) []const Key {
            comptime {
                var list: [self.kvs.len]Key = undefined;

                for (&list, self.kvs) |*key, kv|
                    key.* = kv[0];

                return &list;
            }
        }

        /// Returns the list of all the values in the map.
        pub fn values(comptime self: @This()) []const Value {
            comptime {
                var list: [self.kvs.len]Value = undefined;

                for (&list, self.kvs) |*value, kv|
                    value.* = kv[1];

                return &list;
            }
        }

        /// Checks if the map contains the key.
        pub fn has(comptime self: @This(), comptime key: Key) bool {
            comptime return self.get(key) != null;
        }

        /// Returns the value for the key if any.
        ///
        /// The return type is promoted to `comptime_int` or `comptime_float`
        /// when `Value` is an integer or a float type, respectively.
        pub fn get(comptime self: @This(), comptime key: Key) ?switch (@typeInfo(Value)) {
            .int => comptime_int,
            .float => comptime_float,
            else => Value,
        } {
            comptime {
                const value = self.getRuntime(key) orelse return null;

                return switch (@typeInfo(Value)) {
                    .int => @intCast(value),
                    .float => @floatCast(value),
                    else => value,
                };
            }
        }

        /// Returns the value for the key if any.
        pub fn getUnoptional(comptime self: @This(), comptime key: Key) meta.Child(@TypeOf(self.get(key))) {
            comptime return self.get(key) orelse @compileError(fmt.comptimePrint("unregistered key: {s}", .{key}));
        }

        /// Runtimely returns the value for the key if any.
        pub fn getRuntime(comptime self: @This(), key: Key) ?Value {
            @setEvalBranchQuota(200 * math.pow(usize, self.kvs.len, 2));

            inline for (comptime self.groupKVsWithLength()) |len_kvs| {
                const len = len_kvs.len;
                if (key.len == len)
                    inline for (len_kvs.kvs) |len_kv| {
                        const is_key_eql =
                            if (@inComptime()) // Use standard `mem.eql` at comptime since we don't care about performance in comptime
                                mem.eql(u8, len_kv[0], key)
                            else
                                eql(len_kv[0], key[0..len]);

                        if (is_key_eql)
                            return len_kv[1];
                    };
            }

            return null;
        }

        /// Creates a list of kv sets grouped by different key lengths.
        fn groupKVsWithLength(comptime self: @This()) []const KeyValuesByLength {
            comptime {
                @setEvalBranchQuota(10_000 * math.pow(usize, self.kvs.len + 1, 2));

                var len_kvs_set: []const KeyValuesByLength = &.{};

                add_length: for (self.kvs, 0..) |check_kv, i| {
                    // The key/value pairs with this length will be grouped
                    const check_kv_key_len = check_kv[0].len;

                    // Skip this key/value pair if it has already been grouped
                    for (len_kvs_set) |len_kvs|
                        if (len_kvs.len == check_kv_key_len)
                            continue :add_length;

                    var added_kvs: KeyValues = &.{};

                    for (self.kvs[i..]) |add_kv| {
                        const add_kv_key, _ = add_kv;

                        if (add_kv_key.len == check_kv_key_len) {
                            // Check for redundant keys
                            for (added_kvs) |kv|
                                if (eql(kv[0], add_kv_key[0..check_kv_key_len]))
                                    @compileError("redundant key \"" ++ add_kv_key ++ "\"");

                            added_kvs = added_kvs ++ .{add_kv};
                        }
                    }

                    const added_len_kvs: KeyValuesByLength = .{ .len = check_kv_key_len, .kvs = added_kvs };

                    len_kvs_set = len_kvs_set ++ .{added_len_kvs};
                }

                return len_kvs_set;
            }
        }
    };
}

/// Equality check for a compile-time known string and a runtime-known string.
/// An optimizer can generate similar code, but this code is explicit to ensure
/// we get optimal codegen - specific to each static string's length and value.
fn staticEql(comptime a: []const u8, b: *const [a.len]u8) bool {
    const block_len = comptime (simd.suggestVectorLength(u8) orelse @sizeOf(usize));

    const Chunk = meta.Int(.unsigned, 8 * block_len);

    // Compare `block_count` chunks of `block_len` bytes at a time
    const block_count = comptime (a.len / block_len);

    const block_count_block_len = comptime (block_count * block_len);

    inline for (0..block_count) |i| {
        const i_block_len = comptime (i * block_len);

        const chunk_a: Chunk = comptime @bitCast(a[i_block_len..][0..block_len].*);
        const chunk_b: Chunk = @bitCast(b[i_block_len..][0..block_len].*);

        if (chunk_a != chunk_b) return false;
    }

    // Compare the remainder `rem_count` bytes of both strings
    const remainder_count = comptime (a.len % block_len);

    const Remainder = meta.Int(.unsigned, 8 * remainder_count);

    const remainder_a: Remainder = comptime @bitCast(a[block_count_block_len..][0..remainder_count].*);
    const remainder_b: Remainder = @bitCast(b[block_count_block_len..][0..remainder_count].*);

    return remainder_a == remainder_b;
}

/// Case-insensitive equality check for equal length comptime & runtime string.
fn staticEqlIgnoreCaseAscii(comptime a: []const u8, b: *const [a.len]u8) bool {
    const a_upper = comptime simdToUpper(a.len, a[0..a.len]);
    const b_upper = simdToUpper(a.len, b);

    return staticEql(&a_upper, &b_upper);
}

/// Vectorized uppercase transform for a string with a comptime-known length
fn simdToUpper(comptime len: usize, a: *const [len]u8) [len]u8 {
    const block_len = comptime (simd.suggestVectorLength(u8) orelse @sizeOf(usize));

    var result: [len]u8 = undefined;

    // Convert `block_count` chunks of `block_len` bytes at a time
    const block_count = comptime (len / block_len);

    // Convert the remainder `rem_count` bytes
    const remainder_count = comptime (len % block_len);

    { // Chunk
        const ByteChunk = @Vector(block_len, u8);
        const BoolChunk = @Vector(block_len, bool);

        // Mask the lowercase bytes so they are in the range 'A'...'Z'
        const mask_byte_chunk: ByteChunk = @splat(0b1101_1111);

        const false_bool_chunk: BoolChunk = comptime @splat(false);

        inline for (0..block_count) |i| {
            const offset = comptime (i * block_len);

            // Determine if the chunk is in the range of 'a'...'z'
            const chunk: ByteChunk = a[offset..][0..block_len].*;

            const min: BoolChunk = chunk >= comptime @as(ByteChunk, @splat('a'));
            const max: BoolChunk = chunk <= comptime @as(ByteChunk, @splat('z'));

            const is_lower = @select(bool, min, max, false_bool_chunk);

            const uppercased = @select(u8, is_lower, chunk & mask_byte_chunk, chunk);

            result[offset..][0..block_len].* = uppercased;
        }
    }

    { // Remainder
        const ByteRemainder = @Vector(remainder_count, u8);
        const BoolRemainder = @Vector(remainder_count, bool);

        // Mask the lowercase bytes so they are in the range 'A'...'Z'
        const mask_byte_remainder: ByteRemainder = @splat(0b1101_1111);

        const false_bool_remainder: BoolRemainder = @splat(false);

        const offset = comptime (block_count * block_len);

        // Determine if the remainder is in the range of 'a'...'z'
        const remainder: ByteRemainder = a[offset..][0..remainder_count].*;

        const min: BoolRemainder = remainder >= comptime @as(ByteRemainder, @splat('a'));
        const max: BoolRemainder = remainder <= comptime @as(ByteRemainder, @splat('z'));

        const is_lower = @select(bool, min, max, false_bool_remainder);

        const uppercased = @select(u8, is_lower, remainder & mask_byte_remainder, remainder);

        result[offset..][0..remainder_count].* = uppercased;
    }

    return result;
}

const std = @import("std");
const debug = std.debug;
const assert = debug.assert;
const testing = std.testing;
const simd = std.simd;
const meta = std.meta;
const mem = std.mem;
const fmt = std.fmt;
const math = std.math;

test staticEql {
    const corpus: []const *const [5]u8 = &.{
        "aback", "abase", "abate", "abbey", "abbot", "abhor", "abide", "abled",
        "abode", "abort", "about", "above", "abuse", "abyss", "acorn", "acrid",
    };

    // Strings are equal to themselves
    inline for (corpus) |a|
        try testing.expect(staticEql(a, a));

    // Unequal strings are just that - not equal
    inline for (corpus, 0..) |a, i|
        inline for (corpus[0..i]) |b|
            try testing.expect(!staticEql(a, b));
}

test staticEqlIgnoreCaseAscii {
    const corpus_a: []const *const [5]u8 = &.{
        "wRING", "WRIst", "WRiTe", "wronG", "WROte", "WruNg", "wryly", "yacht",
        "yeaRN", "yeAST", "YIEld", "young", "youth", "zeBRa", "zESTy", "zONal",
    };
    const corpus_b: []const *const [5]u8 = &.{
        "wrIng", "wRISt", "WritE", "WRONG", "wROTE", "wRUnG", "wrYlY", "yAcHt",
        "yEARn", "yeAst", "yiEld", "yoUng", "youTh", "zeBra", "zEsTY", "zonaL",
    };

    // Strings are equal to themselves
    inline for (corpus_a, corpus_b) |a, b| {
        try testing.expect(staticEqlIgnoreCaseAscii(a, a));
        try testing.expect(staticEqlIgnoreCaseAscii(b, b));
    }

    // Strings are equal regardless of alphabetic case
    inline for (corpus_a, corpus_b) |a, b|
        try testing.expect(staticEqlIgnoreCaseAscii(a, b));

    // Unequal strings are just that - not equal
    inline for (corpus_a, corpus_b, 0..) |a_1, b_1, i|
        inline for (corpus_a[0..i], corpus_b[0..i]) |a_2, b_2| {
            try testing.expect(!staticEqlIgnoreCaseAscii(a_1, a_2));
            try testing.expect(!staticEqlIgnoreCaseAscii(a_1, b_2));
            try testing.expect(!staticEqlIgnoreCaseAscii(b_1, a_2));
            try testing.expect(!staticEqlIgnoreCaseAscii(b_1, b_2));
        };
}

test simdToUpper {
    const inputs: []const []const u8 = &.{
        "0",
        "abc",
        "123abc!!!",
        "0xdEaDBeeF",
        "this is a test string, I don't know.",
        "this is the\xFFspiciest test\x00string to ever exist in zig",
    };
    const expects: []const []const u8 = &.{
        "0",
        "ABC",
        "123ABC!!!",
        "0XDEADBEEF",
        "THIS IS A TEST STRING, I DON'T KNOW.",
        "THIS IS THE\xFFSPICIEST TEST\x00STRING TO EVER EXIST IN ZIG",
    };

    inline for (inputs, expects) |input, expected| {
        const actual = simdToUpper(input.len, input[0..input.len]);

        try testing.expectEqualSlices(u8, expected, &actual);
    }
}

const TestEnum = enum { a, b, c, d, e };

const TestEnumValueMap = ComptimeStringMap(TestEnum);
const TestEnumValueMapIgnoreCaseAscii = ComptimeStringMapIgnoreCaseAscii(TestEnum);

const TestVoidValueMap = ComptimeStringMap(void);

test "list literal of list literals" {
    const slice: TestEnumValueMap.KeyValues = &.{
        .{ "these", .d },
        .{ "have", .a },
        .{ "nothing", .b },
        .{ "incommon", .c },
        .{ "samelen", .e },
    };

    const map: TestEnumValueMap = .initComptime(slice);

    try verifyEnumValueMapAnyOption(map);

    // Default comparison is case sensitive
    try testing.expect(null == map.get("NOTHING"));
}

test "get/has with edge cases" {
    const map: ComptimeStringMap(u32) = .initComptime(&.{
        .{ "a", 0 },
        .{ "ab", 3 },
        .{ "abc", 0 },
        .{ "abcd", 1 },
        .{ "abcde", 1 },
    });

    try testing.expectEqual(false, map.has("abcdef"));
    try testing.expectEqual(true, map.has("abcde"));
    try testing.expectEqual(3, map.get("ab"));
    try testing.expectEqual(0, map.get("a"));
    try testing.expectEqual(null, map.get(""));
}

test "array of structs" {
    const array = [_]TestEnumValueMap.KeyValue{
        .{ "these", .d },
        .{ "have", .a },
        .{ "nothing", .b },
        .{ "incommon", .c },
        .{ "samelen", .e },
    };

    try verifyEnumValueMapAnyOption(TestEnumValueMap.initComptime(&array));
}

test "slice of structs" {
    const array = [_]TestEnumValueMap.KeyValue{
        .{ "these", .d },
        .{ "have", .a },
        .{ "nothing", .b },
        .{ "incommon", .c },
        .{ "samelen", .e },
    };

    const slice: TestEnumValueMap.KeyValues = array[0..array.len];

    try verifyEnumValueMapAnyOption(TestEnumValueMap.initComptime(slice));
}

test "void value type, slice of structs" {
    const array = [_]TestVoidValueMap.KeyValue{
        .{"these"},
        .{"have"},
        .{"nothing"},
        .{"incommon"},
        .{"samelen"},
    };

    const map: TestVoidValueMap = .initComptime(&array);

    try verifyVoidValueMap(map);

    // Default comparison is case sensitive
    try testing.expect(null == map.get("NOTHING"));
}

test "void value type, list literal of list literals" {
    const array = [_]TestVoidValueMap.KeyValue{
        .{"these"},
        .{"have"},
        .{"nothing"},
        .{"incommon"},
        .{"samelen"},
    };

    try verifyVoidValueMap(.initComptime(&array));
}

fn verifyVoidValueMap(comptime map: TestVoidValueMap) !void {
    try testing.expectEqual({}, map.get("have").?);
    try testing.expectEqual({}, map.get("nothing").?);

    try testing.expect(null == map.get("missing"));

    try testing.expectEqual({}, map.get("these").?);
    try testing.expectEqual({}, map.get("samelen").?);

    try testing.expect(!map.has("missing"));
    try testing.expect(map.has("these"));

    try testing.expect(null == map.get(""));
    try testing.expect(null == map.get("averylongstringthathasnomatches"));
}

fn verifyEnumValueMapAnyOption(comptime map: anytype) !void {
    try testing.expectEqual(TestEnum.a, map.get("have").?);
    try testing.expectEqual(TestEnum.b, map.get("nothing").?);

    try testing.expect(null == map.get("missing"));

    try testing.expectEqual(TestEnum.d, map.get("these").?);
    try testing.expectEqual(TestEnum.e, map.get("samelen").?);

    try testing.expect(!map.has("missing"));
    try testing.expect(map.has("these"));

    try testing.expect(null == map.get(""));
    try testing.expect(null == map.get("averylongstringthathasnomatches"));
}

fn verifyEnumValueMapIgnoreCaseAscii(comptime map: TestEnumValueMapIgnoreCaseAscii) !void {
    try verifyEnumValueMapAnyOption(map);

    try testing.expectEqual(TestEnum.a, map.get("HAVE").?);
    try testing.expectEqual(TestEnum.e, map.get("SameLen").?);

    try testing.expect(null == map.get("SameLength"));

    try testing.expect(map.has("ThESe"));
}

test "StaticStringMapIgnoreCaseAscii" {
    const array = [_]TestEnumValueMapIgnoreCaseAscii.KeyValue{
        .{ "these", .d },
        .{ "have", .a },
        .{ "nothing", .b },
        .{ "incommon", .c },
        .{ "samelen", .e },
    };

    try verifyEnumValueMapIgnoreCaseAscii(.initComptime(&array));
}

test "empty" {
    const m1: ComptimeStringMap(usize) = .initComptime(&.{});
    try testing.expect(null == m1.get("anything"));

    const m2: ComptimeStringMapIgnoreCaseAscii(usize) = .initComptime(&.{});
    try testing.expect(null == m2.get("anything"));
}

test "comptime-only value" {
    const map: ComptimeStringMap(type) = .initComptime(&.{
        .{ "a", struct {
            pub const foo = 1;
        } },
        .{ "b", struct {
            pub const foo = 2;
        } },
        .{ "c", struct {
            pub const foo = 3;
        } },
    });

    try testing.expect(map.get("a").?.foo == 1);
    try testing.expect(map.get("b").?.foo == 2);
    try testing.expect(map.get("c").?.foo == 3);
    try testing.expect(map.get("d") == null);
}

test "sorting kvs doesn't exceed eval branch quota" {
    // From https://github.com/ziglang/zig/issues/19803
    const TypeToByteSizeLUT: ComptimeStringMap(u32) = .initComptime(&.{
        .{ "bool", 0 },
        .{ "c_int", 0 },
        .{ "c_long", 0 },
        .{ "c_longdouble", 0 },
        .{ "t20", 0 },
        .{ "t19", 0 },
        .{ "t18", 0 },
        .{ "t17", 0 },
        .{ "t16", 0 },
        .{ "t15", 0 },
        .{ "t14", 0 },
        .{ "t13", 0 },
        .{ "t12", 0 },
        .{ "t11", 0 },
        .{ "t10", 0 },
        .{ "t9", 0 },
        .{ "t8", 0 },
        .{ "t7", 0 },
        .{ "t6", 0 },
        .{ "t5", 0 },
        .{ "t4", 0 },
        .{ "t3", 0 },
        .{ "t2", 0 },
        .{ "t1", 1 },
    });

    try testing.expectEqual(1, TypeToByteSizeLUT.get("t1"));
}

test "single string StaticStringMap" {
    const map: TestVoidValueMap = .initComptime(&.{
        .{"Hello, World!"},
    });

    try testing.expectEqual(true, map.has("Hello, World!"));
    try testing.expectEqual(false, map.has("Same len str!"));
    try testing.expectEqual(false, map.has("Hello, World! (not the same)"));
}

test "empty StaticStringMap" {
    const map: TestVoidValueMap = .initComptime(&.{});

    try testing.expectEqual(false, map.has(&.{}));
    try testing.expectEqual(null, map.get(&.{}));
    try testing.expectEqual(false, map.has("anything really"));
}
