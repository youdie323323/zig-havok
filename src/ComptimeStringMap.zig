/// String to V mapping for comptime-known key/value pairs.
/// Branches on the key length, then compares each string.
pub fn ComptimeStringMap(comptime Value: type) type {
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
            return self.get(key) != null;
        }

        /// Returns the value for the key if any.
        pub fn get(comptime self: @This(), comptime key: Key) ?Value {
            comptime {
                @setEvalBranchQuota(200 * self.kvs.len * self.kvs.len);

                const len_kvs_set = self.groupKVsWithLength();

                for (len_kvs_set) |len_kvs| {
                    const len = len_kvs.len;
                    if (key.len == len)
                        for (len_kvs.kvs) |kv|
                            if (eql(u8, kv[0], key[0..len]))
                                return kv[1];
                }

                return null;
            }
        }

        /// Creates a list of kv sets grouped by different key lengths.
        fn groupKVsWithLength(comptime self: @This()) []const KeyValuesByLength {
            comptime {
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
                                if (eql(u8, kv[0], add_kv_key[0..check_kv_key_len]))
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

const eql = @import("std").mem.eql;
