// Helpers used by the meta.config.* RPC methods to bridge the legacy OM_*
// schema to the YAML-native config layout (OpenMowerOS v2). The schema is
// preserved as-is; a separate JSON mapping file translates each OM_* key to
// the dotted YAML path it should be read from / written to.

#pragma once

#include <nlohmann/json.hpp>

#include <string>

namespace xbot_monitoring::yaml_io {

// Read a YAML file and return it parsed as JSON. Empty file → empty object.
// Throws std::runtime_error on read or parse error.
nlohmann::ordered_json read_yaml_file(const std::string& path);

// Serialise a JSON object to YAML and write it atomically (tempfile + rename)
// to `path`. Mirrors the atomic-write pattern in config_io.cpp.
void write_yaml_file_atomic(const std::string& path, const nlohmann::ordered_json& data);

// Resolve a dotted path like "ll.services.gps.datum_lat" inside a JSON object.
// Returns nullptr if any segment is missing. Does not autovivify.
const nlohmann::ordered_json* read_path(const nlohmann::ordered_json& root, const std::string& dotted_path);

// Walks the dotted path inside `root`, creating intermediate objects as
// needed, and writes `value` at the leaf. If a non-object lives at an
// intermediate segment, it is replaced with a new object — callers with
// existing data should read first.
void write_path(nlohmann::ordered_json& root, const std::string& dotted_path, const nlohmann::ordered_json& value);

// Deep-merge `over` into `base` (in-place). Object keys from `over` override
// `base`; arrays and primitives are replaced wholesale.
void deep_merge(nlohmann::ordered_json& base, const nlohmann::ordered_json& over);

}  // namespace xbot_monitoring::yaml_io
