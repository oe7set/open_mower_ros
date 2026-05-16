// Public surface of the meta.config.* helpers.
//
// The functions are thin enough that a header alone documents them. Errors
// are signalled via std::runtime_error so RPC handlers can translate them
// into RpcException(ERROR_INTERNAL).

#pragma once

#include <nlohmann/json.hpp>
#include <string>
#include <unordered_map>
#include <vector>

namespace xbot_monitoring::config_io {

// Read a UTF-8 text file. Throws std::runtime_error on I/O errors.
std::string read_text_file(const std::string& path);

// Atomic write: <path>.tmp + rename. Throws std::runtime_error on I/O errors.
void write_text_file_atomic(const std::string& path, const std::string& content);

// Parse a `mower_config.sh`-style file body. Returns a flat object whose keys
// are env-var names and values are the decoded scalar values (string).
nlohmann::ordered_json parse_config_sh(const std::string& content);

// Read a KEY=VALUE style env file (Docker Compose .env or mower_config.sh).
// Returns an empty object if the file does not exist; throws on parse errors.
// Reuses parse_config_sh internally — both formats share the same syntax.
nlohmann::ordered_json read_env_file(const std::string& path);

// Rewrite `original` so that any keys present in `changes` carry their new
// value. Original line order, comments and blank lines are preserved. Keys
// from `changes` that don't appear in `original` are appended at the end.
std::string write_config_sh(const std::string& original, const nlohmann::ordered_json& changes);

// Build the {filename: yaml_content} object the frontend's settings form
// expects from meta.config.defaults(). The synthesized defaults.yaml lists
// every property's `default` value from the JSON schema.
nlohmann::ordered_json defaults_yaml_from_schema(const nlohmann::ordered_json& schema);

// Description of a single schema leaf as seen from the source-routing system.
// Populated by collect_schema_leaves(); used by meta.config.{get,set} to know
// where each value lives (env file, user YAML, hardware YAML or ROS param).
struct SchemaLeaf {
  // Optional OM_* environment-variable name from the legacy
  // x-environment-variable annotation. Empty when the leaf only has a YAML
  // path (new-style fields).
  std::string env_var;
  // Where the value is sourced from: "env" | "yaml-user" | "yaml-hw" | "ros".
  // Empty if no x-source annotation is present.
  std::string source;
  // Dotted YAML path (e.g. "mower_logic.docking_approach_distance"). Empty
  // when the leaf is sourced from env or ros.
  std::string yaml_path;
  // Fully-qualified ROS parameter name (e.g. "/move_base_flex/FTCPlanner/kp_lat").
  // Empty when the leaf is not ROS-sourced.
  std::string ros_param;
  // True when x-readonly-via-ui is set; meta.config.set must skip such keys.
  bool readonly_via_ui = false;
  // JSON Schema "type" of the leaf, used for value coercion.
  std::string type;
};

// Walk the schema and collect every leaf that has either x-environment-variable
// or x-source. The first map is keyed by the OM_* name (legacy lookups);
// the second is a flat list used to enumerate ROS-param names, validate
// readonly-via-ui sets, and resolve x-yaml-path-only saves.
struct SchemaLeafIndex {
  std::unordered_map<std::string, SchemaLeaf> by_env_var;
  // Indexed by yaml_path so meta.config.set can accept new-style payloads
  // that send YAML paths directly (no OM_* prefix). Populated only for
  // leaves with non-empty yaml_path.
  std::unordered_map<std::string, SchemaLeaf> by_yaml_path;
  // Indexed by ROS parameter name. Used to validate params.get_many
  // requests against the schema-declared whitelist.
  std::unordered_map<std::string, SchemaLeaf> by_ros_param;
  // All leaves, in walk order.
  std::vector<SchemaLeaf> all;
};

SchemaLeafIndex collect_schema_leaves(const nlohmann::ordered_json& schema);

}  // namespace xbot_monitoring::config_io
