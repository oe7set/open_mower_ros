// Public surface of the meta.config.* helpers.
//
// The functions are thin enough that a header alone documents them. Errors
// are signalled via std::runtime_error so RPC handlers can translate them
// into RpcException(ERROR_INTERNAL).

#pragma once

#include <nlohmann/json.hpp>
#include <string>

namespace xbot_monitoring::config_io {

// Read a UTF-8 text file. Throws std::runtime_error on I/O errors.
std::string read_text_file(const std::string& path);

// Atomic write: <path>.tmp + rename. Throws std::runtime_error on I/O errors.
void write_text_file_atomic(const std::string& path, const std::string& content);

// Parse a `mower_config.sh`-style file body. Returns a flat object whose keys
// are env-var names and values are the decoded scalar values (string).
nlohmann::ordered_json parse_config_sh(const std::string& content);

// Rewrite `original` so that any keys present in `changes` carry their new
// value. Original line order, comments and blank lines are preserved. Keys
// from `changes` that don't appear in `original` are appended at the end.
std::string write_config_sh(const std::string& original, const nlohmann::ordered_json& changes);

// Build the {filename: yaml_content} object the frontend's settings form
// expects from meta.config.defaults(). The synthesized defaults.yaml lists
// every property's `default` value from the JSON schema.
nlohmann::ordered_json defaults_yaml_from_schema(const nlohmann::ordered_json& schema);

}  // namespace xbot_monitoring::config_io
