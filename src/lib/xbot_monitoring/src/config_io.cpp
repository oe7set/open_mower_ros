// Helpers used by the meta.config.* RPC methods in xbot_monitoring.
//
// We deliberately keep the file small and self-contained so it can be unit-tested
// in isolation if needed. The shell-config parser/writer round-trips
// `mower_config.sh` while preserving the file's original line order, comments
// and blank lines — anything we don't recognise as `[export ]NAME=value` passes
// through unchanged.
//
// Reading a YAML defaults blob from the JSON schema lets the frontend pretend
// the mower runs on the same file-based defaults stack as the mowglinext app
// (defaults.yaml + board.yaml + mower.yaml). For now we only ever populate the
// top-level defaults.yaml; the other slots stay empty strings.

#include "config_io.h"

#include <ros/ros.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <set>
#include <sstream>

namespace fs = std::filesystem;
using json = nlohmann::ordered_json;

namespace xbot_monitoring::config_io {

namespace {

// Strip surrounding double or single quotes if both ends match. Shell config
// commonly stores OM_FOO="bar" or OM_FOO='bar baz'; we normalise both to bar.
std::string unquote(std::string v) {
  if (v.size() >= 2 && ((v.front() == '"' && v.back() == '"') || (v.front() == '\'' && v.back() == '\''))) {
    return v.substr(1, v.size() - 2);
  }
  return v;
}

// Quote a value for shell output. We always emit double quotes to be safe with
// whitespace and special characters; backslashes and existing double quotes are
// escaped. This keeps round-trips stable for values that didn't need quoting on
// the way in.
std::string shell_quote(const std::string& v) {
  std::string out;
  out.reserve(v.size() + 2);
  out.push_back('"');
  for (char c : v) {
    if (c == '"' || c == '\\' || c == '$' || c == '`') {
      out.push_back('\\');
    }
    out.push_back(c);
  }
  out.push_back('"');
  return out;
}

// Recursively walk the JSON Schema and collect default values keyed by their
// x-environment-variable mapping (or the property name as a fallback).
void collect_defaults(const json& schema, json& out) {
  if (!schema.is_object()) return;
  if (schema.contains("properties") && schema["properties"].is_object()) {
    for (auto& [key, prop] : schema["properties"].items()) {
      if (!prop.is_object()) continue;
      if (prop.contains("default")) {
        std::string env_name = key;
        if (prop.contains("x-environment-variable") && prop["x-environment-variable"].is_string()) {
          env_name = prop["x-environment-variable"];
        }
        out[env_name] = prop["default"];
      }
      collect_defaults(prop, out);
    }
  }
  // Schemas in this repo also use allOf / if-then-else for conditional fields.
  // The defaults inside `then` / `else` are not unconditional, so we skip them
  // here and rely on the form to apply them via the schema engine.
}

}  // namespace

std::string read_text_file(const std::string& path) {
  std::ifstream f(path);
  if (!f) {
    throw std::runtime_error("Could not open file: " + path);
  }
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

void write_text_file_atomic(const std::string& path, const std::string& content) {
  // Write to <path>.tmp then rename, so a crash mid-write cannot leave the
  // config truncated. Both files end up with the same default permissions
  // (0644 from umask) because we don't have a mechanism here to copy the
  // original mode and the production deployment runs as a single user anyway.
  std::string tmp = path + ".tmp";
  {
    std::ofstream f(tmp, std::ios::binary | std::ios::trunc);
    if (!f) {
      throw std::runtime_error("Could not open temp file for write: " + tmp);
    }
    f << content;
    if (!f) {
      throw std::runtime_error("Write failed: " + tmp);
    }
  }
  std::error_code ec;
  fs::rename(tmp, path, ec);
  if (ec) {
    throw std::runtime_error("Atomic rename failed: " + ec.message());
  }
}

json parse_config_sh(const std::string& content) {
  json obj = json::object();
  std::stringstream ss(content);
  std::string line;
  while (std::getline(ss, line)) {
    // Trim leading whitespace
    size_t start = line.find_first_not_of(" \t");
    if (start == std::string::npos) continue;
    if (line[start] == '#') continue;

    // Optionally peel off a leading "export "
    std::string body = line.substr(start);
    if (body.rfind("export ", 0) == 0) {
      body = body.substr(7);
    }

    auto eq = body.find('=');
    if (eq == std::string::npos) continue;
    std::string name = body.substr(0, eq);
    std::string value = body.substr(eq + 1);

    // Strip a trailing inline "# comment" — but only when the # is outside of
    // quotes. The simple heuristic is: if the value contains an unbalanced
    // quote, leave it alone.
    bool in_single = false, in_double = false;
    for (size_t i = 0; i < value.size(); ++i) {
      char c = value[i];
      if (c == '\'' && !in_double) in_single = !in_single;
      else if (c == '"' && !in_single) in_double = !in_double;
      else if (c == '#' && !in_single && !in_double) {
        value = value.substr(0, i);
        break;
      }
    }
    // Trim trailing whitespace
    size_t end = value.find_last_not_of(" \t\r\n");
    value = (end == std::string::npos) ? "" : value.substr(0, end + 1);

    obj[name] = unquote(value);
  }
  return obj;
}

std::string write_config_sh(const std::string& original, const json& changes) {
  // Walk the original line by line and rewrite assignments whose variable name
  // appears in `changes`. New variables (not present in the original) are
  // appended to the end. Anything else passes through verbatim.
  std::stringstream out;
  std::stringstream ss(original);
  std::string line;
  std::set<std::string> handled;

  while (std::getline(ss, line)) {
    std::string body = line;
    size_t indent_end = body.find_first_not_of(" \t");
    std::string indent = (indent_end == std::string::npos) ? "" : body.substr(0, indent_end);
    std::string trimmed = (indent_end == std::string::npos) ? "" : body.substr(indent_end);

    bool has_export = false;
    if (trimmed.rfind("export ", 0) == 0) {
      has_export = true;
      trimmed = trimmed.substr(7);
    }

    auto eq = trimmed.find('=');
    if (trimmed.empty() || trimmed[0] == '#' || eq == std::string::npos) {
      out << line << '\n';
      continue;
    }

    std::string name = trimmed.substr(0, eq);
    if (changes.contains(name)) {
      handled.insert(name);
      const auto& v = changes[name];
      std::string str_val;
      if (v.is_string()) str_val = v.get<std::string>();
      else if (v.is_boolean()) str_val = v.get<bool>() ? "True" : "False";
      else str_val = v.dump();
      out << indent << (has_export ? "export " : "") << name << "=" << shell_quote(str_val) << '\n';
    } else {
      out << line << '\n';
    }
  }

  // Append any new keys that weren't already in the file.
  bool first_new = true;
  for (auto& [name, v] : changes.items()) {
    if (handled.count(name)) continue;
    if (first_new) {
      out << "\n# Added via meta.config.set\n";
      first_new = false;
    }
    std::string str_val;
    if (v.is_string()) str_val = v.get<std::string>();
    else if (v.is_boolean()) str_val = v.get<bool>() ? "True" : "False";
    else str_val = v.dump();
    out << "export " << name << "=" << shell_quote(str_val) << '\n';
  }

  return out.str();
}

json defaults_yaml_from_schema(const json& schema) {
  // The frontend expects the result of meta.config.defaults() to be a map
  // {filename: yaml_content}. We synthesise a single defaults.yaml whose body
  // lists `OM_FOO: value` lines extracted from the schema. The frontend's
  // RELEVANT_DEFAULTS list also references boards/v1.yaml and
  // mowers/YardForce500.yaml — we ship those as empty stubs so its merge step
  // doesn't fail.
  json defaults_obj = json::object();
  collect_defaults(schema, defaults_obj);

  std::stringstream y;
  for (auto& [k, v] : defaults_obj.items()) {
    y << k << ": ";
    if (v.is_string()) {
      // YAML allows unquoted scalar strings as long as they don't start with a
      // reserved character. Quoting always is safer.
      std::string s = v.get<std::string>();
      // Use single quotes; double the embedded single quote per YAML spec.
      std::string escaped;
      escaped.reserve(s.size() + 2);
      escaped.push_back('\'');
      for (char c : s) {
        if (c == '\'') escaped.push_back('\'');
        escaped.push_back(c);
      }
      escaped.push_back('\'');
      y << escaped;
    } else if (v.is_boolean()) {
      y << (v.get<bool>() ? "true" : "false");
    } else {
      y << v.dump();
    }
    y << '\n';
  }

  json result = json::object();
  result["defaults.yaml"] = y.str();
  result["boards/v1.yaml"] = "";
  result["mowers/YardForce500.yaml"] = "";
  return result;
}

}  // namespace xbot_monitoring::config_io
