#include "yaml_io.h"

#include <yaml-cpp/yaml.h>

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;
using json = nlohmann::ordered_json;

namespace xbot_monitoring::yaml_io {

namespace {

// Convert a yaml-cpp Node tree into nlohmann::ordered_json. We preserve key
// order (ordered_json) so round-trips don't reshuffle the file unexpectedly.
json yaml_to_json(const YAML::Node& node) {
  switch (node.Type()) {
    case YAML::NodeType::Null:
      return nullptr;
    case YAML::NodeType::Scalar: {
      // yaml-cpp stores everything as string; sniff the actual type to keep
      // round-trips stable. A quoted "47.3" stays a string in YAML; an
      // unquoted 47.3 becomes a number. yaml-cpp doesn't expose the quote
      // flag directly, so we fall back to "try numeric, else string".
      const std::string& s = node.Scalar();
      if (s == "true" || s == "True" || s == "TRUE") return true;
      if (s == "false" || s == "False" || s == "FALSE") return false;
      if (s == "null" || s == "Null" || s == "NULL" || s == "~") return nullptr;
      // Try integer first.
      try {
        size_t pos = 0;
        long long iv = std::stoll(s, &pos);
        if (pos == s.size()) return iv;
      } catch (...) {}
      // Then double.
      try {
        size_t pos = 0;
        double dv = std::stod(s, &pos);
        if (pos == s.size()) return dv;
      } catch (...) {}
      return s;
    }
    case YAML::NodeType::Sequence: {
      json arr = json::array();
      for (const auto& child : node) arr.push_back(yaml_to_json(child));
      return arr;
    }
    case YAML::NodeType::Map: {
      json obj = json::object();
      for (auto it = node.begin(); it != node.end(); ++it) {
        obj[it->first.as<std::string>()] = yaml_to_json(it->second);
      }
      return obj;
    }
    case YAML::NodeType::Undefined:
    default:
      return nullptr;
  }
}

// Convert nlohmann::json back to a YAML::Emitter stream. We handle the common
// scalar types; ordered_json keeps the insertion order which keeps existing
// files visually stable across writes.
void emit_json(YAML::Emitter& out, const json& j) {
  if (j.is_null()) {
    out << YAML::Null;
  } else if (j.is_boolean()) {
    out << j.get<bool>();
  } else if (j.is_number_integer()) {
    out << j.get<long long>();
  } else if (j.is_number_float()) {
    out << j.get<double>();
  } else if (j.is_string()) {
    out << j.get<std::string>();
  } else if (j.is_array()) {
    out << YAML::BeginSeq;
    for (const auto& el : j) emit_json(out, el);
    out << YAML::EndSeq;
  } else if (j.is_object()) {
    out << YAML::BeginMap;
    for (auto it = j.begin(); it != j.end(); ++it) {
      out << YAML::Key << it.key() << YAML::Value;
      emit_json(out, it.value());
    }
    out << YAML::EndMap;
  }
}

// Split "a.b.c" into ["a", "b", "c"]. Empty input → empty vector. Trailing or
// leading dots are tolerated (treated as empty segments and dropped).
std::vector<std::string> split_path(const std::string& dotted_path) {
  std::vector<std::string> parts;
  std::string cur;
  for (char c : dotted_path) {
    if (c == '.') {
      if (!cur.empty()) parts.push_back(cur);
      cur.clear();
    } else {
      cur.push_back(c);
    }
  }
  if (!cur.empty()) parts.push_back(cur);
  return parts;
}

}  // namespace

json read_yaml_file(const std::string& path) {
  if (!fs::exists(path)) {
    // Empty user-override is valid (first run with no overrides yet).
    return json::object();
  }
  YAML::Node root;
  try {
    root = YAML::LoadFile(path);
  } catch (const YAML::Exception& e) {
    throw std::runtime_error("Could not parse YAML " + path + ": " + e.what());
  }
  if (!root || root.IsNull()) return json::object();
  return yaml_to_json(root);
}

void write_yaml_file_atomic(const std::string& path, const json& data) {
  fs::path dest = path;
  fs::path dir = dest.parent_path();
  if (!dir.empty() && !fs::exists(dir)) {
    std::error_code ec;
    fs::create_directories(dir, ec);
    if (ec) throw std::runtime_error("Could not create dir " + dir.string() + ": " + ec.message());
  }
  std::string tmp = path + ".tmp";

  YAML::Emitter out;
  out.SetIndent(2);
  out.SetMapFormat(YAML::Block);
  out.SetSeqFormat(YAML::Block);
  emit_json(out, data);
  if (!out.good()) {
    throw std::runtime_error("YAML emitter failed: " + std::string(out.GetLastError()));
  }

  {
    std::ofstream f(tmp, std::ios::trunc);
    if (!f) throw std::runtime_error("Could not open temp file for write: " + tmp);
    f << out.c_str() << "\n";
    if (!f) throw std::runtime_error("Write failed: " + tmp);
  }
  std::error_code ec;
  fs::rename(tmp, dest, ec);
  if (ec) {
    fs::remove(tmp, ec);
    throw std::runtime_error("Atomic rename failed: " + ec.message());
  }
}

const json* read_path(const json& root, const std::string& dotted_path) {
  const json* cur = &root;
  for (const auto& seg : split_path(dotted_path)) {
    if (!cur->is_object() || !cur->contains(seg)) return nullptr;
    cur = &(*cur)[seg];
  }
  return cur;
}

void write_path(json& root, const std::string& dotted_path, const json& value) {
  auto parts = split_path(dotted_path);
  if (parts.empty()) return;
  json* cur = &root;
  for (size_t i = 0; i + 1 < parts.size(); ++i) {
    if (!cur->is_object()) *cur = json::object();
    if (!cur->contains(parts[i]) || !(*cur)[parts[i]].is_object()) {
      (*cur)[parts[i]] = json::object();
    }
    cur = &(*cur)[parts[i]];
  }
  if (!cur->is_object()) *cur = json::object();
  (*cur)[parts.back()] = value;
}

void deep_merge(json& base, const json& over) {
  if (!base.is_object() || !over.is_object()) {
    base = over;
    return;
  }
  for (auto it = over.begin(); it != over.end(); ++it) {
    if (base.contains(it.key()) && base[it.key()].is_object() && it.value().is_object()) {
      deep_merge(base[it.key()], it.value());
    } else {
      base[it.key()] = it.value();
    }
  }
}

}  // namespace xbot_monitoring::yaml_io
