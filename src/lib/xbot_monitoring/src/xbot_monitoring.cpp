//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <chrono>
#include <csignal>
#include <cstdlib>
#include <deque>
#include <filesystem>
#include <functional>
#include <thread>
#include <unordered_set>

#include "config_io.h"
#include "yaml_io.h"
#include "ros/ros.h"
#include "rosgraph_msgs/Log.h"
#include <ros/package.h>
#include <ros/this_node.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <memory>
#include <boost/regex.hpp>
#include "xbot_msgs/SensorInfo.h"
#include "xbot_msgs/SensorDataString.h"
#include "xbot_msgs/SensorDataDouble.h"
#include "xbot_msgs/RobotState.h"
#include <mqtt/async_client.h>
#include <nlohmann/json.hpp>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Path.h"
#include "std_msgs/String.h"
#include "visualization_msgs/MarkerArray.h"
#include "xbot_msgs/RegisterActionsSrv.h"
#include "xbot_msgs/ActionInfo.h"
#include "xbot_msgs/MapOverlay.h"
#include "xbot_rpc/RpcError.h"
#include "xbot_rpc/RpcRequest.h"
#include "xbot_rpc/RpcResponse.h"
#include "xbot_rpc/constants.h"
#include "xbot_rpc/provider.h"
#include "xbot_rpc/RegisterMethodsSrv.h"
#include "capabilities.h"

using json = nlohmann::ordered_json;

void publish_capabilities();
void publish_sensor_metadata();
void publish_map();
void publish_map_overlay();
void publish_actions();
void publish_version();
void publish_params();
void rpc_request_callback(const std::string &payload);

// Stores registered actions (prefix to vector<action>)
std::map<std::string, std::vector<xbot_msgs::ActionInfo>> registered_actions;
std::mutex registered_actions_mutex;

// Stores registered RPC methods
std::map<std::string, std::vector<std::string>> registered_methods;
std::mutex registered_methods_mutex;

std::map<std::string, xbot_msgs::SensorInfo> found_sensors;
std::mutex found_sensors_mutex;

ros::NodeHandle *n;

// The MQTT Client
std::shared_ptr<mqtt::async_client> client_;
std::shared_ptr<mqtt::async_client> client_external_;


// Publisher for cmd_vel and commands
ros::Publisher cmd_vel_pub;
ros::Publisher action_pub;
ros::Publisher rpc_request_pub;

// properties for external mqtt
bool external_mqtt_enable = false;
std::string external_mqtt_username = "";
std::string external_mqtt_password = "";
std::string external_mqtt_hostname = "";
std::string external_mqtt_topic_prefix = "";
std::string external_mqtt_port = "";
std::string version_string = "";

class MqttCallback : public mqtt::callback {

    void connected(const mqtt::string &string) override {
        ROS_INFO_STREAM("MQTT Connected");
        publish_capabilities();
        publish_sensor_metadata();
        publish_map();
        publish_map_overlay();
        publish_actions();
        publish_version();
        publish_params();

        // BEGIN: Deprecated code (1/2)
        // Earlier implementations subscribed to "/action" and "prefix//action" topics, we do it to not break stuff as well.
        client_->subscribe(this->mqtt_topic_prefix + "/teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "/action", 0);
        // END: Deprecated code (1/2)

        client_->subscribe(this->mqtt_topic_prefix + "teleop", 0);
        client_->subscribe(this->mqtt_topic_prefix + "command", 0);
        client_->subscribe(this->mqtt_topic_prefix + "action", 0);
        client_->subscribe(this->mqtt_topic_prefix + "rpc/request", 0);
    }

public:
    void setMqttClient(std::shared_ptr<mqtt::async_client> c, const std::string &mqtt_topic_prefix) {
        this->client_ = std::move(c);
        this->mqtt_topic_prefix = mqtt_topic_prefix;
    }
    void message_arrived(mqtt::const_message_ptr ptr) override {
        if(ptr->get_topic() == this->mqtt_topic_prefix + "teleop") {
            try {
                json json = json::from_bson(ptr->get_payload().begin(), ptr->get_payload().end());
                geometry_msgs::Twist t;
                t.linear.x = json["vx"];
                t.angular.z = json["vz"];
                cmd_vel_pub.publish(t);
            } catch (const json::exception &e) {
                ROS_ERROR_STREAM("Error decoding teleop bson: " << e.what());
            }
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "action") {
            ROS_INFO_STREAM("Got action: " + ptr->get_payload());
            std_msgs::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub.publish(action_msg);
        } else if(ptr->get_topic() == this->mqtt_topic_prefix + "/action") {
            // BEGIN: Deprecated code (2/2)
            ROS_WARN_STREAM("Got action on deprecated topic! Change your topic names!: " + ptr->get_payload());
            std_msgs::String action_msg;
            action_msg.data = ptr->get_payload_str();
            action_pub.publish(action_msg);
            // END: Deprecated code (2/2)
        } else if (ptr->get_topic() == this->mqtt_topic_prefix + "rpc/request") {
          std::string payload = ptr->get_payload_str();
          rpc_request_callback(payload);
        }
    }
private:
    std::shared_ptr<mqtt::async_client> client_;
    std::string mqtt_topic_prefix = "";
};

MqttCallback mqtt_callback;
MqttCallback mqtt_callback_external;

json map;
std::mutex map_mutex;
json map_overlay;
std::mutex map_overlay_mutex;
bool has_map = false;
bool has_map_overlay = false;

// Resolve the path to mower_config.sh. Allow override via the ~mower_config_path
// param so packagers and tests can point us at a sandboxed copy. Default is the
// path that the OpenMowerOS images bake into the home directory of the ROS
// container.
static std::string get_mower_config_path() {
    std::string path;
    ros::param::param<std::string>("~mower_config_path", path, std::string(getenv("HOME") ? getenv("HOME") : "/root") + "/mower_config.sh");
    return path;
}

// Resolve the path to mower_config.schema.json. The launch file is expected
// to set ~mower_config_schema_path to the absolute path inside the deployed
// container; we fall back to a path next to the open_mower package for dev
// builds.
static std::string get_mower_config_schema_path() {
    std::string path;
    if (ros::param::get("~mower_config_schema_path", path) && !path.empty()) {
        return path;
    }
    std::string pkg = ros::package::getPath("open_mower");
    if (!pkg.empty()) {
        // open_mower/config/mower_config.schema.json (in source layouts the
        // file lives at the repo's top-level config/, in install layouts
        // share/open_mower/config/).
        std::string candidate = pkg + "/config/mower_config.schema.json";
        if (std::filesystem::exists(candidate)) return candidate;
    }
    // As a last resort, look at the repo top-level path. This works for the
    // dev workspace where source is laid out as expected.
    return "/root/open_mower_ros/config/mower_config.schema.json";
}

// Cache of the parsed schema; loaded lazily to avoid I/O on every RPC call.
static std::mutex schema_cache_mutex;
static json schema_cache;
static bool schema_cache_loaded = false;

static json& load_schema_cached() {
    std::lock_guard<std::mutex> lk(schema_cache_mutex);
    if (!schema_cache_loaded) {
        std::string raw = xbot_monitoring::config_io::read_text_file(get_mower_config_schema_path());
        schema_cache = json::parse(raw);
        schema_cache_loaded = true;
    }
    return schema_cache;
}

// ----- YAML-bridge helpers -------------------------------------------------
// OpenMowerOS v2 stores config in three YAML layers:
//   1) <pkg>/params/openmower_defaults_v2.yaml          (image-baked defaults)
//   2) <pkg>/params/hardware_specific/<MOWER>/params_v2.yaml  (image-baked HW)
//   3) <PARAMS_PATH>/mower_params.yaml                  (host-mounted user)
// The legacy schema (mower_config.schema.json) speaks OM_*; a separate JSON
// mapping file translates each OM_* key to the YAML dotted path it lives
// under. meta.config.get reads-merged + reverse-mapped; meta.config.set
// writes only into the user layer (the only writable one) and pushes the
// value into ros::param so live nodes see it without restart.

// Resolve user-override YAML path (writable). Default matches Compose mount.
static std::string get_user_yaml_path() {
    std::string path;
    ros::param::param<std::string>("~user_yaml_path", path, std::string("/data/params/mower_params.yaml"));
    return path;
}

// Resolve image-baked defaults YAML.
static std::string get_defaults_yaml_path() {
    std::string path;
    if (ros::param::get("~defaults_yaml_path", path) && !path.empty()) {
        return path;
    }
    std::string pkg = ros::package::getPath("open_mower");
    if (!pkg.empty()) {
        std::string candidate = pkg + "/params/openmower_defaults_v2.yaml";
        if (std::filesystem::exists(candidate)) return candidate;
    }
    return "/opt/open_mower_ros/src/open_mower/params/openmower_defaults_v2.yaml";
}

// Resolve OM_* → YAML mapping JSON file.
static std::string get_yaml_mapping_path() {
    std::string path;
    if (ros::param::get("~yaml_mapping_path", path) && !path.empty()) {
        return path;
    }
    std::string pkg = ros::package::getPath("open_mower");
    if (!pkg.empty()) {
        std::string candidate = pkg + "/../../config/mower_config.yaml_mapping.json";
        if (std::filesystem::exists(candidate)) return candidate;
    }
    return "/opt/open_mower_ros/config/mower_config.yaml_mapping.json";
}

// Cache of the OM_* → YAML mapping; lazily loaded.
static std::mutex mapping_cache_mutex;
static json mapping_cache;
static bool mapping_cache_loaded = false;

static const json& load_mapping_cached() {
    std::lock_guard<std::mutex> lk(mapping_cache_mutex);
    if (!mapping_cache_loaded) {
        std::string raw = xbot_monitoring::config_io::read_text_file(get_yaml_mapping_path());
        mapping_cache = json::parse(raw);
        mapping_cache_loaded = true;
    }
    return mapping_cache;
}

// Read all three YAML layers (image-baked defaults, hardware-specific
// defaults selected by the MOWER env var, host-mounted user override) and
// deep-merge them so the frontend sees the same effective values the running
// stack does. Falling back gracefully if hardware layer is missing.
static json read_merged_yaml_config() {
    json merged;
    try {
        merged = xbot_monitoring::yaml_io::read_yaml_file(get_defaults_yaml_path());
    } catch (...) {
        merged = json::object();
    }
    // Hardware-specific defaults: <pkg>/params/hardware_specific/<MOWER>/params_v2.yaml.
    const char* mower = std::getenv("MOWER");
    if (mower && *mower) {
        std::string pkg = ros::package::getPath("open_mower");
        if (!pkg.empty()) {
            std::string hw_path = pkg + "/params/hardware_specific/" + mower + "/params_v2.yaml";
            if (std::filesystem::exists(hw_path)) {
                try {
                    json hw = xbot_monitoring::yaml_io::read_yaml_file(hw_path);
                    xbot_monitoring::yaml_io::deep_merge(merged, hw);
                } catch (const std::exception& e) {
                    ROS_WARN_STREAM("meta.config.get: failed to read hw layer " << hw_path << ": " << e.what());
                }
            }
        }
    }
    try {
        json user_yaml = xbot_monitoring::yaml_io::read_yaml_file(get_user_yaml_path());
        xbot_monitoring::yaml_io::deep_merge(merged, user_yaml);
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("meta.config.get: failed to read user layer: " << e.what());
    }
    return merged;
}

// Coerce a JSON value (which may have arrived as a string from the frontend)
// to the type the schema expects for a given OM_* key. Frontend forms commonly
// emit numbers as strings; YAML stores them as numbers. Without this we'd
// flip "47.301" (number) into "47.301" (string) on first save and break
// downstream consumers.
static json coerce_value_for_schema(const std::string& om_key, const json& value) {
    // Walk schema looking for the property whose x-environment-variable matches.
    // Schema is small (~700 lines) and the recursion depth is shallow, so a
    // simple DFS is fine; we don't bother caching per-key.
    std::function<const json*(const json&)> find_prop = [&](const json& node) -> const json* {
        if (!node.is_object()) return nullptr;
        if (node.contains("x-environment-variable") && node["x-environment-variable"].is_string() &&
            node["x-environment-variable"].get<std::string>() == om_key) {
            return &node;
        }
        if (node.contains("properties") && node["properties"].is_object()) {
            for (auto it = node["properties"].begin(); it != node["properties"].end(); ++it) {
                if (auto p = find_prop(it.value())) return p;
            }
        }
        for (const char* branch : {"allOf", "anyOf", "oneOf"}) {
            if (node.contains(branch) && node[branch].is_array()) {
                for (const auto& sub : node[branch]) {
                    if (sub.is_object()) {
                        if (sub.contains("then") && sub["then"].is_object()) {
                            if (auto p = find_prop(sub["then"])) return p;
                        }
                        if (sub.contains("else") && sub["else"].is_object()) {
                            if (auto p = find_prop(sub["else"])) return p;
                        }
                        if (auto p = find_prop(sub)) return p;
                    }
                }
            }
        }
        return nullptr;
    };

    const json* prop = nullptr;
    try {
        prop = find_prop(load_schema_cached());
    } catch (...) {
        // Schema unavailable — pass the value through as-is.
        return value;
    }
    if (!prop || !prop->contains("type") || !(*prop)["type"].is_string()) return value;
    std::string type = (*prop)["type"].get<std::string>();

    if (value.is_string()) {
        const std::string& s = value.get<std::string>();
        if (type == "number") {
            try { return std::stod(s); } catch (...) { return value; }
        } else if (type == "integer") {
            try { return std::stoll(s); } catch (...) { return value; }
        } else if (type == "boolean") {
            if (s == "true" || s == "True" || s == "1") return true;
            if (s == "false" || s == "False" || s == "0") return false;
        }
    }
    return value;
}

// Push a single value into ros::param so live nodes see the change without
// a restart. YAML path "ll.services.gps.datum_lat" → ROS param
// "/ll/services/gps/datum_lat". Falls silent on type errors — caller logs.
static void apply_to_ros_param(const std::string& yaml_path, const json& value) {
    if (yaml_path.empty()) return;
    std::string ros_path = "/" + yaml_path;
    for (char& c : ros_path) if (c == '.') c = '/';

    if (value.is_boolean()) ros::param::set(ros_path, value.get<bool>());
    else if (value.is_number_integer()) ros::param::set(ros_path, value.get<int>());
    else if (value.is_number_float()) ros::param::set(ros_path, value.get<double>());
    else if (value.is_string()) ros::param::set(ros_path, value.get<std::string>());
}

// Ring buffer for /rosout_agg, populated by rosout_callback. logs.tail RPC
// reads from here. Capacity is generous (~5000) so that a 1000-line query
// always finds enough history without keeping the journal forever.
struct LogEntry {
    double ts;            // header.stamp seconds (epoch).
    std::string level;    // "debug" | "info" | "warn" | "error" | "fatal"
    std::string source;   // node name (rosgraph_msgs/Log::name)
    std::string msg;
};
constexpr size_t LOG_BUFFER_CAPACITY = 5000;
std::deque<LogEntry> log_buffer;
std::mutex log_buffer_mutex;

static std::string rosout_level_to_string(int8_t level) {
    // rosgraph_msgs/Log levels are bit flags: 1=DEBUG, 2=INFO, 4=WARN, 8=ERROR, 16=FATAL.
    switch (level) {
        case rosgraph_msgs::Log::DEBUG: return "debug";
        case rosgraph_msgs::Log::INFO:  return "info";
        case rosgraph_msgs::Log::WARN:  return "warn";
        case rosgraph_msgs::Log::ERROR: return "error";
        case rosgraph_msgs::Log::FATAL: return "fatal";
        default: return "info";
    }
}

// Bridge: mower_sessions_recorder publishes the JSON sessions list as a
// std_msgs/String on `xbot_monitoring/mowing_sessions`; we rebroadcast it
// retained on MQTT for the openmower-app /statistics page.
void mowing_sessions_callback(const std_msgs::String::ConstPtr &msg) {
    try_publish("mowing_sessions/json", msg->data, true);
}

void rosout_callback(const rosgraph_msgs::Log::ConstPtr &msg) {
    LogEntry e;
    e.ts = msg->header.stamp.toSec();
    e.level = rosout_level_to_string(msg->level);
    e.source = msg->name;
    e.msg = msg->msg;
    std::lock_guard<std::mutex> lk(log_buffer_mutex);
    log_buffer.push_back(std::move(e));
    if (log_buffer.size() > LOG_BUFFER_CAPACITY) {
        log_buffer.pop_front();
    }
}

// Whitelist: keys are accepted RPC source filters, values are substring
// patterns matched against rosgraph_msgs/Log::name (with leading slash).
// "all" disables filtering.
static const std::unordered_set<std::string> kLogsTailSources = {
    "all", "mower_logic", "xbot_monitoring", "mower_scheduler",
    "move_base_flex", "map_service", "slic3r_coverage_planner"
};

// Allow-list for system.restart_service. Restarting xbot_monitoring restarts
// the whole container (PID 1), which restarts roslaunch and therefore every
// node — cleaner than fighting systemd from inside a non-privileged container.
static const std::unordered_set<std::string> kRestartServices = {
    "openmower", "mower_logic", "xbot_monitoring", "move_base_flex"
};

xbot_rpc::RpcProvider rpc_provider("xbot_monitoring", {{
    RPC_METHOD("rpc.ping", {
        return "pong";
    }),
    RPC_METHOD("rpc.methods", {
        std::lock_guard<std::mutex> lk(registered_methods_mutex);
        json methods = json::array();
        for (const auto& [_, method_ids] : registered_methods) {
            for (const auto& method_id : method_ids) {
                methods.push_back(method_id);
            }
        }
        std::sort(methods.begin(), methods.end());
        return methods;
    }),
    RPC_METHOD("meta.rpc.ping", {
        return "pong";
    }),
    RPC_METHOD("meta.config.schema", {
        // Frontend expects the schema as a JSON string (it parses it itself
        // because @json-schema-tools/dereferencer wants raw input).
        try {
            return load_schema_cached().dump();
        } catch (const std::exception& e) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          std::string("Failed to load schema: ") + e.what());
        }
    }),
    RPC_METHOD("meta.config.defaults", {
        // Frontend expects {filename: yaml_content}. We synthesise a single
        // defaults.yaml from the schema; boards/ and mowers/ stubs are empty.
        try {
            return xbot_monitoring::config_io::defaults_yaml_from_schema(load_schema_cached());
        } catch (const std::exception& e) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          std::string("Failed to build defaults: ") + e.what());
        }
    }),
    RPC_METHOD("meta.config.get", {
        // Reads the merged YAML config (defaults + user-override) and projects
        // it back into the OM_*-keyed map the frontend expects via the YAML
        // mapping JSON. Keys mapped to null (read-only hardware identifiers)
        // are skipped silently.
        try {
            const json& mapping = load_mapping_cached();
            json merged = read_merged_yaml_config();
            json out = json::object();
            for (auto it = mapping.begin(); it != mapping.end(); ++it) {
                if (it.key().rfind("_", 0) == 0) continue;  // skip _comment etc.
                if (!it.value().is_string()) continue;       // null mapping → not editable
                const std::string yaml_path = it.value().get<std::string>();
                const json* v = xbot_monitoring::yaml_io::read_path(merged, yaml_path);
                if (v) out[it.key()] = *v;
            }
            return out;
        } catch (const std::exception& e) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          std::string("Failed to read config: ") + e.what());
        }
    }),
    RPC_METHOD("meta.config.set", {
        // Accept either {changes: {...}} (named) or [{...}] (positional).
        json changes;
        if (params.is_object() && params.contains("changes") && params["changes"].is_object()) {
            changes = params["changes"];
        } else if (params.is_array() && params.size() == 1 && params[0].is_object()) {
            changes = params[0];
        } else if (params.is_object()) {
            changes = params;
        } else {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS,
                                          "Expected an object of {key: value} changes");
        }
        try {
            const json& mapping = load_mapping_cached();
            const std::string user_path = get_user_yaml_path();
            json user_yaml = xbot_monitoring::yaml_io::read_yaml_file(user_path);

            json updated_keys = json::array();
            json skipped_keys = json::array();
            for (auto it = changes.begin(); it != changes.end(); ++it) {
                const std::string& om_key = it.key();
                if (!mapping.contains(om_key)) {
                    skipped_keys.push_back(om_key);
                    continue;
                }
                const auto& mapped = mapping.at(om_key);
                if (!mapped.is_string()) {
                    // null mapping = read-only or no YAML target.
                    skipped_keys.push_back(om_key);
                    continue;
                }
                const std::string yaml_path = mapped.get<std::string>();
                json coerced = coerce_value_for_schema(om_key, it.value());
                xbot_monitoring::yaml_io::write_path(user_yaml, yaml_path, coerced);
                apply_to_ros_param(yaml_path, coerced);
                updated_keys.push_back(om_key);
            }

            xbot_monitoring::yaml_io::write_yaml_file_atomic(user_path, user_yaml);
            ROS_INFO_STREAM("meta.config.set wrote " << updated_keys.size()
                             << " change(s) to " << user_path
                             << " (skipped " << skipped_keys.size() << " unmapped)");
            return json::object({{"updated", updated_keys.size()},
                                 {"updated_keys", updated_keys},
                                 {"skipped_keys", skipped_keys}});
        } catch (const std::exception& e) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          std::string("Failed to write config: ") + e.what());
        }
    }),
    RPC_METHOD("params.set", {
        // params = {name: string, value: <any>}. We always ros::param::set;
        // additionally trigger dynamic_reconfigure if the parameter name is
        // <node>/<key> and that node exposes /set_parameters. Unknown
        // composites silently fall back to a static set.
        if (!params.is_object() || !params.contains("name") || !params["name"].is_string()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS, "Missing name");
        }
        if (!params.contains("value")) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS, "Missing value");
        }
        const std::string name = params["name"];
        const auto& v = params["value"];
        if (v.is_boolean()) ros::param::set(name, v.get<bool>());
        else if (v.is_number_integer()) ros::param::set(name, v.get<int>());
        else if (v.is_number_float()) ros::param::set(name, v.get<double>());
        else if (v.is_string()) ros::param::set(name, v.get<std::string>());
        else throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS, "Unsupported value type");

        // Best-effort dynamic_reconfigure. Owner node is derived from the
        // first path segment (e.g. /mower_logic/foo → /mower_logic). If the
        // service isn't there we just return success — the static set is
        // already committed.
        size_t slash1 = name.find('/', name[0] == '/' ? 1 : 0);
        size_t slash2 = name.find('/', slash1 + 1);
        if (slash1 != std::string::npos && slash2 != std::string::npos) {
            std::string node_ns = name.substr(0, slash2);
            std::string key = name.substr(slash2 + 1);
            std::string svc = node_ns + "/set_parameters";
            if (ros::service::exists(svc, false)) {
                dynamic_reconfigure::Reconfigure srv;
                if (v.is_boolean()) {
                    dynamic_reconfigure::BoolParameter p;
                    p.name = key; p.value = v.get<bool>();
                    srv.request.config.bools.push_back(p);
                } else if (v.is_number_integer()) {
                    dynamic_reconfigure::IntParameter p;
                    p.name = key; p.value = v.get<int>();
                    srv.request.config.ints.push_back(p);
                } else if (v.is_number_float()) {
                    dynamic_reconfigure::DoubleParameter p;
                    p.name = key; p.value = v.get<double>();
                    srv.request.config.doubles.push_back(p);
                } else if (v.is_string()) {
                    dynamic_reconfigure::StrParameter p;
                    p.name = key; p.value = v.get<std::string>();
                    srv.request.config.strs.push_back(p);
                }
                ros::service::call(svc, srv);
            }
        }
        return nullptr;
    }),
    RPC_METHOD("logs.tail", {
        // Params: { source?: string, lines?: number }. Source is whitelisted
        // against kLogsTailSources; lines clamped to [50, 5000], default 200.
        std::string source = "all";
        int lines = 200;
        if (params.is_object()) {
            if (params.contains("source") && params["source"].is_string()) {
                source = params["source"].get<std::string>();
            }
            if (params.contains("lines") && params["lines"].is_number_integer()) {
                lines = params["lines"].get<int>();
            }
        }
        if (kLogsTailSources.find(source) == kLogsTailSources.end()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS,
                                          "Unknown source; allowed: all, mower_logic, "
                                          "xbot_monitoring, mower_scheduler, "
                                          "move_base_flex, map_service, slic3r_coverage_planner");
        }
        if (lines < 50) lines = 50;
        if (lines > 5000) lines = 5000;

        json entries = json::array();
        std::lock_guard<std::mutex> lk(log_buffer_mutex);
        // Walk backwards collecting up to `lines` matching entries, then reverse.
        std::vector<const LogEntry*> picked;
        picked.reserve(static_cast<size_t>(lines));
        for (auto it = log_buffer.rbegin(); it != log_buffer.rend() && static_cast<int>(picked.size()) < lines; ++it) {
            if (source != "all") {
                // Match either a leading-slash node ("/mower_logic") or
                // unprefixed ("mower_logic"). Substring is enough because
                // we restrict via the whitelist above.
                if (it->source.find(source) == std::string::npos) continue;
            }
            picked.push_back(&(*it));
        }
        for (auto rit = picked.rbegin(); rit != picked.rend(); ++rit) {
            const LogEntry *e = *rit;
            json je;
            je["ts"] = e->ts;
            je["level"] = e->level;
            je["source"] = e->source;
            je["msg"] = e->msg;
            entries.push_back(std::move(je));
        }
        return json::object({{"entries", entries}});
    }),
    RPC_METHOD("system.restart_service", {
        // Params: { service?: string }. Default 'openmower' restarts the
        // container (== whole stack). Always responds with {ok: true} BEFORE
        // killing the process, otherwise the caller never sees the response.
        std::string service = "openmower";
        if (params.is_object() && params.contains("service") && params["service"].is_string()) {
            service = params["service"].get<std::string>();
        }
        if (kRestartServices.find(service) == kRestartServices.end()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS,
                                          "Unknown service; allowed: openmower, "
                                          "mower_logic, xbot_monitoring, move_base_flex");
        }
        ROS_WARN_STREAM("system.restart_service requested for '" << service
                        << "' — terminating container (PID 1).");
        // Defer the kill so that the RPC response is written to MQTT first.
        // Docker's restart-policy (unless-stopped in the OMOSv2 compose) will
        // bring us back; per-node restarts within the container are not
        // supported here — every option restarts the whole container.
        std::thread([]() {
            std::this_thread::sleep_for(std::chrono::milliseconds(500));
            std::raise(SIGTERM);
        }).detach();
        return json::object({{"ok", true}});
    }),
}});

void setupMqttClient() {
    // setup mqtt client for app use
    {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + "127.0.0.1" +
                          std::string(":") + std::to_string(1883);

        try {
            client_ = std::make_shared<mqtt::async_client>(
                    uri, "xbot_monitoring");
            mqtt_callback.setMqttClient(client_, "");
            client_->set_callback(mqtt_callback);

            client_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            ROS_ERROR("Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
    // setup external mqtt client
    if(external_mqtt_enable) {
        // MQTT connection options
        mqtt::connect_options connect_options_;

        // basic client connection options
        connect_options_.set_automatic_reconnect(true);
        connect_options_.set_clean_session(true);
        connect_options_.set_keep_alive_interval(1000);
        connect_options_.set_max_inflight(10);

        if(!external_mqtt_username.empty()) {
            connect_options_.set_user_name(external_mqtt_username);
            connect_options_.set_password(external_mqtt_password);
        }

        // create MQTT client
        std::string uri = "tcp" + std::string("://") + external_mqtt_hostname +
                          std::string(":") + external_mqtt_port;

        try {
            client_external_ = std::make_shared<mqtt::async_client>(
                    uri, "ext_xbot_monitoring");
            mqtt_callback_external.setMqttClient(client_external_, external_mqtt_topic_prefix);
            client_external_->set_callback(mqtt_callback_external);

            client_external_->connect(connect_options_);

        } catch (const mqtt::exception &e) {
            ROS_ERROR("External Client could not be initialized: %s", e.what());
            exit(EXIT_FAILURE);
        }
    }
}

void try_publish(std::string topic, std::string data, bool retain = false) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, 1, true);
        } else {
            client_->publish(topic, data);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
    // publish external
    if(external_mqtt_enable) {
        try {
            if (retain) {
                // QOS 1 so that the data actually arrives at the client at least once.
                client_external_->publish(external_mqtt_topic_prefix + topic, data, 1, true);
            } else {
                client_external_->publish(external_mqtt_topic_prefix + topic, data);
            }
        } catch (const mqtt::exception &e) {
            // client disconnected or something, we drop it.
        }
    }
}

void try_publish_binary(std::string topic, const void *data, size_t size, bool retain = false) {
    try {
        if (retain) {
            // QOS 1 so that the data actually arrives at the client at least once.
            client_->publish(topic, data, size, 1, true);
        } else {
            client_->publish(topic, data, size);
        }
    } catch (const mqtt::exception &e) {
        // client disconnected or something, we drop it.
    }
}

void publish_version() {
    json version = {
            {"version", version_string}
    };
    try_publish("version/json", version.dump(), true);
    auto bson = json::to_bson(version);
    try_publish_binary("version", bson.data(), bson.size(), true);
}

void publish_capabilities() {
  try_publish("capabilities/json", CAPABILITIES.dump(2), true);
}

#pragma GCC diagnostic push
#pragma GCC diagnostic warning "-Wswitch-enum"
json xmlrpc_to_json(XmlRpc::XmlRpcValue value) {
    switch (value.getType()) {
        case XmlRpc::XmlRpcValue::TypeBoolean:
            return static_cast<bool>(value);
        case XmlRpc::XmlRpcValue::TypeInt:
            return static_cast<int>(value);
        case XmlRpc::XmlRpcValue::TypeDouble:
            return static_cast<double>(value);
        case XmlRpc::XmlRpcValue::TypeString:
            return static_cast<std::string>(value);
        case XmlRpc::XmlRpcValue::TypeArray: {
            json arr = json::array();
            for (int i = 0; i < value.size(); ++i)
                arr.push_back(xmlrpc_to_json(value[i]));
            return arr;
        }
        case XmlRpc::XmlRpcValue::TypeStruct: {
            json obj = json::object();
            for (auto it = value.begin(); it != value.end(); ++it)
                obj[it->first] = xmlrpc_to_json(it->second);
            return obj;
        }
        case XmlRpc::XmlRpcValue::TypeDateTime: {
            const struct tm& t = static_cast<const struct tm&>(value);
            char buf[32];
            std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%S", &t);
            return std::string(buf);
        }
        case XmlRpc::XmlRpcValue::TypeBase64: {
            const XmlRpc::XmlRpcValue::BinaryData& data = static_cast<const XmlRpc::XmlRpcValue::BinaryData&>(value);
            static const char* b64 = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
            std::string out;
            out.reserve(((data.size() + 2) / 3) * 4);
            for (size_t i = 0; i < data.size(); i += 3) {
                unsigned int n = (static_cast<unsigned char>(data[i]) << 16)
                    | (i + 1 < data.size() ? static_cast<unsigned char>(data[i + 1]) << 8 : 0)
                    | (i + 2 < data.size() ? static_cast<unsigned char>(data[i + 2]) : 0);
                out += b64[(n >> 18) & 0x3F];
                out += b64[(n >> 12) & 0x3F];
                out += (i + 1 < data.size()) ? b64[(n >> 6) & 0x3F] : '=';
                out += (i + 2 < data.size()) ? b64[n & 0x3F] : '=';
            }
            return out;
        }
        case XmlRpc::XmlRpcValue::TypeInvalid:
            return nullptr;
    }
    return nullptr;
}
#pragma GCC diagnostic pop

void publish_params() {
    std::vector<std::string> param_names;
    ros::param::getParamNames(param_names);
    std::sort(param_names.begin(), param_names.end());

    json params = json::object();
    for (const auto &name : param_names) {
        if (name.find("password") != std::string::npos) {
            params[name] = nullptr;
            continue;
        }
        XmlRpc::XmlRpcValue value;
        if (ros::param::get(name, value)) {
            params[name] = xmlrpc_to_json(value);
        }
    }
    try_publish("params/json", params.dump(), true);
}

void publish_sensor_metadata() {
    json sensor_info;
    {
        std::unique_lock<std::mutex> lk(found_sensors_mutex);

        if(found_sensors.empty())
            return;

        for (const auto &kv: found_sensors) {
            json info;
            info["sensor_id"] = kv.second.sensor_id;
            info["sensor_name"] = kv.second.sensor_name;

            switch (kv.second.value_type) {
                case xbot_msgs::SensorInfo::TYPE_STRING: {
                    info["value_type"] = "STRING";
                    break;
                }
                case xbot_msgs::SensorInfo::TYPE_DOUBLE: {
                    info["value_type"] = "DOUBLE";
                    break;
                }
                default: {
                    info["value_type"] = "UNKNOWN";
                    break;
                }


            }

            switch (kv.second.value_description) {
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_TEMPERATURE: {
                    info["value_description"] = "TEMPERATURE";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VELOCITY: {
                    info["value_description"] = "VELOCITY";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_ACCELERATION: {
                    info["value_description"] = "ACCELERATION";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_VOLTAGE: {
                    info["value_description"] = "VOLTAGE";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_CURRENT: {
                    info["value_description"] = "CURRENT";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_PERCENT: {
                    info["value_description"] = "PERCENT";
                    break;
                }
                case xbot_msgs::SensorInfo::VALUE_DESCRIPTION_RPM: {
                    info["value_description"] = "REVOLUTIONS";
                    break;
                }
                default: {
                    info["value_description"] = "UNKNOWN";
                    break;
                }
            }

            info["unit"] = kv.second.unit;
            info["has_min_max"] = kv.second.has_min_max;
            info["min_value"] = kv.second.min_value;
            info["max_value"] = kv.second.max_value;
            info["has_critical_low"] = kv.second.has_critical_low;
            info["lower_critical_value"] = kv.second.lower_critical_value;
            info["has_critical_high"] = kv.second.has_critical_high;
            info["upper_critical_value"] = kv.second.upper_critical_value;
            sensor_info.push_back(info);
        }
    }
    try_publish("sensor_infos/json", sensor_info.dump(), true);
    json data;
    data["d"] = sensor_info;
    auto bson = json::to_bson(data);
    try_publish_binary("sensor_infos/bson", bson.data(), bson.size(), true);
}

void subscribe_to_sensor(std::string topic, std::vector<ros::Subscriber> &sensor_data_subscribers) {
    xbot_msgs::SensorInfo sensor;
    {
        std::unique_lock<std::mutex> lk(found_sensors_mutex);
        sensor = found_sensors[topic];
    }

    ROS_INFO_STREAM("Subscribing to sensor data for sensor with name: " << sensor.sensor_name);

    std::string data_topic = "xbot_monitoring/sensors/" + sensor.sensor_id + "/data";

    switch (sensor.value_type) {
        case xbot_msgs::SensorInfo::TYPE_DOUBLE: {
            ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataDouble>(data_topic, 10, [info = sensor](
                    const xbot_msgs::SensorDataDouble::ConstPtr &msg) {
                try_publish("sensors/" + info.sensor_id + "/data", std::to_string(msg->data));

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        case xbot_msgs::SensorInfo::TYPE_STRING: {
            ros::Subscriber s = n->subscribe<xbot_msgs::SensorDataString>(data_topic, 10, [info = sensor](
                    const xbot_msgs::SensorDataString::ConstPtr &msg) {
                try_publish("sensors/" + info.sensor_id + "/data", msg->data);

                json data;
                data["d"] = msg->data;
                auto bson = json::to_bson(data);
                try_publish_binary("sensors/" + info.sensor_id + "/bson", bson.data(), bson.size());
            });
            sensor_data_subscribers.push_back(s);
            break;
        }
        default: {
            ROS_ERROR_STREAM("Invalid Sensor Data Type: " << (int) sensor.value_type);
        }
    }
}

void robot_state_callback(const xbot_msgs::RobotState::ConstPtr &msg) {
    // Build a JSON and publish it
    json j;

    j["battery_percentage"] = msg->battery_percentage;
    j["gps_percentage"] = msg->gps_percentage;
    j["current_action_progress"] = msg->current_action_progress;
    j["current_state"] = msg->current_state;
    j["current_sub_state"] = msg->current_sub_state;
    j["current_area"] = msg->current_area;
    j["current_path"] = msg->current_path;
    j["current_path_index"] = msg->current_path_index;
    j["emergency"] = msg->emergency;
    j["is_charging"] = msg->is_charging;
    j["rain_detected"] = msg->rain_detected;
    j["pose"]["x"] = msg->robot_pose.pose.pose.position.x;
    j["pose"]["y"] = msg->robot_pose.pose.pose.position.y;
    j["pose"]["heading"] = msg->robot_pose.vehicle_heading;
    j["pose"]["pos_accuracy"] = msg->robot_pose.position_accuracy;
    j["pose"]["heading_accuracy"] = msg->robot_pose.orientation_accuracy;
    j["pose"]["heading_valid"] = msg->robot_pose.orientation_valid;

    try_publish("robot_state/json", j.dump());
    json data;
    data["d"] = j;
    auto bson = json::to_bson(data);
    try_publish_binary("robot_state/bson", bson.data(), bson.size());
}

void publish_actions() {
    json actions = json::array();
    {
        std::lock_guard<std::mutex> lk(registered_actions_mutex);
        for(const auto &kv : registered_actions) {
            for(const auto &action : kv.second) {
                json action_info;
                action_info["action_id"] = kv.first + "/" + action.action_id;
                action_info["action_name"] = action.action_name;
                action_info["enabled"] = action.enabled;
                actions.push_back(action_info);
            }
        }
    }

    try_publish("actions/json", actions.dump(), true);
    json data;
    data["d"] = actions;

    auto bson = json::to_bson(data);
    try_publish_binary("actions/bson", bson.data(), bson.size(), true);
}

void publish_map() {
    json m;
    {
        std::lock_guard<std::mutex> lk(map_mutex);
        if(!has_map)
            return;
        m = map;
    }
    try_publish("map/json", m.dump(2), true);
    json data;
    data["d"] = m;
    auto bson = json::to_bson(data);
    try_publish_binary("map/bson", bson.data(), bson.size(), true);
}

void publish_map_overlay() {
    json m;
    {
        std::lock_guard<std::mutex> lk(map_overlay_mutex);
        if(!has_map_overlay)
            return;
        m = map_overlay;
    }
    try_publish("map_overlay/json", m.dump(), true);
    json data;
    data["d"] = m;
    auto bson = json::to_bson(data);
    try_publish_binary("map_overlay/bson", bson.data(), bson.size(), true);
}

void map_callback(const std_msgs::String::ConstPtr &msg) {
    try {
        json m = json::parse(msg->data);
        {
            std::lock_guard<std::mutex> lk(map_mutex);
            map = m;
            has_map = true;
        }
        publish_map();
    } catch (const json::exception &e) {
        ROS_ERROR_STREAM("Error processing map JSON: " << e.what());
    }
}


// Re-publishes nav_msgs::Path to <prefix>planned_path/json as [{x,y}, ...].
// Frontend uses this for the orange "this is where the mower is going next"
// overlay; downsampled in the source via the planner already.
void planned_path_callback(const nav_msgs::Path::ConstPtr &msg) {
    json points = json::array();
    points.get_ptr<json::array_t*>()->reserve(msg->poses.size());
    for (const auto &pose : msg->poses) {
        json p;
        p["x"] = pose.pose.position.x;
        p["y"] = pose.pose.position.y;
        points.push_back(std::move(p));
    }
    try_publish("planned_path/json", points.dump(), true);
}

// Re-publishes the slic3r MarkerArray as [{points: [{x,y},...]}, ...].
// Each Marker's points become a separate stripe — the frontend renders them
// as cyan polylines so the user can see the planned mowing pattern.
void coverage_path_callback(const visualization_msgs::MarkerArray::ConstPtr &msg) {
    json stripes = json::array();
    for (const auto &marker : msg->markers) {
        if (marker.points.empty()) continue;
        json stripe;
        json pts = json::array();
        for (const auto &pt : marker.points) {
            json p;
            p["x"] = pt.x;
            p["y"] = pt.y;
            pts.push_back(std::move(p));
        }
        stripe["points"] = std::move(pts);
        stripes.push_back(std::move(stripe));
    }
    try_publish("coverage_path/json", stripes.dump(), true);
}

// Rolling history of mowed paths. We hard-cap the buffer so a long mowing
// session can't blow up MQTT payload size; the cap is the latest N points
// across all received Paths concatenated (older points drop off the front).
constexpr size_t MOWING_TRAIL_CAPACITY = 5000;
std::deque<std::pair<double, double>> mowing_trail_buffer;
std::mutex mowing_trail_mutex;

void mowing_trail_callback(const nav_msgs::Path::ConstPtr &msg) {
    {
        std::lock_guard<std::mutex> lk(mowing_trail_mutex);
        for (const auto &pose : msg->poses) {
            mowing_trail_buffer.emplace_back(pose.pose.position.x, pose.pose.position.y);
            if (mowing_trail_buffer.size() > MOWING_TRAIL_CAPACITY) {
                mowing_trail_buffer.pop_front();
            }
        }
    }
    json points = json::array();
    {
        std::lock_guard<std::mutex> lk(mowing_trail_mutex);
        for (const auto &[x, y] : mowing_trail_buffer) {
            json p;
            p["x"] = x;
            p["y"] = y;
            points.push_back(std::move(p));
        }
    }
    try_publish("mowing_trail/json", points.dump(), true);
}

void map_overlay_callback(const xbot_msgs::MapOverlay::ConstPtr &msg) {
    // Build a JSON and publish it

    json polys;
    for(const auto &poly : msg->polygons) {
        if(poly.polygon.points.size() < 2)
            continue;
        json poly_j;
        {
            json outline_poly_j;
            for (const auto &pt: poly.polygon.points) {
                json p_j;
                p_j["x"] = pt.x;
                p_j["y"] = pt.y;
                outline_poly_j.push_back(p_j);
            }
            poly_j["poly"] = outline_poly_j;
            poly_j["is_closed"] = poly.closed;
            poly_j["line_width"] = poly.line_width;
            poly_j["color"] = poly.color;
        }
        polys.push_back(poly_j);
    }

    json j;
    j["polygons"] = polys;
    {
        std::lock_guard<std::mutex> lk(map_overlay_mutex);
        map_overlay = j;
        has_map_overlay = true;
    }

    publish_map_overlay();
}


bool registerActions(xbot_msgs::RegisterActionsSrvRequest &req, xbot_msgs::RegisterActionsSrvResponse &res) {

    ROS_INFO_STREAM("new actions registered: " << req.node_prefix << " registered " << req.actions.size() << " actions.");

    {
        std::lock_guard<std::mutex> lk(registered_actions_mutex);
        registered_actions[req.node_prefix] = req.actions;
    }

    publish_actions();
    return true;
}

void rpc_publish_error(const int16_t code, const std::string &message, const nlohmann::basic_json<> &id = nullptr) {
    json err_resp = {{"jsonrpc", "2.0"},
                       {"error", {{"code", code}, {"message", message}}},
                       {"id", id}};
    try_publish("rpc/response", err_resp.dump(2));
}

void rpc_request_callback(const std::string &payload) {
    // Parse
    json req;
    try {
      req = json::parse(payload);
    } catch (const json::parse_error &e) {
      return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_JSON, "Could not parse request JSON");
    }

    // Validate
    if (!req.is_object()) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "Request is not a JSON object");
    }
    json id = req.contains("id") ? req["id"] : nullptr;
    if (id != nullptr && !id.is_string()) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "ID is not a string", id);
    } else if (!req.contains("jsonrpc") || !req["jsonrpc"].is_string() || req["jsonrpc"] != "2.0") {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "Invalid JSON-RPC version");
    } else if (!req.contains("method") || !req["method"].is_string()) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INVALID_REQUEST, "Method is not a string", req["id"]);
    }

    // Check if the method is registered
    const std::string method = req["method"];
    bool is_registered = false;
    {
        std::lock_guard<std::mutex> lk(registered_methods_mutex);
        for (const auto& [_, method_ids] : registered_methods) {
            if (std::find(method_ids.begin(), method_ids.end(), method) != method_ids.end()) {
                is_registered = true;
                break;
            }
        }
    }
    if (!is_registered) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_METHOD_NOT_FOUND, "Method \"" + method + "\" not found", req["id"]);
    }

    // Forward to the providers as ROS message
    xbot_rpc::RpcRequest msg;
    msg.method = method;
    msg.params = req.contains("params") ? req["params"].dump() : "";
    msg.id = id != nullptr ? id : "";
    rpc_request_pub.publish(msg);
}

void rpc_response_callback(const xbot_rpc::RpcResponse::ConstPtr &msg) {
    json result;
    try {
        result = json::parse(msg->result);
    } catch (const json::parse_error &e) {
        return rpc_publish_error(xbot_rpc::RpcError::ERROR_INTERNAL, "Internal error while parsing result JSON: " + std::string(e.what()), msg->id);
    }

    json j = {{"jsonrpc", "2.0"}, {"result", result}, {"id", msg->id}};
    try_publish("rpc/response", j.dump(2));
}

void rpc_error_callback(const xbot_rpc::RpcError::ConstPtr &msg) {
    rpc_publish_error(msg->code, msg->message, msg->id);
}

bool register_methods(xbot_rpc::RegisterMethodsSrvRequest &req, xbot_rpc::RegisterMethodsSrvResponse &res) {
    std::lock_guard<std::mutex> lk(registered_methods_mutex);
    registered_methods[req.node_id] = req.methods;
    ROS_INFO_STREAM("new methods registered: " << req.node_id << " registered " << req.methods.size() << " methods.");
    return true;
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "xbot_monitoring");
    has_map = false;
    has_map_overlay = false;


    n = new ros::NodeHandle();
    ros::NodeHandle paramNh("~");

    version_string = paramNh.param("software_version", std::string("UNKNOWN VERSION"));
    if(version_string.empty()) {
        version_string = "UNKNOWN VERSION";
    }

    external_mqtt_enable = paramNh.param("external_mqtt_enable", false);
    external_mqtt_topic_prefix = paramNh.param("external_mqtt_topic_prefix", std::string(""));
    if(!external_mqtt_topic_prefix.empty() && external_mqtt_topic_prefix.back() != '/') {
        // append the /
        external_mqtt_topic_prefix = external_mqtt_topic_prefix+"/";
    }

    external_mqtt_hostname = paramNh.param("external_mqtt_hostname", std::string(""));
    external_mqtt_port = std::to_string(paramNh.param("external_mqtt_port", 1883));
    external_mqtt_username = paramNh.param("external_mqtt_username", std::string(""));
    external_mqtt_password = paramNh.param("external_mqtt_password", std::string(""));

    if(external_mqtt_enable) {
        ROS_INFO_STREAM("Using external MQTT broker: " << external_mqtt_hostname << ":" << external_mqtt_port << " with topic prefix: " + external_mqtt_topic_prefix);
    }

    // First setup MQTT
    setupMqttClient();

    ros::ServiceServer register_action_service = n->advertiseService("xbot/register_actions", registerActions);

    ros::Subscriber robotStateSubscriber = n->subscribe("xbot_monitoring/robot_state", 10, robot_state_callback);
    ros::Subscriber mapSubscriber = n->subscribe("mower_map_service/json_map", 10, map_callback);
    ros::Subscriber mapOverlaySubscriber = n->subscribe("xbot_monitoring/map_overlay", 10, map_overlay_callback);
    // Path-layer bridges. Topics are advertised in their respective publishers
    // (ftc_local_planner, slic3r_coverage_planner, mower_logic). Frontend
    // subscribes to <prefix>planned_path/json etc.
    ros::Subscriber plannedPathSubscriber = n->subscribe(
        "/move_base_flex/FTCPlanner/global_plan", 1, planned_path_callback);
    ros::Subscriber coveragePathSubscriber = n->subscribe(
        "/slic3r_coverage_planner/path_marker_array", 1, coverage_path_callback);
    ros::Subscriber mowingTrailSubscriber = n->subscribe(
        "/mower_logic/mowing_path", 10, mowing_trail_callback);
    // /rosout_agg is the aggregated log stream that rqt_console reads from.
    // Buffered via rosout_callback so logs.tail() can return recent history.
    ros::Subscriber rosoutSubscriber = n->subscribe(
        "/rosout_agg", 100, rosout_callback);
    // mower_sessions_recorder publishes the full JSON sessions list as String;
    // we rebroadcast it retained on MQTT for the openmower-app /statistics page.
    ros::Subscriber mowingSessionsSubscriber = n->subscribe(
        "xbot_monitoring/mowing_sessions", 1, mowing_sessions_callback);

    cmd_vel_pub = n->advertise<geometry_msgs::Twist>("xbot_monitoring/remote_cmd_vel", 1);
    action_pub = n->advertise<std_msgs::String>("xbot/action", 1);

    rpc_request_pub = n->advertise<xbot_rpc::RpcRequest>(xbot_rpc::TOPIC_REQUEST, 100);
    ros::Subscriber rpc_response_sub = n->subscribe(xbot_rpc::TOPIC_RESPONSE, 100, rpc_response_callback);
    ros::Subscriber rpc_error_sub = n->subscribe(xbot_rpc::TOPIC_ERROR, 100, rpc_error_callback);
    ros::ServiceServer register_methods_service = n->advertiseService(xbot_rpc::SERVICE_REGISTER_METHODS, register_methods);

    ros::AsyncSpinner spinner(1);
    spinner.start();

    rpc_provider.init();

    ros::Rate sensor_check_rate(10.0);

    boost::regex topic_regex("/xbot_monitoring/sensors/.*/info");

    // Maps a sensor info topic to its subscriber. Only touched by this thread.
    std::map<std::string, ros::Subscriber> active_subscribers;
    std::vector<ros::Subscriber> sensor_data_subscribers;

    while (ros::ok()) {
        // Read the topics in /xbot_monitoring/sensors/.*/info and subscribe to them.
        ros::master::V_TopicInfo topics;
        ros::master::getTopics(topics);
        std::for_each(topics.begin(), topics.end(), [&](const ros::master::TopicInfo &item) {

            if (!boost::regex_match(item.name, topic_regex) || active_subscribers.count(item.name) != 0)
                return;

            ROS_INFO_STREAM("Found new sensor topic " << item.name);
            active_subscribers[item.name] = n->subscribe<xbot_msgs::SensorInfo>(
                item.name, 1, [topic = item.name, &sensor_data_subscribers](const xbot_msgs::SensorInfo::ConstPtr &msg) {
                    ROS_INFO_STREAM("Got sensor info for sensor on topic " << msg->sensor_name << " on topic " << topic);

                    bool is_new = false;
                    {
                        std::unique_lock<std::mutex> lk(found_sensors_mutex);
                        is_new = found_sensors.count(topic) == 0;

                        // Sensor already known and sensor-info equals?
                        if (!is_new && found_sensors[topic] == *msg) return;

                        found_sensors[topic] = *msg;  // Save the (new|changed) sensor info
                    }

                    // Let the info subscription alive for dynamic threshold changes
                    //active_subscribers.erase(topic);  // Stop subscribing to infos

                    if (is_new) {
                        subscribe_to_sensor(topic, sensor_data_subscribers);  // Subscribe for data
                    }

                    // Republish (new|changed) sensor info
                    // NOTE: If a sensor name or id changes, the related data topic wouldn't change!
                    //       But do we dynamically change a sensor name or id?
                    publish_sensor_metadata();
                }
            );
        });
        sensor_check_rate.sleep();
    }
    return 0;
}
