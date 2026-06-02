//
// Created by Clemens Elflein on 22.11.22.
// Copyright (c) 2022 Clemens Elflein. All rights reserved.
//
#include <array>
#include <atomic>
#include <cctype>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <mutex>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <deque>
#include <fcntl.h>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
#include <sys/wait.h>
#include <thread>
#include <unistd.h>
#include <unordered_set>

#include "config_io.h"
#include "events_io.h"
#include "sensors_history.h"
#include "yaml_io.h"
#include "xbot_msgs/Event.h"
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
#include "sensor_msgs/Imu.h"
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
#include "mower_msgs/HighLevelControlSrv.h"
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
void try_publish(std::string topic, std::string data, bool retain = false);

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

// Firmware version JSON object cached from mower_comms_v2/firmware_version.
// Empty until the first message arrives (firmware not yet booted, V1 setups
// without an STM32, etc.). Republished as part of version/json so the app gets
// a single retained payload covering both backend and firmware.
std::mutex firmware_version_mutex;
nlohmann::ordered_json firmware_version_json;

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

// Serialises the read-modify-write cycle on the user YAML file inside
// meta.config.set. Concurrent RPC handlers run on independent MQTT callback
// threads — without this lock, two parallel saves (e.g. two browser tabs
// hitting "Save" within ms of each other) both load the same on-disk state,
// apply their own changes, then race to write_yaml_file_atomic. The second
// write wins and the first set of changes is silently lost. The atomic write
// only guarantees crash safety, not concurrency safety.
static std::mutex config_write_mutex;
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

// Cache of the schema-derived leaf index. Built once on first use; the schema
// itself is already cached above so the work here is a single tree walk.
static std::mutex leaf_index_cache_mutex;
static std::unique_ptr<xbot_monitoring::config_io::SchemaLeafIndex> leaf_index_cache;

static const xbot_monitoring::config_io::SchemaLeafIndex& load_leaf_index_cached() {
    std::lock_guard<std::mutex> lk(leaf_index_cache_mutex);
    if (!leaf_index_cache) {
        leaf_index_cache = std::make_unique<xbot_monitoring::config_io::SchemaLeafIndex>(
            xbot_monitoring::config_io::collect_schema_leaves(load_schema_cached()));
    }
    return *leaf_index_cache;
}

// Resolve the path to the Docker-Compose .env file holding the OM_* /
// HARDWARE_PLATFORM-style hardware identifiers. The OpenMowerOS image puts
// it at /opt/stacks/openmower/.env; sandboxed runs can override it via the
// ~env_file_path ROS param.
static std::string get_env_file_path() {
    std::string path;
    ros::param::param<std::string>("~env_file_path", path,
                                   std::string("/opt/stacks/openmower/.env"));
    return path;
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

// Notification / event store. Single owner, single writer thread (the ROS
// callback). Persisted to ~/.openmower/events.json with a debounced write so
// bursty event streams do not hammer the disk.
xbot_monitoring::events_io::EventStore event_store;
std::string event_store_path;

// In-memory ring buffer of recent SensorDataDouble samples (~1 h @ 2 Hz per
// sensor). Populated on every numeric sensor message we forward to MQTT and
// consumed by the sensors.history* RPCs so the frontend can show charts
// immediately on connect instead of waiting for live samples.
xbot_monitoring::sensors_history::SensorHistory sensor_history;
std::mutex event_persist_mutex;
std::chrono::steady_clock::time_point event_persist_pending_until{};
std::thread event_persist_thread;
std::atomic<bool> event_persist_thread_running{false};

void schedule_event_persist() {
    using namespace std::chrono;
    if (event_store_path.empty()) return;
    {
        std::lock_guard<std::mutex> lk(event_persist_mutex);
        event_persist_pending_until = steady_clock::now() + milliseconds(500);
        if (event_persist_thread_running.exchange(true)) {
            return;  // a worker is already coalescing.
        }
    }
    std::thread([] {
        while (true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
            std::chrono::steady_clock::time_point deadline;
            {
                std::lock_guard<std::mutex> lk(event_persist_mutex);
                deadline = event_persist_pending_until;
            }
            if (std::chrono::steady_clock::now() < deadline) continue;
            try {
                event_store.persist(event_store_path);
            } catch (const std::exception& e) {
                ROS_WARN_STREAM("events: failed to persist " << event_store_path << ": " << e.what());
            }
            std::lock_guard<std::mutex> lk(event_persist_mutex);
            // Another emit may have re-armed the deadline while we were
            // writing; loop again rather than racing.
            if (std::chrono::steady_clock::now() < event_persist_pending_until) continue;
            event_persist_thread_running = false;
            return;
        }
    }).detach();
}

void event_callback(const xbot_msgs::Event::ConstPtr& msg) {
    json view = event_store.add(*msg);
    // Live stream: non-retained, single event per message.
    try_publish("events/stream", view.dump(), false);
    // Retained snapshot: full buffer, used by app cold-start.
    try_publish("events/json", event_store.snapshot_json().dump(), true);
    schedule_event_persist();
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

// Probes whether the OpenMowerOS compose file mounted us into the host PID
// namespace (`pid: host`). When set, we can `nsenter -t 1 -a <cmd>` to run
// host-scoped commands like reboot / docker / df. Without it, system.* calls
// that need host data degrade gracefully.
static bool host_namespace_available() {
    static bool checked = false;
    static bool available = false;
    if (!checked) {
        checked = true;
        // We run as a non-root user (`openmower`, see Dockerfile), so the
        // /proc/1/ns/* symlinks are not readable for us — readlink returns
        // EACCES. Use /proc/1/comm instead, which is world-readable: with
        // `pid: host` it reports the host init (e.g. "systemd"); without it,
        // PID 1 is our container's own entrypoint ("openmower_entrypo" /
        // "bash" / similar). Treat anything that isn't our own command as
        // "host pid namespace shared".
        std::ifstream f("/proc/1/comm");
        if (f) {
            std::string pid1_comm;
            std::getline(f, pid1_comm);
            // Strip trailing newline / whitespace just in case.
            while (!pid1_comm.empty() && std::isspace(static_cast<unsigned char>(pid1_comm.back()))) {
                pid1_comm.pop_back();
            }
            // Container PID 1 candidates we know we ship:
            //   - "openmower_entrypo" (truncated openmower_entrypoint.sh, /proc/comm caps at 15 chars)
            //   - "bash" / "sh" (when launched via shell wrappers)
            //   - "roslaunch" (only if exec'd directly, unlikely)
            // Anything else (typically "systemd" on the Pi) means we have
            // joined the host PID namespace.
            available = !(pid1_comm == "openmower_entrypo" ||
                          pid1_comm == "openmower_entry" ||
                          pid1_comm == "bash" ||
                          pid1_comm == "sh" ||
                          pid1_comm.empty());
        }
    }
    return available;
}

// Run a shell command, capture combined stdout+stderr, with a hard timeout.
// Returns {exit_code, output}. On timeout the child is killed and exit_code
// is set to -1. We do not use std::system because it cannot enforce a
// timeout, and a runaway `docker image prune` would block xbot_monitoring.
struct ShellResult {
    int exit_code;
    std::string output;
};
static ShellResult run_with_timeout(const std::string& command, std::chrono::seconds timeout) {
    ShellResult r{-1, ""};
    FILE* pipe = popen((command + " 2>&1").c_str(), "r");
    if (!pipe) {
        r.output = "popen failed";
        return r;
    }
    auto deadline = std::chrono::steady_clock::now() + timeout;
    char buf[4096];
    int fd = fileno(pipe);
    int flags = fcntl(fd, F_GETFL, 0);
    fcntl(fd, F_SETFL, flags | O_NONBLOCK);
    while (true) {
        if (std::chrono::steady_clock::now() > deadline) {
            pclose(pipe);
            r.output += "\n[timeout]";
            return r;
        }
        ssize_t n = read(fd, buf, sizeof(buf));
        if (n > 0) {
            r.output.append(buf, static_cast<size_t>(n));
        } else if (n == 0) {
            break;  // EOF
        } else if (errno == EAGAIN || errno == EWOULDBLOCK) {
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        } else {
            break;
        }
    }
    int status = pclose(pipe);
    r.exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
    return r;
}

// Read /proc/stat aggregate CPU line into the active+total tick counts.
// Returns false if the file cannot be parsed. Two snapshots taken ~100ms
// apart let us compute a percentage without depending on /proc/uptime.
static bool read_cpu_ticks(unsigned long long& active_out, unsigned long long& total_out) {
    std::ifstream f("/proc/stat");
    if (!f) return false;
    std::string line;
    if (!std::getline(f, line)) return false;
    // line: "cpu  user nice system idle iowait irq softirq steal guest guest_nice"
    std::istringstream is(line);
    std::string tag;
    is >> tag;
    if (tag != "cpu") return false;
    unsigned long long fields[10] = {0};
    int i = 0;
    while (i < 10 && (is >> fields[i])) ++i;
    unsigned long long total = 0;
    for (int k = 0; k < i; ++k) total += fields[k];
    // active = total - (idle + iowait)
    unsigned long long idle_ticks = (i > 3 ? fields[3] : 0) + (i > 4 ? fields[4] : 0);
    active_out = total - idle_ticks;
    total_out = total;
    return true;
}

static double measure_cpu_percent() {
    unsigned long long a1, t1, a2, t2;
    if (!read_cpu_ticks(a1, t1)) return -1.0;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    if (!read_cpu_ticks(a2, t2)) return -1.0;
    if (t2 == t1) return 0.0;
    double pct = 100.0 * static_cast<double>(a2 - a1) / static_cast<double>(t2 - t1);
    if (pct < 0) pct = 0;
    if (pct > 100) pct = 100;
    return pct;
}

static bool read_meminfo(uint64_t& total_bytes, uint64_t& available_bytes) {
    std::ifstream f("/proc/meminfo");
    if (!f) return false;
    std::string line;
    bool got_total = false, got_avail = false;
    while (std::getline(f, line)) {
        // "MemTotal:        4012544 kB"
        if (line.rfind("MemTotal:", 0) == 0) {
            unsigned long long kb = 0;
            if (std::sscanf(line.c_str(), "MemTotal: %llu kB", &kb) == 1) {
                total_bytes = static_cast<uint64_t>(kb) * 1024ULL;
                got_total = true;
            }
        } else if (line.rfind("MemAvailable:", 0) == 0) {
            unsigned long long kb = 0;
            if (std::sscanf(line.c_str(), "MemAvailable: %llu kB", &kb) == 1) {
                available_bytes = static_cast<uint64_t>(kb) * 1024ULL;
                got_avail = true;
            }
        }
        if (got_total && got_avail) break;
    }
    return got_total && got_avail;
}

// Pi CPU temperature. Try the kernel sysfs interface first (works for any
// Linux); fall back to vcgencmd via nsenter when the thermal_zone isn't
// exposed (some carrier boards on non-Pi hardware).
static bool read_cpu_temp_celsius(double& temp_out) {
    std::ifstream f("/sys/class/thermal/thermal_zone0/temp");
    if (f) {
        long millicelsius = 0;
        f >> millicelsius;
        if (f) {
            temp_out = millicelsius / 1000.0;
            return true;
        }
    }
    if (host_namespace_available()) {
        // sudo -n: container runs as non-root `openmower`; nsenter needs
        // CAP_SYS_ADMIN. Passwordless sudo is wired up in the Dockerfile.
        ShellResult r = run_with_timeout("sudo -n nsenter -t 1 -a vcgencmd measure_temp",
                                         std::chrono::seconds(2));
        if (r.exit_code == 0) {
            // Output format: "temp=54.2'C\n"
            double t = 0;
            if (std::sscanf(r.output.c_str(), "temp=%lf", &t) == 1) {
                temp_out = t;
                return true;
            }
        }
    }
    return false;
}

// Disk usage of the host root filesystem. statvfs("/") inside the container
// returns the overlay's view, not the SD/eMMC. nsenter -t 1 -a df gives us
// the host-side numbers we actually want to surface.
static bool read_host_disk_usage(uint64_t& total, uint64_t& used, uint64_t& free) {
    if (!host_namespace_available()) return false;
    ShellResult r = run_with_timeout(
        "sudo -n nsenter -t 1 -a df -B1 --output=size,used,avail /",
        std::chrono::seconds(3));
    if (r.exit_code != 0) return false;
    // Output:
    //   1B-blocks       Used      Avail
    //   31000000000  9100000000  20000000000
    std::istringstream is(r.output);
    std::string line;
    std::getline(is, line);  // header
    if (!std::getline(is, line)) return false;
    std::istringstream ds(line);
    unsigned long long t = 0, u = 0, a = 0;
    if (!(ds >> t >> u >> a)) return false;
    total = t;
    used = u;
    free = a;
    return true;
}

static bool read_uptime_seconds(double& uptime_out) {
    std::ifstream f("/proc/uptime");
    if (!f) return false;
    f >> uptime_out;
    return static_cast<bool>(f);
}

// Calibration collector for the imu.calibrate_level RPC. The RPC arms the
// collector and waits on the condition variable; the raw-IMU subscriber
// (imu_raw_callback) appends samples until the target count is reached,
// then notifies the waiter. Keeping the buffer on the *raw* topic
// (ll/imu/data_raw) — not the orientation-filtered one — guarantees the
// measured bias/tilt reflects the actual sensor state, free of Madgwick
// feedback. Declared here, before rpc_provider, so the lambda inside the
// RPC method list can reference it via unqualified name lookup.
struct ImuCalibrationCollector {
    std::mutex mtx;
    std::condition_variable cv;
    bool active = false;
    size_t target = 0;
    std::vector<std::array<double, 6>> samples;  // ax, ay, az, gx, gy, gz
};
ImuCalibrationCollector imu_calibration_collector;

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
        // Builds a flat snapshot the frontend can use to populate the form.
        // Three sources are merged:
        //   * yaml-user / yaml-hw fields → looked up by their dotted YAML
        //     path inside the merged YAML config (defaults + HW + user).
        //   * env fields → looked up in /opt/stacks/openmower/.env (the
        //     Docker-Compose env file holding hardware identifiers).
        //   * legacy OM_* fields without an x-yaml-path / x-source still get
        //     resolved via mower_config.yaml_mapping.json so older deployments
        //     keep working.
        // Keys are emitted under both their OM_* name (when known) and their
        // x-yaml-path (when known); the frontend picks whichever it sent.
        try {
            const json& mapping = load_mapping_cached();
            const auto& leaves = load_leaf_index_cached();
            json merged = read_merged_yaml_config();

            json env_snapshot = json::object();
            try {
                env_snapshot = xbot_monitoring::config_io::read_env_file(get_env_file_path());
            } catch (const std::exception& e) {
                ROS_WARN_STREAM("meta.config.get: failed to read env file: " << e.what());
            }

            json out = json::object();

            // Schema-driven projection — covers new yaml-path-only fields, ENV
            // fields, and legacy OM_* fields with x-source set.
            for (const auto& leaf : leaves.all) {
                json value;
                bool have_value = false;
                if (leaf.source == "env") {
                    if (!leaf.env_var.empty() && env_snapshot.contains(leaf.env_var)) {
                        value = env_snapshot[leaf.env_var];
                        have_value = true;
                    }
                } else if (leaf.source == "yaml-user" || leaf.source == "yaml-hw") {
                    if (!leaf.yaml_path.empty()) {
                        const json* v = xbot_monitoring::yaml_io::read_path(merged, leaf.yaml_path);
                        if (v) {
                            value = *v;
                            have_value = true;
                        }
                    }
                }
                if (!have_value) continue;
                if (!leaf.env_var.empty()) {
                    out[leaf.env_var] = value;
                }
                if (!leaf.yaml_path.empty()) {
                    out[leaf.yaml_path] = value;
                }
            }

            // Legacy fallback: any OM_* key in the mapping JSON that the schema
            // walker didn't already cover (e.g. fields without x-source) still
            // gets projected via the mapping path.
            for (auto it = mapping.begin(); it != mapping.end(); ++it) {
                if (it.key().rfind("_", 0) == 0) continue;
                if (!it.value().is_string()) continue;
                if (out.contains(it.key())) continue;
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
            // Hold the lock for the full read-modify-write cycle so a second
            // meta.config.set running on a parallel MQTT thread cannot read
            // the same on-disk state and clobber our update.
            std::lock_guard<std::mutex> config_lk(config_write_mutex);
            const json& mapping = load_mapping_cached();
            const auto& leaves = load_leaf_index_cached();
            const std::string user_path = get_user_yaml_path();
            json user_yaml = xbot_monitoring::yaml_io::read_yaml_file(user_path);

            // Collect the per-key write decisions in two passes: first compile
            // the planned writes (key, yaml_path, coerced value) and the skip
            // reasons, then apply them in one block. The two-pass split keeps
            // the user-YAML in a fully consistent state if a single bad change
            // would otherwise abort midway through the loop.
            struct PlannedWrite {
                std::string key;
                std::string yaml_path;
                json value;
            };
            std::vector<PlannedWrite> planned;
            json skipped_keys = json::array();

            auto skip = [&](const std::string& key, const char* reason) {
                skipped_keys.push_back(json::object({{"key", key}, {"reason", reason}}));
            };

            for (auto it = changes.begin(); it != changes.end(); ++it) {
                const std::string& key = it.key();

                // Resolve the change key against three indexes, in priority
                // order: schema-leaf-by-env-var (legacy + new),
                // schema-leaf-by-yaml-path (new yaml-path-only fields),
                // legacy mapping JSON (catch-all). The first match wins.
                const xbot_monitoring::config_io::SchemaLeaf* leaf = nullptr;
                if (auto sit = leaves.by_env_var.find(key); sit != leaves.by_env_var.end()) {
                    leaf = &sit->second;
                } else if (auto sit2 = leaves.by_yaml_path.find(key); sit2 != leaves.by_yaml_path.end()) {
                    leaf = &sit2->second;
                }

                std::string yaml_path;
                std::string om_key_for_coercion = key;
                if (leaf) {
                    if (leaf->readonly_via_ui) {
                        skip(key, "readonly");
                        continue;
                    }
                    if (leaf->source == "env") {
                        skip(key, "env-source");
                        continue;
                    }
                    if (leaf->source == "ros") {
                        skip(key, "ros-source");
                        continue;
                    }
                    yaml_path = leaf->yaml_path;
                    if (!leaf->env_var.empty()) {
                        om_key_for_coercion = leaf->env_var;
                    }
                } else if (mapping.contains(key) && mapping.at(key).is_string()) {
                    yaml_path = mapping.at(key).get<std::string>();
                } else {
                    skip(key, "unknown");
                    continue;
                }

                if (yaml_path.empty()) {
                    skip(key, "no-yaml-path");
                    continue;
                }

                json coerced = coerce_value_for_schema(om_key_for_coercion, it.value());

                // Range check: if the schema declares minimum/maximum for the
                // leaf and the (coerced) value is numeric and outside the
                // bound, refuse the write so a buggy or hostile caller cannot
                // poke wheel_distance_m=0 or antenna_offset_x=99 into the
                // running stack. Non-numeric values fall through.
                if (leaf && coerced.is_number()) {
                    double n = coerced.get<double>();
                    if (leaf->minimum && n < *leaf->minimum) {
                        skip(key, "out-of-range");
                        continue;
                    }
                    if (leaf->maximum && n > *leaf->maximum) {
                        skip(key, "out-of-range");
                        continue;
                    }
                }

                planned.push_back({key, yaml_path, coerced});
            }

            // Apply all planned writes to the in-memory YAML tree.
            for (const auto& p : planned) {
                xbot_monitoring::yaml_io::write_path(user_yaml, p.yaml_path, p.value);
            }

            // Crash-safe persist: copy the previous user YAML to <path>.bak,
            // then atomically write, then verify the new file parses back to
            // a YAML object. If the verify step fails we restore from the
            // backup so the next container start has a usable params file.
            const std::string backup_path = user_path + ".bak";
            std::error_code ec;
            std::filesystem::copy_file(
                user_path, backup_path,
                std::filesystem::copy_options::overwrite_existing, ec);
            if (ec) {
                ROS_WARN_STREAM("meta.config.set: could not back up " << user_path
                                << " to " << backup_path << ": " << ec.message());
            }

            xbot_monitoring::yaml_io::write_yaml_file_atomic(user_path, user_yaml);

            try {
                json verify = xbot_monitoring::yaml_io::read_yaml_file(user_path);
                if (!verify.is_object()) {
                    throw std::runtime_error("post-write YAML did not parse to an object");
                }
            } catch (const std::exception& verify_err) {
                ROS_ERROR_STREAM("meta.config.set: post-write verification failed: "
                                 << verify_err.what() << "; rolling back from " << backup_path);
                std::filesystem::copy_file(
                    backup_path, user_path,
                    std::filesystem::copy_options::overwrite_existing, ec);
                if (ec) {
                    throw std::runtime_error(
                        std::string("rollback failed: ") + ec.message() +
                        " (verification error: " + verify_err.what() + ")");
                }
                throw std::runtime_error(
                    std::string("post-write verification failed; rolled back: ") +
                    verify_err.what());
            }

            // YAML on disk is good. Now push each value into ros::param so
            // live nodes see it without a restart. We delay this until after
            // the file write so a write failure cannot leave the param
            // server divergent from disk.
            json updated_keys = json::array();
            for (const auto& p : planned) {
                apply_to_ros_param(p.yaml_path, p.value);
                updated_keys.push_back(p.key);
            }

            ROS_INFO_STREAM("meta.config.set wrote " << updated_keys.size()
                             << " change(s) to " << user_path
                             << " (skipped " << skipped_keys.size() << ")");
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
    RPC_METHOD("params.get_many", {
        // Read multiple ROS parameters at once. To avoid leaking arbitrary
        // rosparam values, the request is filtered against the schema's
        // x-ros-param whitelist — names not declared in the schema are
        // dropped. The frontend uses this to populate the live values for
        // the Motion Control / Localization / Navigation advanced sections.
        if (!params.is_object() || !params.contains("names") || !params["names"].is_array()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS,
                                          "Missing names array");
        }
        const auto& leaves = load_leaf_index_cached();
        json values = json::object();
        for (const auto& entry : params["names"]) {
            if (!entry.is_string()) continue;
            const std::string name = entry.get<std::string>();
            if (leaves.by_ros_param.find(name) == leaves.by_ros_param.end()) {
                continue;
            }

            // We don't know the runtime type, so probe in the order the
            // schema would have indicated. ros::param has typed getters,
            // try them in turn; first one to succeed wins. XmlRpcValue
            // would be more elegant but the typed API keeps numeric
            // precision intact for the JSON round-trip.
            bool got = false;
            {
                bool b;
                if (ros::param::get(name, b)) {
                    values[name] = b;
                    got = true;
                }
            }
            if (!got) {
                int i;
                if (ros::param::get(name, i)) {
                    values[name] = i;
                    got = true;
                }
            }
            if (!got) {
                double d;
                if (ros::param::get(name, d)) {
                    values[name] = d;
                    got = true;
                }
            }
            if (!got) {
                std::string s;
                if (ros::param::get(name, s)) {
                    values[name] = s;
                    got = true;
                }
            }
            // Leave the key out of the response when the param is not set on
            // the server. The frontend treats the absence as "fall back to
            // the schema default"; emitting JSON null instead would make RHF
            // coerce it to the string "null" and break validation.
            (void)got;
        }
        return json::object({{"values", values}});
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
    RPC_METHOD("system.reboot", {
        // Params: { delay_s?: number, default 1, clamped to [0, 30] }.
        // Requires `pid: host` in the OpenMowerOS compose so nsenter can hop
        // into the host PID namespace and call /sbin/reboot. Without it the
        // call is rejected up-front rather than silently no-oping.
        if (!host_namespace_available()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          "Host namespace not available — compose stack needs `pid: host`");
        }
        int delay_s = 1;
        if (params.is_object() && params.contains("delay_s") && params["delay_s"].is_number()) {
            delay_s = params["delay_s"].get<int>();
        }
        if (delay_s < 0) delay_s = 0;
        if (delay_s > 30) delay_s = 30;
        ROS_WARN_STREAM("system.reboot requested — host will reboot in " << delay_s << "s");
        std::thread([delay_s]() {
            std::this_thread::sleep_for(std::chrono::seconds(delay_s));
            // /sbin/reboot via nsenter into the host's mount + pid namespace.
            // Output discarded; we already responded to the RPC.
            int rc = std::system("sudo -n nsenter -t 1 -a /sbin/reboot >/dev/null 2>&1");
            if (rc != 0) {
                ROS_ERROR_STREAM("system.reboot: nsenter reboot exited with " << rc);
            }
        }).detach();
        return json::object({{"ok", true}, {"delay_s", delay_s}});
    }),
    RPC_METHOD("system.stats", {
        // Read host CPU%, RAM, CPU temp, disk usage and uptime. Each field
        // is best-effort: if we can't read it (e.g. running in dev without
        // pid: host) the field is simply omitted from the response so the
        // frontend can render "n/a" without the whole call failing.
        json result = json::object();
        double cpu = measure_cpu_percent();
        if (cpu >= 0) result["cpu_percent"] = cpu;
        uint64_t ram_total = 0, ram_avail = 0;
        if (read_meminfo(ram_total, ram_avail)) {
            result["ram_total_bytes"] = ram_total;
            result["ram_used_bytes"] = ram_total > ram_avail ? (ram_total - ram_avail) : 0;
            result["ram_available_bytes"] = ram_avail;
        }
        double temp = 0;
        if (read_cpu_temp_celsius(temp)) {
            result["cpu_temp_c"] = temp;
        }
        uint64_t disk_total = 0, disk_used = 0, disk_free = 0;
        if (read_host_disk_usage(disk_total, disk_used, disk_free)) {
            result["disk_total_bytes"] = disk_total;
            result["disk_used_bytes"] = disk_used;
            result["disk_free_bytes"] = disk_free;
        }
        double up = 0;
        if (read_uptime_seconds(up)) {
            result["uptime_seconds"] = up;
        }
        return result;
    }),
    RPC_METHOD("events.list", {
        // Params (all optional):
        //   limit: number, default 200, clamped to [1, 500]
        //   since_ts: number (epoch ms) — return events strictly newer than this
        //   severity_min: "info" | "warning" | "error" | "critical"
        //   types: string[] — if set, only return events whose type is in the list
        // Returns: { events: [...], unread: <int> }. Newest event first.
        xbot_monitoring::events_io::ListFilter filter;
        size_t limit = 200;
        if (params.is_object()) {
            if (params.contains("limit") && params["limit"].is_number_integer()) {
                int l = params["limit"].get<int>();
                if (l < 1) l = 1;
                if (l > 500) l = 500;
                limit = static_cast<size_t>(l);
            }
            if (params.contains("since_ts") && params["since_ts"].is_number()) {
                filter.since_ts = static_cast<uint64_t>(params["since_ts"].get<double>());
            }
            if (params.contains("severity_min") && params["severity_min"].is_string()) {
                std::string s = params["severity_min"].get<std::string>();
                if (s == "info") filter.severity_min = xbot_msgs::Event::SEVERITY_INFO;
                else if (s == "warning") filter.severity_min = xbot_msgs::Event::SEVERITY_WARNING;
                else if (s == "error") filter.severity_min = xbot_msgs::Event::SEVERITY_ERROR;
                else if (s == "critical") filter.severity_min = xbot_msgs::Event::SEVERITY_CRITICAL;
                else throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS,
                                                  "severity_min must be info|warning|error|critical");
            }
            if (params.contains("types") && params["types"].is_array()) {
                for (const auto& t : params["types"]) {
                    if (t.is_string()) filter.types.push_back(t.get<std::string>());
                }
            }
        }
        filter.limit = limit;
        return json::object({
            {"events", event_store.list_json(filter)},
            {"unread", event_store.unread_count()},
        });
    }),
    RPC_METHOD("events.ack", {
        // Params: { id: string }. Marks the matching event as acked. Unknown
        // ids are not an error — the result simply reports ok=false.
        if (!params.is_object() || !params.contains("id") || !params["id"].is_string()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS,
                                          "id (string) is required");
        }
        bool ok = event_store.ack(params["id"].get<std::string>());
        if (ok) {
            try_publish("events/json", event_store.snapshot_json().dump(), true);
            schedule_event_persist();
        }
        return json::object({{"ok", ok}});
    }),
    RPC_METHOD("events.ack_all", {
        event_store.ack_all();
        try_publish("events/json", event_store.snapshot_json().dump(), true);
        schedule_event_persist();
        return json::object({{"ok", true}});
    }),
    RPC_METHOD("events.clear", {
        event_store.clear();
        try_publish("events/json", event_store.snapshot_json().dump(), true);
        schedule_event_persist();
        return json::object({{"ok", true}});
    }),
    RPC_METHOD("sensors.history", {
        // Params:
        //   sensor_id: string  (required) — id matching SensorInfo.sensor_id
        //   since_ts:  number  (optional, epoch ms) — return strictly newer
        //   limit:     number  (optional, default 7200, clamped to [1, 7200])
        // Returns: { sensor_id, samples: [{ts_ms, value}, ...] } oldest→newest
        if (!params.is_object() || !params.contains("sensor_id") ||
            !params["sensor_id"].is_string()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INVALID_PARAMS,
                                          "sensor_id (string) is required");
        }
        std::optional<uint64_t> since_ts;
        if (params.contains("since_ts") && params["since_ts"].is_number()) {
            since_ts = static_cast<uint64_t>(params["since_ts"].get<double>());
        }
        size_t limit = xbot_monitoring::sensors_history::kSamplesPerSensor;
        if (params.contains("limit") && params["limit"].is_number_integer()) {
            int l = params["limit"].get<int>();
            if (l < 1) l = 1;
            if (l > static_cast<int>(xbot_monitoring::sensors_history::kSamplesPerSensor)) {
                l = static_cast<int>(xbot_monitoring::sensors_history::kSamplesPerSensor);
            }
            limit = static_cast<size_t>(l);
        }
        return sensor_history.list_json(params["sensor_id"].get<std::string>(), since_ts, limit);
    }),
    RPC_METHOD("sensors.history_bulk", {
        // Params (all optional):
        //   since_ts: number  (epoch ms) — return strictly newer
        //   limit:    number  (default 7200, clamped to [1, 7200]) — per sensor
        // Returns: { sensors: { <sensor_id>: [{ts_ms, value}, ...], ... } }
        std::optional<uint64_t> since_ts;
        size_t limit = xbot_monitoring::sensors_history::kSamplesPerSensor;
        if (params.is_object()) {
            if (params.contains("since_ts") && params["since_ts"].is_number()) {
                since_ts = static_cast<uint64_t>(params["since_ts"].get<double>());
            }
            if (params.contains("limit") && params["limit"].is_number_integer()) {
                int l = params["limit"].get<int>();
                if (l < 1) l = 1;
                if (l > static_cast<int>(xbot_monitoring::sensors_history::kSamplesPerSensor)) {
                    l = static_cast<int>(xbot_monitoring::sensors_history::kSamplesPerSensor);
                }
                limit = static_cast<size_t>(l);
            }
        }
        return sensor_history.list_all_json(since_ts, limit);
    }),
    RPC_METHOD("imu.calibrate_level", {
        // "Treat current pose as level" calibration. The mower must be parked
        // on flat ground, motionless, with the chassis at its real-world
        // resting orientation. We average ll/imu/data_raw for ~2 s, derive:
        //   - mounting roll/pitch from the gravity vector in the IMU frame
        //   - gyro bias x/y/z from the mean angular velocity
        // and persist them under ll.services.imu.* via the same atomic-write
        // pipeline meta.config.set uses, so a stack restart is not required.
        // Yaw is *not* part of the calibration: drift is corrected continuously
        // from /xbot_positioning/xb_pose inside imu_orientation_filter.
        constexpr size_t kTargetSamples = 200;
        constexpr auto kMaxWait = std::chrono::seconds(4);
        constexpr double kAccelStddevThreshold = 0.10;   // m/s^2 per axis
        constexpr double kGyroStddevThreshold = 0.02;    // rad/s per axis

        // Arm the collector and wait for the raw subscriber to fill the
        // buffer. The mutex serialises against imu_raw_callback; the cv lets
        // the callback wake us as soon as the target count is reached.
        {
            std::lock_guard<std::mutex> lk(imu_calibration_collector.mtx);
            imu_calibration_collector.samples.clear();
            imu_calibration_collector.samples.reserve(kTargetSamples);
            imu_calibration_collector.target = kTargetSamples;
            imu_calibration_collector.active = true;
        }

        std::vector<std::array<double, 6>> samples;
        {
            std::unique_lock<std::mutex> lk(imu_calibration_collector.mtx);
            imu_calibration_collector.cv.wait_for(lk, kMaxWait, [] {
                return imu_calibration_collector.samples.size() >= imu_calibration_collector.target;
            });
            imu_calibration_collector.active = false;
            samples = std::move(imu_calibration_collector.samples);
            imu_calibration_collector.samples.clear();
        }

        if (samples.size() < kTargetSamples / 2) {
            // Less than ~1 s of data — the raw IMU topic is probably down.
            throw xbot_rpc::RpcException(
                xbot_rpc::RpcError::ERROR_INTERNAL,
                std::string("Only ") + std::to_string(samples.size()) +
                    " IMU samples received in " + std::to_string(kMaxWait.count()) +
                    "s — is ll/imu/data_raw publishing?");
        }

        // Mean and stddev per axis. We compute stddev with the two-pass
        // formulation (mean first, squared deviations second) — Welford would
        // be marginally more numerically stable but the value range here
        // (|a| ~ 10, |g| ~ 0.01) is well within double precision.
        std::array<double, 6> mean{};
        for (const auto& s : samples) {
            for (size_t i = 0; i < 6; ++i) mean[i] += s[i];
        }
        for (double& m : mean) m /= static_cast<double>(samples.size());

        std::array<double, 6> var{};
        for (const auto& s : samples) {
            for (size_t i = 0; i < 6; ++i) {
                double d = s[i] - mean[i];
                var[i] += d * d;
            }
        }
        for (double& v : var) v /= static_cast<double>(samples.size());
        std::array<double, 6> stddev{};
        for (size_t i = 0; i < 6; ++i) stddev[i] = std::sqrt(var[i]);

        // Per-axis check: if any axis exceeds the threshold the mower moved
        // during the recording. The thresholds are loose enough to tolerate
        // sensor noise on a still mower (LSM6DS3TR-C @ 100 Hz) but tight
        // enough to catch a hand-shake.
        const double accel_max = std::max({stddev[0], stddev[1], stddev[2]});
        const double gyro_max = std::max({stddev[3], stddev[4], stddev[5]});
        if (accel_max > kAccelStddevThreshold || gyro_max > kGyroStddevThreshold) {
            json err_data;
            err_data["accel_stddev"] = json::array({stddev[0], stddev[1], stddev[2]});
            err_data["gyro_stddev"] = json::array({stddev[3], stddev[4], stddev[5]});
            err_data["accel_threshold"] = kAccelStddevThreshold;
            err_data["gyro_threshold"] = kGyroStddevThreshold;
            err_data["samples_count"] = samples.size();
            throw xbot_rpc::RpcException(
                xbot_rpc::RpcError::ERROR_INTERNAL,
                std::string("MOWER_NOT_STILL: accel_stddev_max=") +
                    std::to_string(accel_max) + " rad/s^2, gyro_stddev_max=" +
                    std::to_string(gyro_max) + " rad/s — " + err_data.dump());
        }

        // Mounting offsets from the gravity vector. With the mower on a flat
        // surface, the only acceleration the IMU sees is g pointing in -Z
        // (REP-103 body frame: X forward, Y left, Z up). The remap baked in
        // ll.services.imu.axis_config (default "+X-Y-Z") already brings the
        // raw chip axes into this frame. Any deviation of (ax, ay, az) from
        // (0, 0, +g) is the mounting tilt:
        //   roll  = atan2( ay, az )         (rotation around X, +Y up tilts +)
        //   pitch = atan2(-ax, sqrt(ay^2 + az^2))  (rotation around Y)
        // These offsets are subtracted at the filter *output* so the raw
        // sensor stream stays untouched.
        const double ax = mean[0], ay = mean[1], az = mean[2];
        const double mounting_roll = std::atan2(ay, az);
        const double mounting_pitch = std::atan2(-ax, std::sqrt(ay * ay + az * az));
        const double gyro_bias_x = mean[3];
        const double gyro_bias_y = mean[4];
        const double gyro_bias_z = mean[5];

        // Persist via the same write pipeline meta.config.set uses: serialise
        // on config_write_mutex, write all five paths into the in-memory tree,
        // atomic-write to disk with a .bak rollback, then push into the live
        // ros::param tree so imu_orientation_filter sees the new offsets on
        // its next dynamic_reconfigure poll (or, failing that, restart).
        try {
            std::lock_guard<std::mutex> config_lk(config_write_mutex);
            const std::string user_path = get_user_yaml_path();
            json user_yaml;
            try {
                user_yaml = xbot_monitoring::yaml_io::read_yaml_file(user_path);
            } catch (...) {
                user_yaml = json::object();
            }
            if (!user_yaml.is_object()) user_yaml = json::object();

            xbot_monitoring::yaml_io::write_path(user_yaml,
                "ll.services.imu.mounting_roll_offset_rad", mounting_roll);
            xbot_monitoring::yaml_io::write_path(user_yaml,
                "ll.services.imu.mounting_pitch_offset_rad", mounting_pitch);
            xbot_monitoring::yaml_io::write_path(user_yaml,
                "ll.services.imu.gyro_bias.x", gyro_bias_x);
            xbot_monitoring::yaml_io::write_path(user_yaml,
                "ll.services.imu.gyro_bias.y", gyro_bias_y);
            xbot_monitoring::yaml_io::write_path(user_yaml,
                "ll.services.imu.gyro_bias.z", gyro_bias_z);

            const std::string backup_path = user_path + ".bak";
            std::error_code ec;
            std::filesystem::copy_file(
                user_path, backup_path,
                std::filesystem::copy_options::overwrite_existing, ec);
            if (ec) {
                ROS_WARN_STREAM("imu.calibrate_level: could not back up " << user_path
                                << " to " << backup_path << ": " << ec.message());
            }

            xbot_monitoring::yaml_io::write_yaml_file_atomic(user_path, user_yaml);

            try {
                json verify = xbot_monitoring::yaml_io::read_yaml_file(user_path);
                if (!verify.is_object()) {
                    throw std::runtime_error("post-write YAML did not parse to an object");
                }
            } catch (const std::exception& verify_err) {
                ROS_ERROR_STREAM("imu.calibrate_level: post-write verification failed: "
                                 << verify_err.what() << "; rolling back from " << backup_path);
                std::filesystem::copy_file(
                    backup_path, user_path,
                    std::filesystem::copy_options::overwrite_existing, ec);
                throw std::runtime_error(
                    std::string("post-write verification failed: ") + verify_err.what());
            }

            apply_to_ros_param("ll.services.imu.mounting_roll_offset_rad", mounting_roll);
            apply_to_ros_param("ll.services.imu.mounting_pitch_offset_rad", mounting_pitch);
            apply_to_ros_param("ll.services.imu.gyro_bias.x", gyro_bias_x);
            apply_to_ros_param("ll.services.imu.gyro_bias.y", gyro_bias_y);
            apply_to_ros_param("ll.services.imu.gyro_bias.z", gyro_bias_z);
        } catch (const std::exception& e) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          std::string("Failed to persist IMU calibration: ") + e.what());
        }

        ROS_INFO_STREAM("imu.calibrate_level: roll=" << mounting_roll
                        << " pitch=" << mounting_pitch
                        << " bias=(" << gyro_bias_x << "," << gyro_bias_y << "," << gyro_bias_z << ")"
                        << " from " << samples.size() << " samples");

        json result;
        result["mounting_roll_offset_rad"] = mounting_roll;
        result["mounting_pitch_offset_rad"] = mounting_pitch;
        result["gyro_bias"] = json::object({
            {"x", gyro_bias_x},
            {"y", gyro_bias_y},
            {"z", gyro_bias_z},
        });
        result["samples_count"] = samples.size();
        result["accel_stddev"] = json::array({stddev[0], stddev[1], stddev[2]});
        result["gyro_stddev"] = json::array({stddev[3], stddev[4], stddev[5]});
        return result;
    }),
    RPC_METHOD("mower.return_home", {
        // State-agnostic "go home now" — bridges to mower_logic's
        // HighLevelControlSrv with COMMAND_HOME so it works from idle, mowing,
        // and area-recording alike. The service handler in mower_logic.cpp
        // routes the call into the active behavior's command_home() override.
        // The service client is cached in a static local so we don't re-resolve
        // it on every call (the rpc_provider is constructed before NodeHandle
        // is available, so we can't init it eagerly).
        static ros::ServiceClient client;
        if (!client) {
            client = n->serviceClient<mower_msgs::HighLevelControlSrv>(
                "mower_service/high_level_control");
        }
        if (!client.waitForExistence(ros::Duration(1.0))) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          "mower_service/high_level_control not available");
        }
        mower_msgs::HighLevelControlSrv srv;
        srv.request.command = mower_msgs::HighLevelControlSrv::Request::COMMAND_HOME;
        if (!client.call(srv)) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          "Failed to call mower_service/high_level_control");
        }
        return json::object({{"ok", true}});
    }),
    RPC_METHOD("system.docker_prune", {
        // Removes all unused (dangling and not-referenced-by-any-container)
        // images on the host. The hardcoded command does not accept any
        // user-supplied arguments — there is no path to inject other docker
        // subcommands here.
        if (!host_namespace_available()) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          "Host namespace not available — compose stack needs `pid: host`");
        }
        ROS_WARN_STREAM("system.docker_prune requested — running `docker image prune -af` on host");
        ShellResult shell = run_with_timeout("sudo -n nsenter -t 1 -a docker image prune -af",
                                              std::chrono::seconds(60));
        if (shell.exit_code == -1) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          "docker image prune timed out or failed to spawn");
        }
        if (shell.exit_code != 0) {
            throw xbot_rpc::RpcException(xbot_rpc::RpcError::ERROR_INTERNAL,
                                          std::string("docker image prune failed: ") + shell.output);
        }
        // Parse "Total reclaimed space: 1.234GB" out of the output. Sizes are
        // emitted in human-readable form, so do a small unit table.
        uint64_t reclaimed_bytes = 0;
        const std::string marker = "Total reclaimed space:";
        size_t pos = shell.output.rfind(marker);
        if (pos != std::string::npos) {
            std::string tail = shell.output.substr(pos + marker.size());
            double n = 0;
            char unit_buf[8] = {0};
            if (std::sscanf(tail.c_str(), " %lf%7s", &n, unit_buf) >= 1) {
                std::string unit(unit_buf);
                double mult = 1.0;
                if (unit == "B" || unit.empty()) mult = 1.0;
                else if (unit == "kB" || unit == "KB") mult = 1000.0;
                else if (unit == "MB") mult = 1000.0 * 1000.0;
                else if (unit == "GB") mult = 1000.0 * 1000.0 * 1000.0;
                else if (unit == "TB") mult = 1000.0 * 1000.0 * 1000.0 * 1000.0;
                else if (unit == "KiB") mult = 1024.0;
                else if (unit == "MiB") mult = 1024.0 * 1024.0;
                else if (unit == "GiB") mult = 1024.0 * 1024.0 * 1024.0;
                reclaimed_bytes = static_cast<uint64_t>(n * mult);
            }
        }
        return json::object({
            {"ok", true},
            {"reclaimed_bytes", reclaimed_bytes},
            {"output", shell.output},
        });
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

void try_publish(std::string topic, std::string data, bool retain) {
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
    {
        std::lock_guard<std::mutex> lk(firmware_version_mutex);
        if (!firmware_version_json.empty()) {
            version["firmware"] = firmware_version_json;
        }
    }
    try_publish("version/json", version.dump(), true);
    auto bson = json::to_bson(version);
    try_publish_binary("version", bson.data(), bson.size(), true);
}

void firmware_version_callback(const std_msgs::String::ConstPtr& msg) {
    nlohmann::ordered_json parsed;
    try {
        parsed = nlohmann::ordered_json::parse(msg->data);
    } catch (const std::exception& e) {
        ROS_WARN_STREAM("firmware_version: invalid JSON payload: " << e.what());
        return;
    }
    {
        std::lock_guard<std::mutex> lk(firmware_version_mutex);
        if (firmware_version_json == parsed) {
            return;
        }
        firmware_version_json = std::move(parsed);
    }
    // Republish the retained version payload so late MQTT subscribers see the
    // firmware block without waiting for a backend restart.
    publish_version();
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

                // Mirror into the local ring buffer so sensors.history* can
                // replay the trailing 1 h on demand. Use the sensor's own
                // stamp when set, otherwise fall back to wall-clock — the
                // chart is plotted against ts_ms, not ROS time.
                uint64_t ts_ms = static_cast<uint64_t>(msg->stamp.toNSec() / 1'000'000ULL);
                if (ts_ms == 0) {
                    using namespace std::chrono;
                    ts_ms = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();
                }
                sensor_history.push(info.sensor_id, msg->data, ts_ms);
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

    // R9a — detailed GPS + WLAN fields the App's /dashboard and /heatmap need.
    j["gps_fix_type"] = msg->gps_fix_type;
    j["gps_satellite_count"] = msg->gps_satellite_count;
    j["gps_pdop"] = msg->gps_pdop;
    j["wifi_signal_dbm"] = msg->wifi_signal_dbm;
    j["wifi_link_quality"] = msg->wifi_link_quality;

    try_publish("robot_state/json", j.dump());
    json data;
    data["d"] = j;
    auto bson = json::to_bson(data);
    try_publish_binary("robot_state/bson", bson.data(), bson.size());
}

// Calibration collector lives near the top (see declaration above
// rpc_provider) so the imu.calibrate_level lambda can reference it via
// unqualified name lookup. The callback is defined here, after sensor_msgs
// includes have all been resolved.
void imu_raw_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    std::lock_guard<std::mutex> lk(imu_calibration_collector.mtx);
    if (!imu_calibration_collector.active) return;
    imu_calibration_collector.samples.push_back({
        msg->linear_acceleration.x, msg->linear_acceleration.y, msg->linear_acceleration.z,
        msg->angular_velocity.x, msg->angular_velocity.y, msg->angular_velocity.z,
    });
    if (imu_calibration_collector.samples.size() >= imu_calibration_collector.target) {
        imu_calibration_collector.cv.notify_all();
    }
}

// Bridge for the Madgwick-filtered IMU stream. The filter publishes at the
// sensor's native ~100 Hz; for the live UI 30 Hz is plenty and saves about
// two thirds of the WebSocket bandwidth. We use the message's own header
// stamp for throttling so a paused or slow filter does not get amplified by
// wall-clock jitter on the bridge side.
void imu_data_callback(const sensor_msgs::Imu::ConstPtr &msg) {
    constexpr double kPublishPeriodSeconds = 1.0 / 30.0;
    static ros::Time last_publish_stamp(0, 0);

    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
    if (!last_publish_stamp.isZero() && (stamp - last_publish_stamp).toSec() < kPublishPeriodSeconds) {
        return;
    }
    last_publish_stamp = stamp;

    json payload;
    payload["ax"] = msg->linear_acceleration.x;
    payload["ay"] = msg->linear_acceleration.y;
    payload["az"] = msg->linear_acceleration.z;
    payload["gx"] = msg->angular_velocity.x;
    payload["gy"] = msg->angular_velocity.y;
    payload["gz"] = msg->angular_velocity.z;
    payload["qw"] = msg->orientation.w;
    payload["qx"] = msg->orientation.x;
    payload["qy"] = msg->orientation.y;
    payload["qz"] = msg->orientation.z;
    // Wall-clock millisecond stamp so the App can compute dt for charts
    // even when the ROS stamp clock differs from the browser clock.
    payload["ts_ms"] = static_cast<int64_t>(stamp.toNSec() / 1000000);

    json envelope;
    envelope["d"] = payload;
    auto bson = json::to_bson(envelope);
    try_publish_binary("imu/stream", bson.data(), bson.size());
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
    // Compact dump (no indent): the client parses this as JSON, so pretty-print
    // whitespace is pure overhead — it inflated every RPC response ~25-30%,
    // which made large payloads (e.g. telemetry.get_session) slow over the
    // WebSocket broker.
    try_publish("rpc/response", j.dump());
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

    // Default carries a hint about *why* the version is unknown so the UI
    // immediately surfaces a misconfiguration (missing param) rather than the
    // ambiguous string "UNKNOWN VERSION".
    version_string = paramNh.param("software_version", std::string("unknown (no software_version param set)"));
    if(version_string.empty()) {
        version_string = "unknown (empty software_version param)";
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
    // Bridge the orientation-filtered IMU stream (sensor_msgs/Imu with
    // populated quaternion) to MQTT as throttled BSON for the openmower-app
    // 3D viewer. See imu_orientation_filter package for the publisher side.
    ros::Subscriber imuDataSubscriber = n->subscribe("imu/data", 50, imu_data_callback);
    // Raw IMU subscription used exclusively by imu.calibrate_level. Always
    // active but a no-op until the RPC arms the collector.
    ros::Subscriber imuRawSubscriber = n->subscribe("ll/imu/data_raw", 50, imu_raw_callback);
    // Firmware version is published by HighLevelServiceInterface in mower_comms_v2
    // as a latched String (JSON: {git_hash, build_date}). Cache + republish via
    // version/json so the openmower-app sees the full version block.
    ros::Subscriber firmwareVersionSubscriber = n->subscribe(
        "mower_comms_v2/firmware_version", 1, firmware_version_callback);
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

    // Notification / event bus. Producers (mower_logic, mower_scheduler,
    // others) publish on /events; we persist + mirror to MQTT. Path is
    // overridable via ROS param so tests/dev setups can redirect it.
    {
        const char *home = std::getenv("HOME");
        std::string default_events_path = std::string(home ? home : "/root") + "/.openmower/events.json";
        event_store_path = paramNh.param("events_path", default_events_path);
        try {
            event_store.load(event_store_path);
            // Republish the loaded snapshot so subscribers connecting before
            // the first new event still see history.
            try_publish("events/json", event_store.snapshot_json().dump(), true);
        } catch (const std::exception& e) {
            ROS_WARN_STREAM("events: load failed: " << e.what());
        }
    }
    ros::Subscriber eventsSubscriber = n->subscribe("/events", 50, event_callback);

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
