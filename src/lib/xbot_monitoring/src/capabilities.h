#pragma once

#include "nlohmann/json.hpp"

inline const nlohmann::ordered_json CAPABILITIES = {
    {"rpc", 1},
    {"map:json", 1},
    {"mqtt:params", 1},
    {"logs.tail", 1},
    {"system.restart_service", 1},
    {"system.reboot", 1},
    {"system.stats", 1},
    {"system.docker_prune", 1},
    {"telemetry.recording", 1},
    {"telemetry.list_sessions", 1},
    {"telemetry.get_session", 1},
};
