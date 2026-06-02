// Created by Clemens Elflein on 2/21/22.
// Copyright (c) 2022 Clemens Elflein and OpenMower contributors. All rights reserved.
//
// This file is part of OpenMower.
//
// OpenMower is free software: you can redistribute it and/or modify it under the terms of the GNU General Public
// License as published by the Free Software Foundation, version 3 of the License.
//
// OpenMower is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY; without even the implied
// warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License along with OpenMower. If not, see
// <https://www.gnu.org/licenses/>.
//
#include "MowingBehavior.h"

#include <cryptopp/cryptlib.h>
#include <cryptopp/hex.h>
#include <cryptopp/sha.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/Reconfigure.h>
#include <nav_msgs/Path.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <algorithm>
#include <cmath>
#include <event_publisher/event_publisher.hpp>
#include <limits>

#include "DockingBehavior.h"
#include "IdleBehavior.h"
#include "mower_logic/CheckPoint.h"
#include "mower_map/ClearNavPointSrv.h"
#include "mower_map/GetMowingAreaSrv.h"
#include "mower_map/SetNavPointSrv.h"

extern ros::ServiceClient mapClient;
extern ros::ServiceClient pathClient;
extern ros::ServiceClient pathProgressClient;
extern ros::ServiceClient setNavPointClient;
extern ros::ServiceClient clearNavPointClient;

extern actionlib::SimpleActionClient<mbf_msgs::MoveBaseAction>* mbfClient;
extern actionlib::SimpleActionClient<mbf_msgs::ExePathAction>* mbfClientExePath;
extern mower_logic::MowerLogicConfig getConfig();
extern void setConfig(mower_logic::MowerLogicConfig);

extern void registerActions(std::string prefix, const std::vector<xbot_msgs::ActionInfo>& actions);

MowingBehavior MowingBehavior::INSTANCE;

std::string MowingBehavior::state_name() {
  if (paused) {
    return "PAUSED";
  }
  return "MOWING";
}

Behavior* MowingBehavior::execute() {
  shared_state->active_semiautomatic_task = true;

  while (ros::ok() && !aborted) {
    if (currentMowingPaths.empty() && !create_mowing_plan(currentMowingArea)) {
      ROS_INFO_STREAM("MowingBehavior: Could not create mowing plan, docking");
      // Start again from first area next time.
      reset();
      // We cannot create a plan, so we're probably done. Go to docking station
      return &DockingBehavior::INSTANCE;
    }

    // No plan will be created if the area is skipped
    if (currentMowingPaths.empty()) {
      if (!advance_to_next_area()) {
        // Explicit area queue exhausted (all listed areas were inactive).
        reset();
        return &DockingBehavior::INSTANCE;
      }
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      continue;
    }

    // We have a plan, execute it
    ROS_INFO_STREAM("MowingBehavior: Executing mowing plan");
    bool finished = execute_mowing_plan();
    if (finished) {
      // skip to next area if current
      ROS_INFO_STREAM("MowingBehavior: Executing mowing plan - finished");
      currentMowingPaths.clear();
      if (!advance_to_next_area()) {
        // Explicit area queue fully mowed — we're done, dock.
        reset();
        return &DockingBehavior::INSTANCE;
      }
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
    }
  }

  if (!ros::ok()) {
    // something went wrong
    return nullptr;
  }
  // We got aborted. The default abort path goes to docking; abort_to_idle
  // requests an in-place stop and returns to idle so the user can intervene
  // (e.g. carry the mower) without the autopilot driving away first.
  if (abort_to_idle) {
    return &IdleBehavior::INSTANCE;
  }
  return &DockingBehavior::INSTANCE;
}

bool MowingBehavior::advance_to_next_area() {
  if (requestedAreaQueue.empty()) {
    // Legacy / 24-7 whole-map iteration: walk every map area in order.
    // Termination happens implicitly when create_mowing_plan() fails on an
    // out-of-range index, so we always report "more to do" here.
    currentMowingArea++;
    return true;
  }
  // Explicit scheduler-dispatched queue: hand out the next listed area.
  requestedAreaQueuePos++;
  if (requestedAreaQueuePos >= requestedAreaQueue.size()) {
    return false;
  }
  currentMowingArea = requestedAreaQueue[requestedAreaQueuePos];
  ROS_INFO_STREAM("MowingBehavior: advancing to queued area "
                  << currentMowingArea << " (" << (requestedAreaQueuePos + 1) << "/" << requestedAreaQueue.size()
                  << ")");
  return true;
}

void MowingBehavior::consume_next_run() {
  // A fresh scheduled run is signalled by next_run/pending=true. We latch the
  // whole param block into members and clear the flag so a resume after a
  // charge dock/undock cycle (this behavior is a persistent singleton) keeps
  // the in-progress run rather than re-dispatching it. map.start_in_area uses
  // the separate legacy /mower_logic/next_area_index and never sets pending.
  bool pending = false;
  if (ros::param::getCached("/mower_logic/next_run/pending", pending) && pending) {
    // A resume dispatch (next_run/resume=true) re-issues a run that was
    // interrupted (block window / stop-at-time) so it picks up where it left
    // off. The abort-to-dock path does not call reset(), so this singleton
    // still holds currentMowingPaths and the path progress in memory; we latch
    // the queue and overrides as usual but keep that progress, so execute()
    // resumes the retained plan from currentMowingPathIndex. If the paths were
    // lost (e.g. process restart, restore_checkpoint() rehydrated only the
    // indices), create_mowing_plan() rebuilds them and the plan-digest match
    // advances to the saved checkpoint instead of restarting the area.
    bool resume = false;
    ros::param::getCached("/mower_logic/next_run/resume", resume);

    std::vector<int> areas;
    ros::param::getCached("/mower_logic/next_run/area_indices", areas);
    requestedAreaQueue = areas;
    requestedAreaQueuePos = 0;

    requestedFillType = -1;
    ros::param::getCached("/mower_logic/next_run/pattern", requestedFillType);
    requestedSpeed = std::numeric_limits<double>::quiet_NaN();
    ros::param::getCached("/mower_logic/next_run/speed_mps", requestedSpeed);
    requestedTravelSpeed = std::numeric_limits<double>::quiet_NaN();
    ros::param::getCached("/mower_logic/next_run/travel_speed_mps", requestedTravelSpeed);
    requestedAngleDeg = std::numeric_limits<double>::quiet_NaN();
    ros::param::getCached("/mower_logic/next_run/angle_deg", requestedAngleDeg);
    requestedOutlineCount = -1;
    ros::param::getCached("/mower_logic/next_run/outline_count", requestedOutlineCount);
    requestedRunId.clear();
    ros::param::getCached("/mower_logic/next_run/run_id", requestedRunId);

    if (resume) {
      // Resume: keep currentMowingArea / currentMowingPath(Index) and the plan
      // digest from the checkpoint so the area continues at the saved position.
      // Fall back to the queue front only if we have no in-progress area.
      if (currentMowingPaths.empty() && !requestedAreaQueue.empty()) {
        currentMowingArea = requestedAreaQueue.front();
      }
      // Align the queue cursor with the area we are resuming so advance_to_next_
      // area() continues through the rest of the list rather than from the top.
      for (size_t i = 0; i < requestedAreaQueue.size(); i++) {
        if (requestedAreaQueue[i] == currentMowingArea) {
          requestedAreaQueuePos = i;
          break;
        }
      }
      ROS_INFO_STREAM("MowingBehavior: resuming scheduled run at area "
                      << currentMowingArea << " path " << currentMowingPath << " index " << currentMowingPathIndex);
    } else {
      if (!requestedAreaQueue.empty()) {
        currentMowingArea = requestedAreaQueue.front();
      }
      currentMowingPaths.clear();
      currentMowingPath = 0;
      currentMowingPathIndex = 0;
      ROS_INFO_STREAM(
          "MowingBehavior: starting scheduled run over "
          << (requestedAreaQueue.empty() ? std::string("all active") : std::to_string(requestedAreaQueue.size()))
          << " area(s), fill=" << requestedFillType << " speed=" << requestedSpeed << " travel_speed="
          << requestedTravelSpeed << " angle_deg=" << requestedAngleDeg << " outline=" << requestedOutlineCount);
    }
    ros::param::set("/mower_logic/next_run/pending", false);
    return;
  }

  // No scheduled run pending. Honor the legacy single-area start from
  // map.start_in_area, and treat any non-scheduled entry as a manual run by
  // dropping per-run overrides so they cannot leak across runs.
  int requested_area = -1;
  if (ros::param::getCached("/mower_logic/next_area_index", requested_area) && requested_area >= 0) {
    ROS_INFO_STREAM("MowingBehavior: starting in area " << requested_area << " (from map.start_in_area)");
    requestedAreaQueue.clear();
    requestedAreaQueuePos = 0;
    requestedFillType = -1;
    requestedSpeed = std::numeric_limits<double>::quiet_NaN();
    requestedTravelSpeed = std::numeric_limits<double>::quiet_NaN();
    requestedAngleDeg = std::numeric_limits<double>::quiet_NaN();
    requestedOutlineCount = -1;
    requestedRunId.clear();
    currentMowingArea = requested_area;
    currentMowingPaths.clear();
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
    ros::param::set("/mower_logic/next_area_index", -1);
  }
}

// Namespace of the FTC local planner's dynamic_reconfigure server. The planner
// runs as a controller plugin inside move_base_flex (see
// open_mower/params/move_base_flex.yaml + ftc_local_planner.yaml), so its
// params live under /move_base_flex/FTCPlanner.
static const char* FTC_NS = "/move_base_flex/FTCPlanner";

void MowingBehavior::capture_global_speeds() {
  // Capture the current speeds so a per-run / per-area override can be restored
  // afterwards. dynamic_reconfigure mirrors its values onto the param server,
  // so a plain param read is enough.
  savedSpeedSlow = 0.15;
  savedSpeedFast = 0.4;
  ros::param::get(std::string(FTC_NS) + "/speed_slow", savedSpeedSlow);
  ros::param::get(std::string(FTC_NS) + "/speed_fast", savedSpeedFast);
  speedOverridden = false;
}

bool MowingBehavior::set_ftc_speeds(double slow, double fast) {
  dynamic_reconfigure::Reconfigure srv;
  dynamic_reconfigure::DoubleParameter p_slow;
  p_slow.name = "speed_slow";
  p_slow.value = slow;
  srv.request.config.doubles.push_back(p_slow);
  dynamic_reconfigure::DoubleParameter p_fast;
  p_fast.name = "speed_fast";
  p_fast.value = fast;
  srv.request.config.doubles.push_back(p_fast);
  return ros::service::call(std::string(FTC_NS) + "/set_parameters", srv);
}

void MowingBehavior::apply_mow_speed(double mowing_speed, double travel_speed) {
  const bool mowing_override = !std::isnan(mowing_speed) && mowing_speed > 0.0;
  const bool travel_override = !std::isnan(travel_speed) && travel_speed > 0.0;

  if (!mowing_override && !travel_override) {
    // No override for this area — restore the captured global speeds if a
    // previous area had overridden them, so each area mows at its own speed.
    restore_speed_override();
    return;
  }

  // speed_slow is the actual mowing speed; speed_fast is the approach/traversal
  // speed. Apply whichever is overridden and keep the captured global for the
  // other. The planner assumes slow <= fast, so clamp the effective slow to the
  // effective fast.
  double slow = mowing_override ? mowing_speed : savedSpeedSlow;
  double fast = travel_override ? travel_speed : savedSpeedFast;
  if (slow > fast) {
    slow = fast;
  }
  if (set_ftc_speeds(slow, fast)) {
    speedOverridden = true;
    ROS_INFO_STREAM("MowingBehavior: set FTC speeds slow=" << slow << " fast=" << fast << " m/s (global was slow="
                                                           << savedSpeedSlow << " fast=" << savedSpeedFast << ")");
  } else {
    ROS_WARN_STREAM("MowingBehavior: failed to set FTC speed override; keeping current speed");
  }
}

void MowingBehavior::restore_speed_override() {
  if (!speedOverridden) {
    return;
  }
  if (set_ftc_speeds(savedSpeedSlow, savedSpeedFast)) {
    ROS_INFO_STREAM("MowingBehavior: restored FTC speeds (slow=" << savedSpeedSlow << ", fast=" << savedSpeedFast
                                                                 << ")");
  } else {
    ROS_WARN_STREAM("MowingBehavior: failed to restore FTC speeds");
  }
  speedOverridden = false;
}

void MowingBehavior::enter() {
  skip_area = false;
  skip_path = false;
  abort_to_idle = false;
  paused = aborted = false;

  consume_next_run();
  // Capture the global FTC speeds once; the effective per-run/per-area speed is
  // applied inside create_mowing_plan() as each area's plan is built.
  capture_global_speeds();

  for (auto& a : actions) {
    a.enabled = true;
  }
  registerActions("mower_logic:mowing", actions);

  open_mower::events::EventPublisher::info("mowing.started", "Mowing started",
                                           {{"area_index", currentMowingArea}, {"run_id", requestedRunId}});
}

void MowingBehavior::exit() {
  // Restore the FTC planner speed before anything else so a per-run speed
  // override never bleeds into docking or a subsequent run; this runs on every
  // exit path (completion, abort, pause-to-dock).
  restore_speed_override();

  for (auto& a : actions) {
    a.enabled = false;
  }
  registerActions("mower_logic:mowing", actions);

  if (aborted) {
    // An aborted run previously emitted nothing here, leaving the scheduler
    // unable to tell a stuck/cancelled run from a completed one. Emit an
    // explicit aborted event carrying the run_id so failure correlation works.
    // abort_to_idle is a deliberate user stop; a plain abort is the failure /
    // stop-at-time / dock path.
    open_mower::events::EventPublisher::warning(
        "mowing.aborted", "Mowing aborted",
        {{"area_index", currentMowingArea}, {"run_id", requestedRunId}, {"to_idle", abort_to_idle}});
  } else if (!paused) {
    open_mower::events::EventPublisher::info("mowing.session_completed", "Mowing session finished",
                                             {{"area_index", currentMowingArea}, {"run_id", requestedRunId}});
  }
}

void MowingBehavior::reset() {
  currentMowingPaths.clear();
  currentMowingArea = 0;
  currentMowingPath = 0;
  currentMowingPathIndex = 0;
  // A completed (or unplannable) run ends the scheduler-dispatched queue and
  // its per-run overrides; the next dispatch repopulates them in enter().
  requestedAreaQueue.clear();
  requestedAreaQueuePos = 0;
  requestedFillType = -1;
  requestedSpeed = std::numeric_limits<double>::quiet_NaN();
  requestedTravelSpeed = std::numeric_limits<double>::quiet_NaN();
  requestedAngleDeg = std::numeric_limits<double>::quiet_NaN();
  requestedOutlineCount = -1;
  // increase cumulative mowing angle offset increment
  currentMowingAngleIncrementSum = std::fmod(currentMowingAngleIncrementSum + getConfig().mow_angle_increment, 360);
  checkpoint();

  if (config.automatic_mode == eAutoMode::SEMIAUTO) {
    ROS_INFO_STREAM("MowingBehavior: Finished semiautomatic task");
    shared_state->active_semiautomatic_task = false;
  }
}

bool MowingBehavior::needs_gps() {
  return true;
}

bool MowingBehavior::mower_enabled() {
  return mowerEnabled;
}

void MowingBehavior::update_actions() {
  for (auto& a : actions) {
    a.enabled = true;
  }

  // pause / resume switch. other actions are always available
  actions[0].enabled = !(requested_pause_flag & pauseType::PAUSE_MANUAL);
  actions[1].enabled = requested_pause_flag & pauseType::PAUSE_MANUAL;

  registerActions("mower_logic:mowing", actions);
}

bool MowingBehavior::create_mowing_plan(int area_index) {
  ROS_INFO_STREAM("MowingBehavior: Creating mowing plan for area: " << area_index);
  // Delete old plan and progress.
  currentMowingPaths.clear();

  // get the mowing area
  mower_map::GetMowingAreaSrv mapSrv;
  mapSrv.request.index = area_index;
  if (!mapClient.call(mapSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error loading mowing area");
    return false;
  }

  if (!mapSrv.response.area.active) {
    ROS_INFO_STREAM("MowingBehavior: Skipping inactive mowing area");
    return true;
  }

  // Area orientation is the same as the first point, unless explicitly specified in the area attributes
  double angle = 0;
  if (!std::isnan(mapSrv.response.area.angle)) {
    angle = mapSrv.response.area.angle;
    ROS_INFO_STREAM("MowingBehavior: Using explicitly specified mow angle: " << angle);
  } else {
    auto points = mapSrv.response.area.area.points;
    if (points.size() >= 2) {
      tf2::Vector3 first(points[0].x, points[0].y, 0);
      for (auto point : points) {
        tf2::Vector3 second(point.x, point.y, 0);
        auto diff = second - first;
        if (diff.length() > 2.0) {
          // we have found a point that has a distance of > 2 m, calculate the angle
          angle = atan2(diff.y(), diff.x());
          ROS_INFO_STREAM("MowingBehavior: Detected mow angle: " << angle);
          break;
        }
      }
    }
  }

  // add mowing angle offset increment and return into the <-180, 180> range
  double mow_angle_offset = std::fmod(getConfig().mow_angle_offset + currentMowingAngleIncrementSum + 180, 360);
  if (mow_angle_offset < 0) mow_angle_offset += 360;
  mow_angle_offset -= 180;
  ROS_INFO_STREAM("MowingBehavior: mowing angle offset (deg): " << mow_angle_offset);
  if (config.mow_angle_offset_is_absolute) {
    angle = mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Custom mowing angle: " << angle);
  } else {
    angle = angle + mow_angle_offset * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Auto-detected mowing angle + mowing angle offset: " << angle);
  }

  // A per-run angle override from the scheduler is absolute and supersedes the
  // auto-detected/global angle entirely (equivalent to a one-off
  // mow_angle_offset_is_absolute for this run), without mutating global config.
  if (!std::isnan(requestedAngleDeg)) {
    angle = requestedAngleDeg * (M_PI / 180.0);
    ROS_INFO_STREAM("MowingBehavior: Using per-run absolute mowing angle (deg): " << requestedAngleDeg);
  }

  // calculate coverage
  const auto& area = mapSrv.response.area;
  auto overrideOrGlobal = [](auto override, auto global, auto sentinel) {
    return (override != sentinel) ? override : global;
  };

  // Fill pattern, precedence per-run > per-area > global. The per-run override
  // is latched from the scheduler (requestedFillType, -1 = none); the per-area
  // value lives on the map area (-1 = none). Values map directly onto the
  // PlanPath fill enum.
  int fill_type = (requestedFillType >= 0) ? requestedFillType
                                           : overrideOrGlobal(area.fill_type, getConfig().default_mow_pattern, -1);
  ROS_INFO_STREAM("MowingBehavior: using fill pattern " << fill_type);

  // Effective mowing speed, precedence per-run > per-area > global. Travel speed
  // has no per-area value, so it is per-run only (NaN keeps the global). Applied
  // to the FTC planner now so this area mows at its own speed; restore_speed_
  // override() inside apply_mow_speed() resets it for areas without a value.
  double effectiveSpeed = !std::isnan(requestedSpeed) ? requestedSpeed : area.speed_mps;
  apply_mow_speed(effectiveSpeed, requestedTravelSpeed);

  slic3r_coverage_planner::PlanPath pathSrv;
  pathSrv.request.angle = angle;
  // A per-run outline override from the scheduler (>= 0) wins over the per-area
  // value and the global config, mirroring how requestedFillType/Angle behave.
  const int outline_count = overrideOrGlobal(area.outline_count, config.outline_count, -1);
  pathSrv.request.outline_count = (requestedOutlineCount >= 0) ? requestedOutlineCount : outline_count;
  pathSrv.request.outline_overlap_count =
      overrideOrGlobal(area.outline_overlap_count, config.outline_overlap_count, -1);
  pathSrv.request.outline = area.area;
  pathSrv.request.holes = area.obstacles;
  pathSrv.request.fill_type = static_cast<uint8_t>(fill_type);
  pathSrv.request.outer_offset = std::isnan(area.outline_offset) ? config.outline_offset : area.outline_offset;
  // Tool width / stripe spacing, per-area override (NaN = global tool_width).
  pathSrv.request.distance = std::isnan(area.distance) ? config.tool_width : area.distance;
  if (!pathClient.call(pathSrv)) {
    ROS_ERROR_STREAM("MowingBehavior: Error during coverage planning");
    return false;
  }

  // The coverage planner falls back to a linear fill when the requested pattern produces
  // no usable infill for (part of) the area. Surface that so the user understands why a
  // scheduled/parametrised run did not mow the pattern they picked on this area.
  if (pathSrv.response.fill_fallback) {
    ROS_WARN_STREAM("MowingBehavior: requested fill pattern " << fill_type << " produced no infill on area "
                                                              << area_index << "; planner fell back to linear fill");
    open_mower::events::EventPublisher::warning(
        "mowing.fill_fallback", "Requested mow pattern produced no infill; used linear fill instead",
        {{"area_index", area_index}, {"requested_fill", fill_type}, {"run_id", requestedRunId}});
  }

  currentMowingPaths = pathSrv.response.paths;

  // Calculate mowing plan digest from the poses
  // TODO: move to slic3r_coverage_planner
  CryptoPP::SHA256 hash;
  byte digest[CryptoPP::SHA256::DIGESTSIZE];
  for (const auto& path : currentMowingPaths) {
    for (const auto& pose_stamped : path.path.poses) {
      hash.Update(reinterpret_cast<const byte*>(&pose_stamped.pose), sizeof(geometry_msgs::Pose));
    }
  }
  hash.Final((byte*)&digest[0]);
  CryptoPP::HexEncoder encoder;
  std::string mowingPlanDigest = "";
  encoder.Attach(new CryptoPP::StringSink(mowingPlanDigest));
  encoder.Put(digest, sizeof(digest));
  encoder.MessageEnd();

  // Proceed to checkpoint?
  if (mowingPlanDigest == currentMowingPlanDigest) {
    ROS_INFO_STREAM("MowingBehavior: Advancing to checkpoint, path: " << currentMowingPath
                                                                      << " index: " << currentMowingPathIndex);
  } else {
    ROS_INFO_STREAM("MowingBehavior: Ignoring checkpoint for plan ("
                    << currentMowingPlanDigest << ") current mowing plan is (" << mowingPlanDigest << ")");
    // Plan has changed so must restart the area
    currentMowingPlanDigest = mowingPlanDigest;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
  }

  return true;
}

int getCurrentMowPathIndex() {
  ftc_local_planner::PlannerGetProgress progressSrv;
  int currentIndex = -1;
  if (pathProgressClient.call(progressSrv)) {
    currentIndex = progressSrv.response.index;
  } else {
    ROS_ERROR("MowingBehavior: getMowIndex() - Error getting progress from FTC planner");
  }
  return (currentIndex);
}

void printNavState(int state) {
  switch (state) {
    case actionlib::SimpleClientGoalState::PENDING: ROS_INFO(">>> State: Pending <<<"); break;
    case actionlib::SimpleClientGoalState::ACTIVE: ROS_INFO(">>> State: Active <<<"); break;
    case actionlib::SimpleClientGoalState::RECALLED: ROS_INFO(">>> State: Recalled <<<"); break;
    case actionlib::SimpleClientGoalState::REJECTED: ROS_INFO(">>> State: Rejected <<<"); break;
    case actionlib::SimpleClientGoalState::PREEMPTED: ROS_INFO(">>> State: Preempted <<<"); break;
    case actionlib::SimpleClientGoalState::ABORTED: ROS_INFO(">>> State: Aborted <<<"); break;
    case actionlib::SimpleClientGoalState::SUCCEEDED: ROS_INFO(">>> State: Succeeded <<<"); break;
    case actionlib::SimpleClientGoalState::LOST: ROS_INFO(">>> State: Lost <<<"); break;
    default: ROS_INFO(">>> State: Unknown Hu ? <<<"); break;
  }
}

bool MowingBehavior::execute_mowing_plan() {
  int first_point_attempt_counter = 0;
  int first_point_trim_counter = 0;
  ros::Time paused_time(0.0);

  // loop through all mowingPaths to execute the plan fully.
  while (currentMowingPath < currentMowingPaths.size() && ros::ok() && !aborted) {
    ////////////////////////////////////////////////
    // PAUSE HANDLING
    ////////////////////////////////////////////////
    if (requested_pause_flag) {  // pause was requested
      paused = true;
      mowerEnabled = false;
      u_int8_t last_requested_pause_flags = 0;
      while (requested_pause_flag && !aborted)  // while emergency and/or manual pause not asked to continue, we wait
      {
        if (last_requested_pause_flags != requested_pause_flag) {
          update_actions();
        }
        last_requested_pause_flags = requested_pause_flag;

        std::string pause_reason = "";
        if (requested_pause_flag & pauseType::PAUSE_EMERGENCY) {
          pause_reason += "on EMERGENCY";
          if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
            pause_reason += " and ";
          }
        }
        if (requested_pause_flag & pauseType::PAUSE_MANUAL) {
          pause_reason += "waiting for CONTINUE";
        }
        ROS_INFO_STREAM_THROTTLE(30, "MowingBehavior: PAUSED (" << pause_reason << ")");
        ros::Rate r(1.0);
        r.sleep();
      }
      // we will drop into paused, thus will also wait for GPS to be valid again
    }
    if (paused) {
      paused_time = ros::Time::now();
      while (!this->hasGoodGPS() && !aborted)  // while no good GPS we wait
      {
        ROS_INFO_STREAM("MowingBehavior: PAUSED (" << (ros::Time::now() - paused_time).toSec()
                                                   << "s) (waiting for GPS)");
        ros::Rate r(1.0);
        r.sleep();
      }
      ROS_INFO_STREAM("MowingBehavior: CONTINUING");
      paused = false;
      update_actions();
    }

    auto& path = currentMowingPaths[currentMowingPath];
    ROS_INFO_STREAM("MowingBehavior: Path segment length: " << path.path.poses.size() << " poses.");

    // Check if path is empty. If so, directly skip it
    if (currentMowingPathIndex >= path.path.poses.size()) {
      ROS_INFO_STREAM("MowingBehavior: Skipping empty path.");
      currentMowingPath++;
      currentMowingPathIndex = 0;
      continue;
    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    // DRIVE TO THE FIRST POINT OF THE MOW PATH
    //
    // * we have n attempts, if we fail we go to pause() mode because most likely it was GPS problems that
    //   prevented us from reaching the inital pose
    // * after n attempts, we fail the mow area and skip to the next one
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Moving to path segment starting point");
      if (path.is_outline && getConfig().add_fake_obstacle) {
        mower_map::SetNavPointSrv set_nav_point_srv;
        set_nav_point_srv.request.nav_pose = path.path.poses[currentMowingPathIndex].pose;
        setNavPointClient.call(set_nav_point_srv);
        sleep(1);
      }

      mbf_msgs::MoveBaseGoal moveBaseGoal;
      moveBaseGoal.target_pose = path.path.poses[currentMowingPathIndex];
      moveBaseGoal.controller = "FTCPlanner";
      mbfClient->sendGoal(moveBaseGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClient->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            mbfClientExePath->cancelAllGoals();
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (FIRST POINT) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            return false;
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (FIRST POINT)  Got status "
                          << current_status.state_ << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      first_point_attempt_counter++;
      if (current_status.state_ != actionlib::SimpleClientGoalState::SUCCEEDED) {
        // we cannot reach the start point
        ROS_ERROR_STREAM("MowingBehavior: (FIRST POINT) - Could not reach goal (first point). Planner Status was: "
                         << current_status.state_);
        // we have 3 attempts to get to the start pose of the mowing area
        if (first_point_attempt_counter < config.max_first_point_attempts) {
          ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt " << first_point_attempt_counter << " / "
                                                                     << config.max_first_point_attempts
                                                                     << " Making a little pause ...");
          paused = true;
          update_actions();
        } else {
          // We failed to reach the first point in the mow path by simply repeating the drive to process
          // So now we will trim the path by removing the first pose
          if (first_point_trim_counter < config.max_first_point_trim_attempts) {
            // We try now to remove the first point so the 2nd, 3rd etc point becomes our target
            // mow path points are offset by 10cm
            ROS_WARN_STREAM("MowingBehavior: (FIRST POINT) - Attempt "
                            << first_point_trim_counter << " / " << config.max_first_point_trim_attempts
                            << " Trimming first point off the beginning of the mow path.");
            currentMowingPathIndex++;
            first_point_trim_counter++;
            first_point_attempt_counter = 0;  // give it another <config.max_first_point_attempts> attempts
            paused = true;
            update_actions();
          } else {
            // Unable to reach the start of the mow path (we tried multiple attempts for the same point, and we skipped
            // points which also didnt work, time to give up)
            ROS_ERROR_STREAM(
                "MowingBehavior: (FIRST POINT) Max retries reached, we are unable to reach any of the first points - "
                "aborting at index: "
                << currentMowingPathIndex << " path: " << currentMowingPath << " area: " << currentMowingArea);
            this->abort();
          }
        }
        continue;
      }

      mower_map::ClearNavPointSrv clear_nav_point_srv;
      clearNavPointClient.call(clear_nav_point_srv);

      // we have reached the start pose of the mow area, reset error handling values
      first_point_attempt_counter = 0;
      first_point_trim_counter = 0;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // Execute the path segment and either drop it if we finished it successfully or trim it if we were aborted
    ////////////////////////////////////////////////////////////////////////////////////////////////////////////
    {
      // enable mower (only when we reach the start not on the way to mowing already)
      mowerEnabled = true;

      mbf_msgs::ExePathGoal exePathGoal;
      nav_msgs::Path exePath;
      exePath.header = path.path.header;
      exePath.poses = std::vector<geometry_msgs::PoseStamped>(path.path.poses.begin() + currentMowingPathIndex,
                                                              path.path.poses.end());
      int exePathStartIndex = currentMowingPathIndex;
      exePathGoal.path = exePath;
      exePathGoal.angle_tolerance = 5.0 * (M_PI / 180.0);
      exePathGoal.dist_tolerance = 0.2;
      exePathGoal.tolerance_from_action = true;
      exePathGoal.controller = "FTCPlanner";

      ROS_INFO_STREAM("MowingBehavior: (MOW) First point reached - Executing mow path with "
                      << path.path.poses.size() << " poses, from index " << exePathStartIndex);
      mbfClientExePath->sendGoal(exePathGoal);
      sleep(1);
      actionlib::SimpleClientGoalState current_status(actionlib::SimpleClientGoalState::PENDING);
      ros::Rate r(10);

      // wait for path execution to finish
      while (ros::ok()) {
        current_status = mbfClientExePath->getState();
        if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE ||
            current_status.state_ == actionlib::SimpleClientGoalState::PENDING) {
          // path is being executed, everything seems fine.
          // check if we should pause or abort mowing
          if (skip_area) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) SKIP AREA was requested.");
            // remove all paths in current area and return true
            mowerEnabled = false;
            currentMowingPaths.clear();
            skip_area = false;
            return true;
          }
          if (skip_path) {
            skip_path = false;
            currentMowingPath++;
            currentMowingPathIndex = 0;
            return false;
          }
          if (aborted) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) ABORT was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSE was requested - stopping path execution.");
            mbfClientExePath->cancelAllGoals();
            mowerEnabled = false;
            break;  // Trim path
          }
          if (current_status.state_ == actionlib::SimpleClientGoalState::ACTIVE) {
            // show progress
            int currentIndex = getCurrentMowPathIndex();
            if (currentIndex != -1) {
              currentMowingPathIndex = exePathStartIndex + currentIndex;
            }
            ROS_INFO_STREAM_THROTTLE(
                5, "MowingBehavior: (MOW) Progress: " << currentMowingPathIndex << "/" << path.path.poses.size());
            if (ros::Time::now() - last_checkpoint > ros::Duration(30.0)) checkpoint();
          }
        } else {
          ROS_INFO_STREAM("MowingBehavior: (MOW)  Got status " << current_status.state_
                                                               << " from MBF/FTCPlanner -> Stopping path execution.");
          // we're done, break out of the loop
          break;
        }
        r.sleep();
      }

      // Only skip/trim if goal execution began
      if (current_status.state_ != actionlib::SimpleClientGoalState::PENDING &&
          current_status.state_ != actionlib::SimpleClientGoalState::RECALLED) {
        ROS_INFO_STREAM(">> MowingBehavior: (MOW) PlannerGetProgress currentMowingPathIndex = "
                        << currentMowingPathIndex << " of " << path.path.poses.size());
        printNavState(current_status.state_);
        // if we have fully processed the segment or we have encountered an error, drop the path segment
        /* TODO: we can not trust the SUCCEEDED state because the planner sometimes says suceeded with
            the currentIndex far from the size of the poses ! (BUG in planner ?)
            instead we trust only the currentIndex vs. poses.size() */
        if (currentMowingPathIndex >= path.path.poses.size() ||
            (path.path.poses.size() - currentMowingPathIndex) < 5)  // fully mowed the path ?
        {
          ROS_INFO_STREAM("MowingBehavior: (MOW) Mow path finished, skipping to next mow path.");
          currentMowingPath++;
          currentMowingPathIndex = 0;
          // continue with next segment
        } else {
          // we didnt drive all points in the mow path, so we go into pause mode
          // TODO: we should figure out the likely reason for our failure to complete the path
          // if GPS -> PAUSE
          // if something else -> Recovery Behaviour ?

          // currentMowingPathIndex might be 0 if we never consumed one of the points, we advance at least 1 point
          if (currentMowingPathIndex == 0) currentMowingPathIndex++;
          if (!requested_pause_flag) {
            ROS_INFO_STREAM("MowingBehavior: (MOW) PAUSED due to MBF Error at " << currentMowingPathIndex);
            paused = true;
            update_actions();
          }
        }
      }
    }
  }

  mowerEnabled = false;

  // true, if we have executed all paths
  return currentMowingPath >= currentMowingPaths.size();
}

void MowingBehavior::command_home() {
  if (shared_state->active_semiautomatic_task) {
    // We are in semiautomatic task, mark it as manually paused.
    ROS_INFO_STREAM("Manually pausing semiautomatic task");
    auto config = getConfig();
    config.manual_pause_mowing = true;
    setConfig(config);
  }
  if (paused) {
    // Request continue to wait for odom
    this->requestContinue();
    // Then instantly abort i.e. go to dock.
  }
  this->abort();
}

void MowingBehavior::command_start() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL CONTINUE");
  auto config = getConfig();
  if (shared_state->active_semiautomatic_task && config.manual_pause_mowing) {
    // We are in semiautomatic task and paused, user wants to resume, so store that immediately.
    // This way, once we are docked the mower will continue as soon as all other conditions are g2g
    ROS_INFO_STREAM("Resuming semiautomatic task");
    config.manual_pause_mowing = false;
    setConfig(config);
  }
  this->requestContinue();
}

void MowingBehavior::command_s1() {
  ROS_INFO_STREAM("MowingBehavior: MANUAL PAUSED");
  this->requestPause();
}

void MowingBehavior::command_s2() {
  skip_area = true;
}

bool MowingBehavior::redirect_joystick() {
  return false;
}

uint8_t MowingBehavior::get_sub_state() {
  return 0;
}

uint8_t MowingBehavior::get_state() {
  return mower_msgs::HighLevelStatus::HIGH_LEVEL_STATE_AUTONOMOUS;
}

int16_t MowingBehavior::get_current_area() {
  return currentMowingArea;
}

int16_t MowingBehavior::get_current_path() {
  return currentMowingPath;
}

int16_t MowingBehavior::get_current_path_index() {
  return currentMowingPathIndex;
}

float MowingBehavior::get_current_progress() {
  if (currentMowingPaths.empty()) {
    return 0.0f;
  }
  size_t total_poses = 0;
  size_t done_poses = 0;
  for (size_t i = 0; i < currentMowingPaths.size(); i++) {
    const size_t poses = currentMowingPaths[i].path.poses.size();
    total_poses += poses;
    if (static_cast<int>(i) < currentMowingPath) {
      // Fully mowed paths contribute all of their poses.
      done_poses += poses;
    } else if (static_cast<int>(i) == currentMowingPath) {
      // The path currently being mowed contributes its consumed pose count, clamped so a
      // stale index (the FTC planner can report an index beyond the path on SUCCEEDED)
      // can never push progress above 1.0.
      done_poses += std::min(static_cast<size_t>(std::max(currentMowingPathIndex, 0)), poses);
    }
  }
  if (total_poses == 0) {
    return 0.0f;
  }
  return static_cast<float>(done_poses) / static_cast<float>(total_poses);
}

MowingBehavior::MowingBehavior() {
  last_checkpoint = ros::Time(0.0);
  // No explicit area queue until the scheduler dispatches one. After a crash,
  // restore_checkpoint() rehydrates the area/path progress but not the queue,
  // so recovery falls back to whole-map iteration by design.
  requestedAreaQueuePos = 0;
  requestedFillType = -1;
  requestedSpeed = std::numeric_limits<double>::quiet_NaN();
  requestedTravelSpeed = std::numeric_limits<double>::quiet_NaN();
  requestedAngleDeg = std::numeric_limits<double>::quiet_NaN();
  requestedOutlineCount = -1;
  speedOverridden = false;
  savedSpeedSlow = 0.15;
  savedSpeedFast = 0.4;
  xbot_msgs::ActionInfo pause_action;
  pause_action.action_id = "pause";
  pause_action.enabled = false;
  pause_action.action_name = "Pause Mowing";

  xbot_msgs::ActionInfo continue_action;
  continue_action.action_id = "continue";
  continue_action.enabled = false;
  continue_action.action_name = "Continue Mowing";

  xbot_msgs::ActionInfo abort_mowing_action;
  abort_mowing_action.action_id = "abort_mowing";
  abort_mowing_action.enabled = false;
  abort_mowing_action.action_name = "Stop Mowing";

  xbot_msgs::ActionInfo abort_to_idle_action;
  abort_to_idle_action.action_id = "abort_to_idle";
  abort_to_idle_action.enabled = false;
  abort_to_idle_action.action_name = "Stop";

  xbot_msgs::ActionInfo skip_area_action;
  skip_area_action.action_id = "skip_area";
  skip_area_action.enabled = false;
  skip_area_action.action_name = "Skip Area";

  xbot_msgs::ActionInfo skip_path_action;
  skip_path_action.action_id = "skip_path";
  skip_path_action.enabled = false;
  skip_path_action.action_name = "Skip Path";

  actions.clear();
  actions.push_back(pause_action);
  actions.push_back(continue_action);
  actions.push_back(abort_mowing_action);
  actions.push_back(abort_to_idle_action);
  actions.push_back(skip_area_action);
  actions.push_back(skip_path_action);
  restore_checkpoint();
}

void MowingBehavior::handle_action(std::string action) {
  if (action == "mower_logic:mowing/pause") {
    ROS_INFO_STREAM("got pause command");
    this->requestPause();
  } else if (action == "mower_logic:mowing/continue") {
    ROS_INFO_STREAM("got continue command");
    this->requestContinue();
  } else if (action == "mower_logic:mowing/abort_mowing") {
    ROS_INFO_STREAM("got abort mowing command");
    command_home();
  } else if (action == "mower_logic:mowing/abort_to_idle") {
    // Stop mowing and return to idle without driving to the dock. We must not
    // mark the semiautomatic task as paused (that would auto-resume mowing
    // after the next dock cycle); leaving the active flag intact and aborting
    // is enough — execute() routes us to IdleBehavior via abort_to_idle.
    ROS_INFO_STREAM("got abort to idle command");
    abort_to_idle = true;
    if (paused) {
      this->requestContinue();
    }
    this->abort();
  } else if (action == "mower_logic:mowing/skip_area") {
    ROS_INFO_STREAM("got skip_area command");
    skip_area = true;
  } else if (action == "mower_logic:mowing/skip_path") {
    ROS_INFO_STREAM("got skip_path command");
    skip_path = true;
  }
  update_actions();
}

void MowingBehavior::checkpoint() {
  rosbag::Bag bag;
  mower_logic::CheckPoint cp;
  cp.currentMowingPath = currentMowingPath;
  cp.currentMowingArea = currentMowingArea;
  cp.currentMowingPathIndex = currentMowingPathIndex;
  cp.currentMowingPlanDigest = currentMowingPlanDigest;
  cp.currentMowingAngleIncrementSum = currentMowingAngleIncrementSum;
  bag.open("checkpoint.bag", rosbag::bagmode::Write);
  bag.write("checkpoint", ros::Time::now(), cp);
  bag.close();
  last_checkpoint = ros::Time::now();
}

bool MowingBehavior::restore_checkpoint() {
  rosbag::Bag bag;
  bool found = false;
  try {
    bag.open("checkpoint.bag");
  } catch (rosbag::BagIOException& e) {
    // Checkpoint does not exist or is corrupt, start at the very beginning
    currentMowingArea = 0;
    currentMowingPath = 0;
    currentMowingPathIndex = 0;
    currentMowingAngleIncrementSum = 0;
    return false;
  }
  {
    rosbag::View view(bag, rosbag::TopicQuery("checkpoint"));
    for (rosbag::MessageInstance const m : view) {
      auto cp = m.instantiate<mower_logic::CheckPoint>();
      if (cp) {
        ROS_INFO_STREAM("Restoring checkpoint for plan ("
                        << cp->currentMowingPlanDigest << ")"
                        << " area: " << cp->currentMowingArea << " path: " << cp->currentMowingPath
                        << " index: " << cp->currentMowingPathIndex
                        << " angle increment sum: " << cp->currentMowingAngleIncrementSum);
        currentMowingPath = cp->currentMowingPath;
        currentMowingArea = cp->currentMowingArea;
        currentMowingPathIndex = cp->currentMowingPathIndex;
        currentMowingPlanDigest = cp->currentMowingPlanDigest;
        currentMowingAngleIncrementSum = cp->currentMowingAngleIncrementSum;
        found = true;
        break;
      }
    }
    bag.close();
  }
  return found;
}
