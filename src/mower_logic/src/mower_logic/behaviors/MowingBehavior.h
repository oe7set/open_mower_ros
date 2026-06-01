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
#ifndef SRC_MOWINGBEHAVIOR_H
#define SRC_MOWINGBEHAVIOR_H

#include "Behavior.h"
#include "UndockingBehavior.h"
#include "ftc_local_planner/PlannerGetProgress.h"
#include "slic3r_coverage_planner/Path.h"
#include "slic3r_coverage_planner/PlanPath.h"
#include "xbot_msgs/ActionInfo.h"

class MowingBehavior : public Behavior {
 private:
  std::vector<xbot_msgs::ActionInfo> actions;

  bool skip_area;
  bool skip_path;
  // Set when the user requested an abort that should stop in place (transition
  // back to IdleBehavior) instead of the default behavior of going to dock.
  bool abort_to_idle;
  bool create_mowing_plan(int area_index);

  bool execute_mowing_plan();

  // Per-run dispatch latched from the scheduler's /mower_logic/next_run param
  // block on a fresh run (signalled by next_run/pending). The values persist
  // for the whole run — including resumes after a charge dock/undock cycle —
  // and are reset by reset() or a manual map.start_in_area start so they never
  // leak into a subsequent manual run.
  //
  // requestedAreaQueue: explicit area list. When non-empty, the mowing loop
  // walks exactly these indices in order; empty keeps the legacy "mow all
  // active areas" behavior (24/7 mode and manual whole-map starts).
  std::vector<int> requestedAreaQueue;
  size_t requestedAreaQueuePos;
  int requestedFillType;      // slic3r fill enum; -1 = use default_mow_pattern
  double requestedSpeed;      // m/s; NaN = use the global FTC planner speed
  double requestedAngleDeg;   // absolute degrees; NaN = use global/auto angle
  // Correlation id for the scheduler-dispatched run, echoed into mowing
  // lifecycle events so the scheduler can tie failures back to the occurrence
  // that triggered them. Empty for manual runs.
  std::string requestedRunId;

  // Speed-override bookkeeping so exit() can restore the FTC planner's original
  // speeds (pushed via dynamic_reconfigure in enter()).
  bool speedOverridden;
  double savedSpeedSlow;
  double savedSpeedFast;

  // Consume the next_run param block into the requested* members when a fresh
  // scheduled run is pending; reset the overrides for a manual start.
  void consume_next_run();
  // Push / restore the FTC planner mowing speed via dynamic_reconfigure.
  void apply_speed_override();
  void restore_speed_override();

  // Advance currentMowingArea to the next area to mow. Returns false when the
  // explicit queue is exhausted (caller should finish and dock); the legacy
  // whole-map path never returns false (it relies on create_mowing_plan
  // failing on an out-of-range index to terminate).
  bool advance_to_next_area();

  // Progress
  bool mowerEnabled = false;
  std::vector<slic3r_coverage_planner::Path> currentMowingPaths;

  ros::Time last_checkpoint;
  int currentMowingPath;
  int currentMowingArea;
  int currentMowingPathIndex;
  std::string currentMowingPlanDigest;
  double currentMowingAngleIncrementSum;

 public:
  MowingBehavior();

  static MowingBehavior INSTANCE;

  std::string state_name() override;

  Behavior* execute() override;

  void enter() override;

  void exit() override;

  void reset() override;

  bool needs_gps() override;

  bool mower_enabled() override;

  void command_home() override;

  void command_start() override;

  void command_s1() override;

  void command_s2() override;

  bool redirect_joystick() override;

  uint8_t get_sub_state() override;

  uint8_t get_state() override;

  int16_t get_current_area();

  int16_t get_current_path();

  int16_t get_current_path_index();

  void handle_action(std::string action) override;

  void update_actions();

  void checkpoint();

  bool restore_checkpoint();
};

#endif  // SRC_MOWINGBEHAVIOR_H
