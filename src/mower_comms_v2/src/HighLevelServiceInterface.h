/*
 * OpenMower V2 Firmware
 * Part of open_mower_ros (https://github.com/ClemensElflein/open_mower_ros)
 *
 * Copyright (C) 2026 The OpenMower Contributors
 *
 * SPDX-License-Identifier: GPL-3.0-or-later
 */

/**
 * @file HighLevelServiceInterface.h
 * @brief Service interface for high-level mower status communication
 * @author Apehaenger <joerg@ebeling.ws>
 * @date 2026-01-10
 */

#ifndef HIGHLEVELSERVICEINTERFACE_H
#define HIGHLEVELSERVICEINTERFACE_H

#include <mower_msgs/HighLevelStatus.h>
#include <ros/ros.h>
#include <std_msgs/String.h>

#include <mutex>
#include <string>

#include <HighLevelServiceInterfaceBase.hpp>

class HighLevelServiceInterface : public HighLevelServiceInterfaceBase {
 public:
  HighLevelServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx)
      : HighLevelServiceInterfaceBase(service_id, ctx) {
    ros::NodeHandle nh;
    high_level_status_sub_ =
        nh.subscribe("mower_logic/current_state", 1, &HighLevelServiceInterface::highLevelStatusReceived, this);
    // Latched so that subscribers (xbot_monitoring) which connect after the
    // firmware sent its version still receive the cached value.
    firmware_version_pub_ = nh.advertise<std_msgs::String>("mower_comms_v2/firmware_version", 1, true);
  }

 protected:
  // Called when the firmware sends an output (Action string)
  void OnActionChanged(const char* new_value, uint32_t length) override;

  // Firmware build identifiers, sent once per claim by the HighLevelService on
  // the mainboard. Both arrive as separate messages; we publish the combined
  // JSON once both have been seen.
  void OnFirmwareGitHashChanged(const char* new_value, uint32_t length) override;
  void OnFirmwareBuildDateChanged(const char* new_value, uint32_t length) override;

 private:
  void highLevelStatusReceived(const mower_msgs::HighLevelStatus::ConstPtr& msg);
  void publishFirmwareVersionIfReady();

  ros::Subscriber high_level_status_sub_;
  ros::Publisher firmware_version_pub_;

  std::mutex firmware_version_mutex_;
  std::string firmware_git_hash_;
  std::string firmware_build_date_;
};

#endif  // HIGHLEVELSERVICEINTERFACE_H
