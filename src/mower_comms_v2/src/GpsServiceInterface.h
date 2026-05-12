//
// Created by clemens on 30.11.24.
//

#ifndef GPSSERVICEINTERFACE_H
#define GPSSERVICEINTERFACE_H
#include <ros/publisher.h>
#include <xbot_msgs/AbsolutePose.h>
#include <xbot_msgs/GpsStatus.h>

#include <GpsServiceInterfaceBase.hpp>

class GpsServiceInterface : public GpsServiceInterfaceBase {
 public:
  GpsServiceInterface(uint16_t service_id, const xbot::serviceif::Context& ctx, const ros::Publisher& imu_publisher,
                      const ros::Publisher& nmea_publisher, const ros::Publisher& gps_status_publisher,
                      double datum_lat, double datum_long, double datum_height, uint32_t baud_rate,
                      const std::string& protocol, uint8_t port_index, bool absolute_coords);

  bool OnConfigurationRequested(uint16_t service_id) override;

 protected:
  void OnPositionChanged(const double* new_value, uint32_t length) override;
  void OnPositionHorizontalAccuracyChanged(const double& new_value) override;
  void OnFixTypeChanged(const char* new_value, uint32_t length) override;
  void OnMotionVectorENUChanged(const double* new_value, uint32_t length) override;
  void OnMotionHeadingAndAccuracyChanged(const double* new_value, uint32_t length) override;
  void OnVehicleHeadingAndAccuracyChanged(const double* new_value, uint32_t length) override;

 private:
  void OnTransactionStart(uint64_t timestamp) override;
  void OnTransactionEnd() override;

  const ros::Publisher& absolute_pose_publisher_;
  const ros::Publisher& nmea_publisher_;
  const ros::Publisher& gps_status_publisher_;

  // Last known fix_type on the v2 service. We map the FIX/FLOAT string
  // delivered by the firmware to the same 0..5 scale the frontend expects.
  // numSV / pDOP are not exposed by the xbot_framework GpsService yet — we
  // ship them as 0 so the schema stays uniform across hardware platforms.
  uint8_t last_fix_type_{0};
  // Wall-clock of the most recent position update — if the firmware stops
  // delivering positions (GPS antenna unplugged, robot in a tunnel, etc.) we
  // downgrade fix_type to 0 in OnTransactionEnd so the dashboard does not
  // keep claiming "RTK Fixed" while the receiver is actually blind.
  ros::Time last_position_update_{0};

  std::string protocol_;
  uint32_t baud_rate_;
  uint8_t port_index_;
  bool absolute_coords_;

  xbot_msgs::AbsolutePose pose_msg_{};
  double datum_e_, datum_n_, datum_u_;
  std::string datum_zone_;
  void SendNMEA(double lat_in, double lon_in);
};

#endif  // GPSSERVICEINTERFACE_H
