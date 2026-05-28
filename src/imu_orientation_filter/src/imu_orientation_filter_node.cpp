// imu_orientation_filter_node
//
// Fuses the OpenMower low-level IMU (gyroscope + accelerometer) into a
// Madgwick 6-DOF orientation estimate and publishes it as a fully
// populated sensor_msgs/Imu. The yaw-axis null-space inherent to a
// magnetometer-less filter is closed by gently slerping toward the
// vehicle heading reported by xbot_positioning whenever the RTK fix is
// trustworthy. Mounting offsets and gyro bias from imu.calibrate_level
// are applied here at the point where they have the cleanest semantics
// (bias before integration, mounting at the output).
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <tf2/LinearMath/Quaternion.h>
#include <xbot_msgs/AbsolutePose.h>

#include <cmath>

#include "madgwick_ahrs.h"

namespace {

constexpr double kDefaultBeta = 0.1;
constexpr double kDefaultYawBlend = 0.05;
// Reject xbot_positioning samples whose orientation accuracy is worse
// than this; otherwise a noisy heading would constantly nudge the filter
// the wrong way. 0.2 rad ~= 11 degrees is generous enough to track on
// RTK float and still cuts off pure dead-reckoning fallbacks.
constexpr double kDefaultMaxYawAccuracyRad = 0.2;
// Hard cap on dt so a paused sensor or a debugger break does not unleash
// an unbounded gyro integration step on resume.
constexpr double kMaxDtSeconds = 0.1;

}  // namespace

class ImuOrientationFilterNode {
 public:
  ImuOrientationFilterNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh) {
    pnh_.param("beta", beta_, kDefaultBeta);
    pnh_.param("yaw_blend", yaw_blend_, kDefaultYawBlend);
    pnh_.param("max_yaw_accuracy_rad", max_yaw_accuracy_rad_, kDefaultMaxYawAccuracyRad);
    pnh_.param("frame_id", frame_id_, std::string("base_link"));

    // Mounting offsets and gyro bias live under ll/services/imu so they
    // sit next to axis_config in the existing config schema.
    nh_.param("ll/services/imu/mounting_roll_offset_rad", mounting_roll_offset_, 0.0);
    nh_.param("ll/services/imu/mounting_pitch_offset_rad", mounting_pitch_offset_, 0.0);
    nh_.param("ll/services/imu/gyro_bias/x", gyro_bias_x_, 0.0);
    nh_.param("ll/services/imu/gyro_bias/y", gyro_bias_y_, 0.0);
    nh_.param("ll/services/imu/gyro_bias/z", gyro_bias_z_, 0.0);

    filter_.setBeta(beta_);

    imu_pub_ = nh_.advertise<sensor_msgs::Imu>("imu/data", 10);
    imu_sub_ = nh_.subscribe("ll/imu/data_raw", 50, &ImuOrientationFilterNode::onImu, this);
    pose_sub_ = nh_.subscribe("xbot_positioning/xb_pose", 5, &ImuOrientationFilterNode::onPose, this);

    // Re-read the mounting/bias params at 1 Hz so a successful imu.calibrate_level
    // RPC takes effect without a node restart. The cost is negligible (five
    // param-server reads per second) and the lag is well below human perception.
    param_reload_timer_ = nh_.createTimer(ros::Duration(1.0), &ImuOrientationFilterNode::reloadCalibrationParams, this);

    ROS_INFO_STREAM("imu_orientation_filter started (beta=" << beta_ << ", yaw_blend=" << yaw_blend_
                                                            << ", max_yaw_acc=" << max_yaw_accuracy_rad_ << " rad)");
    if (mounting_roll_offset_ != 0.0 || mounting_pitch_offset_ != 0.0) {
      ROS_INFO("Applying mounting offset roll=%.4f rad, pitch=%.4f rad", mounting_roll_offset_, mounting_pitch_offset_);
    }
    if (gyro_bias_x_ != 0.0 || gyro_bias_y_ != 0.0 || gyro_bias_z_ != 0.0) {
      ROS_INFO("Applying gyro bias [%.5f, %.5f, %.5f] rad/s", gyro_bias_x_, gyro_bias_y_, gyro_bias_z_);
    }
  }

 private:
  void onImu(const sensor_msgs::Imu::ConstPtr& msg) {
    const ros::Time stamp = msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;

    // Bias correction: subtract from the input so the integrated state
    // is bias-free. The same correction would be wrong at the output
    // because the filter would already have integrated the bias.
    const double gx = msg->angular_velocity.x - gyro_bias_x_;
    const double gy = msg->angular_velocity.y - gyro_bias_y_;
    const double gz = msg->angular_velocity.z - gyro_bias_z_;

    const double ax = msg->linear_acceleration.x;
    const double ay = msg->linear_acceleration.y;
    const double az = msg->linear_acceleration.z;

    if (!have_first_imu_) {
      // Bootstrap with the gravity vector immediately so the model on
      // the UI side does not visibly converge from identity. If we have
      // already received a yaw from xbot_positioning use it, otherwise
      // start at zero yaw and let correctYaw() pull it in once a fix
      // arrives.
      const double yaw0 = have_pose_yaw_ ? latest_pose_yaw_ : 0.0;
      filter_.initFromAccelAndYaw(ax, ay, az, yaw0);
      last_imu_stamp_ = stamp;
      have_first_imu_ = true;
      return;
    }

    double dt = (stamp - last_imu_stamp_).toSec();
    last_imu_stamp_ = stamp;
    if (dt <= 0.0) {
      return;
    }
    if (dt > kMaxDtSeconds) {
      dt = kMaxDtSeconds;
    }

    filter_.update(gx, gy, gz, ax, ay, az, dt);

    publish(msg, stamp);
  }

  void reloadCalibrationParams(const ros::TimerEvent&) {
    double roll = mounting_roll_offset_;
    double pitch = mounting_pitch_offset_;
    double bx = gyro_bias_x_;
    double by = gyro_bias_y_;
    double bz = gyro_bias_z_;
    nh_.getParam("ll/services/imu/mounting_roll_offset_rad", roll);
    nh_.getParam("ll/services/imu/mounting_pitch_offset_rad", pitch);
    nh_.getParam("ll/services/imu/gyro_bias/x", bx);
    nh_.getParam("ll/services/imu/gyro_bias/y", by);
    nh_.getParam("ll/services/imu/gyro_bias/z", bz);
    if (roll != mounting_roll_offset_ || pitch != mounting_pitch_offset_ || bx != gyro_bias_x_ || by != gyro_bias_y_ ||
        bz != gyro_bias_z_) {
      mounting_roll_offset_ = roll;
      mounting_pitch_offset_ = pitch;
      gyro_bias_x_ = bx;
      gyro_bias_y_ = by;
      gyro_bias_z_ = bz;
      ROS_INFO("imu_orientation_filter: reloaded calibration (roll=%.4f pitch=%.4f bias=[%.5f, %.5f, %.5f])", roll,
               pitch, bx, by, bz);
    }
  }

  void onPose(const xbot_msgs::AbsolutePose::ConstPtr& msg) {
    if (!msg->orientation_valid) {
      return;
    }
    if (msg->orientation_accuracy <= 0.0 || msg->orientation_accuracy > max_yaw_accuracy_rad_) {
      return;
    }
    latest_pose_yaw_ = msg->vehicle_heading;
    have_pose_yaw_ = true;
    if (have_first_imu_) {
      filter_.correctYaw(latest_pose_yaw_, yaw_blend_);
    }
  }

  void publish(const sensor_msgs::Imu::ConstPtr& src, const ros::Time& stamp) {
    sensor_msgs::Imu out;
    out.header.stamp = stamp;
    out.header.frame_id = frame_id_;

    // Linear acceleration and angular velocity are passed through
    // unchanged; the bias correction is internal to the orientation
    // estimator and we want consumers (charts, RViz) to see the real
    // sensor values.
    out.linear_acceleration = src->linear_acceleration;
    out.angular_velocity = src->angular_velocity;
    out.linear_acceleration_covariance = src->linear_acceleration_covariance;
    out.angular_velocity_covariance = src->angular_velocity_covariance;

    // Build the orientation quaternion and apply the mounting offset as
    // a body-frame Euler pre-rotation. mounting_offset_world = q_filter
    // * q_offset^-1 in body frame removes the constant tilt of the
    // sensor relative to the chassis: when the mower stands level the
    // published orientation reads zero roll/pitch even though the chip
    // is mounted skewed.
    tf2::Quaternion q_filter(filter_.qx(), filter_.qy(), filter_.qz(), filter_.qw());
    if (mounting_roll_offset_ != 0.0 || mounting_pitch_offset_ != 0.0) {
      tf2::Quaternion q_offset;
      q_offset.setRPY(mounting_roll_offset_, mounting_pitch_offset_, 0.0);
      q_filter = q_filter * q_offset.inverse();
      q_filter.normalize();
    }

    out.orientation.w = q_filter.w();
    out.orientation.x = q_filter.x();
    out.orientation.y = q_filter.y();
    out.orientation.z = q_filter.z();

    // Mark the orientation as valid (non-zero diagonal). We do not have
    // a rigorous covariance estimate for the Madgwick output, so use a
    // conservative isotropic value derived from beta and the yaw blend
    // factor; downstream consumers that care can tighten it later.
    const double rp_var = std::max(0.001, beta_ * beta_ * 0.5);
    const double y_var = std::max(0.005, max_yaw_accuracy_rad_ * max_yaw_accuracy_rad_);
    out.orientation_covariance[0] = rp_var;
    out.orientation_covariance[4] = rp_var;
    out.orientation_covariance[8] = y_var;

    imu_pub_.publish(out);
  }

  ros::NodeHandle& nh_;
  ros::NodeHandle& pnh_;
  ros::Publisher imu_pub_;
  ros::Subscriber imu_sub_;
  ros::Subscriber pose_sub_;
  ros::Timer param_reload_timer_;

  imu_orientation_filter::MadgwickAhrs filter_;

  double beta_;
  double yaw_blend_;
  double max_yaw_accuracy_rad_;
  std::string frame_id_;

  double mounting_roll_offset_;
  double mounting_pitch_offset_;
  double gyro_bias_x_;
  double gyro_bias_y_;
  double gyro_bias_z_;

  ros::Time last_imu_stamp_;
  bool have_first_imu_ = false;

  double latest_pose_yaw_ = 0.0;
  bool have_pose_yaw_ = false;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_orientation_filter");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");
  ImuOrientationFilterNode node(nh, pnh);
  ros::spin();
  return 0;
}
