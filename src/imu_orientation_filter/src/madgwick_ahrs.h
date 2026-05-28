// Madgwick 6-DOF AHRS algorithm. Public domain reference implementation
// (Sebastian Madgwick, https://x-io.co.uk/), reformulated into a self-
// contained class with explicit double precision and an additional yaw-
// correction step that blends an externally measured world-frame yaw
// (e.g. RTK-GNSS heading) into the filter state without disturbing roll
// and pitch.
#ifndef IMU_ORIENTATION_FILTER_MADGWICK_AHRS_H
#define IMU_ORIENTATION_FILTER_MADGWICK_AHRS_H

namespace imu_orientation_filter {

class MadgwickAhrs {
 public:
  MadgwickAhrs();

  // Set the algorithm gain. Higher beta tracks the accelerometer more
  // aggressively (faster convergence, more sensitivity to linear
  // acceleration); typical values are 0.05 to 0.2.
  void setBeta(double beta);

  // Reset the quaternion to identity and mark the filter as needing a
  // fresh initial alignment from the next accelerometer sample.
  void reset();

  // Initialise the orientation directly from a stationary accelerometer
  // reading and an external yaw (radians, world frame). Use this on the
  // first sample so the visualisation does not have to wait for the
  // filter to converge from identity.
  void initFromAccelAndYaw(double ax, double ay, double az, double yaw);

  // Single update step. Inputs are body-frame angular velocity in rad/s
  // and linear acceleration in m/s^2 (gravity included, sign convention
  // a = +g when at rest with body z pointing up). dt is the elapsed time
  // since the previous update in seconds.
  void update(double gx, double gy, double gz, double ax, double ay, double az, double dt);

  // Blend a known world-frame yaw into the current orientation. The
  // blend factor (0..1) controls how aggressively the correction is
  // applied per call; small values (0.02..0.1) give a smooth visual
  // pull without observable jumps. Roll and pitch are preserved.
  void correctYaw(double target_yaw, double blend);

  // Output accessors. Quaternion order is (w, x, y, z), Hamilton
  // convention, body-to-world.
  double qw() const { return q0_; }
  double qx() const { return q1_; }
  double qy() const { return q2_; }
  double qz() const { return q3_; }

 private:
  void normalise();

  double q0_, q1_, q2_, q3_;
  double beta_;
  bool initialised_;
};

}  // namespace imu_orientation_filter

#endif  // IMU_ORIENTATION_FILTER_MADGWICK_AHRS_H
