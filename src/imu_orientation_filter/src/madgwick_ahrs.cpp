#include "madgwick_ahrs.h"

#include <cmath>

namespace imu_orientation_filter {

MadgwickAhrs::MadgwickAhrs() : q0_(1.0), q1_(0.0), q2_(0.0), q3_(0.0), beta_(0.1), initialised_(false) {}

void MadgwickAhrs::setBeta(double beta) {
  beta_ = beta;
}

void MadgwickAhrs::reset() {
  q0_ = 1.0;
  q1_ = 0.0;
  q2_ = 0.0;
  q3_ = 0.0;
  initialised_ = false;
}

void MadgwickAhrs::initFromAccelAndYaw(double ax, double ay, double az, double yaw) {
  // Roll/pitch from gravity vector, yaw from external source. We use
  // the standard ZYX (yaw-pitch-roll) Tait-Bryan convention to map back
  // to a quaternion.
  const double norm = std::sqrt(ax * ax + ay * ay + az * az);
  if (norm < 1e-6) {
    // No gravity signal yet; keep identity and let update() take over.
    q0_ = 1.0;
    q1_ = 0.0;
    q2_ = 0.0;
    q3_ = 0.0;
    initialised_ = true;
    return;
  }
  const double ax_n = ax / norm;
  const double ay_n = ay / norm;
  const double az_n = az / norm;

  const double roll = std::atan2(ay_n, az_n);
  const double pitch = std::atan2(-ax_n, std::sqrt(ay_n * ay_n + az_n * az_n));

  const double cy = std::cos(yaw * 0.5);
  const double sy = std::sin(yaw * 0.5);
  const double cp = std::cos(pitch * 0.5);
  const double sp = std::sin(pitch * 0.5);
  const double cr = std::cos(roll * 0.5);
  const double sr = std::sin(roll * 0.5);

  q0_ = cr * cp * cy + sr * sp * sy;
  q1_ = sr * cp * cy - cr * sp * sy;
  q2_ = cr * sp * cy + sr * cp * sy;
  q3_ = cr * cp * sy - sr * sp * cy;
  normalise();
  initialised_ = true;
}

void MadgwickAhrs::update(double gx, double gy, double gz, double ax, double ay, double az, double dt) {
  if (dt <= 0.0) {
    return;
  }

  // Rate of change of quaternion from gyroscope.
  double qDot1 = 0.5 * (-q1_ * gx - q2_ * gy - q3_ * gz);
  double qDot2 = 0.5 * (q0_ * gx + q2_ * gz - q3_ * gy);
  double qDot3 = 0.5 * (q0_ * gy - q1_ * gz + q3_ * gx);
  double qDot4 = 0.5 * (q0_ * gz + q1_ * gy - q2_ * gx);

  // Compute feedback only if accelerometer measurement is valid (avoids
  // NaN in normalisation when the sensor is zeroed out at startup).
  const double a_norm_sq = ax * ax + ay * ay + az * az;
  if (a_norm_sq > 1e-12) {
    const double a_norm = std::sqrt(a_norm_sq);
    const double ax_n = ax / a_norm;
    const double ay_n = ay / a_norm;
    const double az_n = az / a_norm;

    // Auxiliary variables to avoid repeated arithmetic.
    const double _2q0 = 2.0 * q0_;
    const double _2q1 = 2.0 * q1_;
    const double _2q2 = 2.0 * q2_;
    const double _2q3 = 2.0 * q3_;
    const double _4q0 = 4.0 * q0_;
    const double _4q1 = 4.0 * q1_;
    const double _4q2 = 4.0 * q2_;
    const double _8q1 = 8.0 * q1_;
    const double _8q2 = 8.0 * q2_;
    const double q0q0 = q0_ * q0_;
    const double q1q1 = q1_ * q1_;
    const double q2q2 = q2_ * q2_;
    const double q3q3 = q3_ * q3_;

    // Gradient descent algorithm corrective step.
    double s0 = _4q0 * q2q2 + _2q2 * ax_n + _4q0 * q1q1 - _2q1 * ay_n;
    double s1 = _4q1 * q3q3 - _2q3 * ax_n + 4.0 * q0q0 * q1_ - _2q0 * ay_n - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 +
                _4q1 * az_n;
    double s2 = 4.0 * q0q0 * q2_ + _2q0 * ax_n + _4q2 * q3q3 - _2q3 * ay_n - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 +
                _4q2 * az_n;
    double s3 = 4.0 * q1q1 * q3_ - _2q1 * ax_n + 4.0 * q2q2 * q3_ - _2q2 * ay_n;

    const double s_norm_sq = s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3;
    if (s_norm_sq > 1e-12) {
      const double s_norm = std::sqrt(s_norm_sq);
      s0 /= s_norm;
      s1 /= s_norm;
      s2 /= s_norm;
      s3 /= s_norm;

      qDot1 -= beta_ * s0;
      qDot2 -= beta_ * s1;
      qDot3 -= beta_ * s2;
      qDot4 -= beta_ * s3;
    }
  }

  q0_ += qDot1 * dt;
  q1_ += qDot2 * dt;
  q2_ += qDot3 * dt;
  q3_ += qDot4 * dt;

  normalise();
  initialised_ = true;
}

void MadgwickAhrs::correctYaw(double target_yaw, double blend) {
  if (blend <= 0.0) {
    return;
  }
  if (blend > 1.0) {
    blend = 1.0;
  }

  // Extract current yaw (ZYX convention) directly from quaternion.
  const double siny_cosp = 2.0 * (q0_ * q3_ + q1_ * q2_);
  const double cosy_cosp = 1.0 - 2.0 * (q2_ * q2_ + q3_ * q3_);
  const double current_yaw = std::atan2(siny_cosp, cosy_cosp);

  // Smallest signed angular difference, wrapped to (-pi, pi].
  double err = target_yaw - current_yaw;
  while (err > M_PI) err -= 2.0 * M_PI;
  while (err < -M_PI) err += 2.0 * M_PI;

  const double half_angle = 0.5 * blend * err;
  const double cz = std::cos(half_angle);
  const double sz = std::sin(half_angle);

  // Pre-multiply q by a pure yaw quaternion (0, 0, sin, cos) so the
  // correction acts in the world frame and leaves roll/pitch untouched.
  const double nw = cz * q0_ - sz * q3_;
  const double nx = cz * q1_ - sz * q2_;
  const double ny = cz * q2_ + sz * q1_;
  const double nz = cz * q3_ + sz * q0_;

  q0_ = nw;
  q1_ = nx;
  q2_ = ny;
  q3_ = nz;
  normalise();
}

void MadgwickAhrs::normalise() {
  const double n = std::sqrt(q0_ * q0_ + q1_ * q1_ + q2_ * q2_ + q3_ * q3_);
  if (n < 1e-12) {
    q0_ = 1.0;
    q1_ = 0.0;
    q2_ = 0.0;
    q3_ = 0.0;
    return;
  }
  const double inv = 1.0 / n;
  q0_ *= inv;
  q1_ *= inv;
  q2_ *= inv;
  q3_ *= inv;
}

}  // namespace imu_orientation_filter
