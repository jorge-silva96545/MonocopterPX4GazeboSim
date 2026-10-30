// Reduced-attitude controller of Bernardes, Boyer & Viollet, "Modelling, control and
// simulation of a single rotor UAV with swashplateless torque modulation" (HAL
// hal-04139662), Sec. 6 and 7.1. Pure math: no Gazebo, no ROS, so the same class can be
// moved into a PX4 ROS 2 mode later without changes.
//
// Frames: world ENU, body FLU (z up), as in the paper and in ThrustVectorActuator. The
// caller converts PX4's NED/FRD estimate before calling update().
//
// Control law, one call per control step:
//   n     = R e_z                                   body up axis in world      (Eq. 52)
//   m     = (n + e_z) / |n + e_z|                   middle normal vector       (Eq. 54)
//   m_bar = (n_bar + e_z) / |n_bar + e_z|           same, for the setpoint     (Eq. 94)
//   w_r   = P R^T (m x m_bar) / (m . m_bar)         desired body rate          (Eq. 74)
//   dw    = w_r - w                                                           (Eq. 68)
//   tau_c = Kp dw + Kd d(dw)/dt + Ki int(dw) dt     rate PID                   (Eq. 69)
//   f_c   = (1/h_P) tau_c x e_z                     lateral thrust             (Eq. 71)
//   beta  = atan2(f_c,y, f_c,x)                                               (Eq. 80)
//   alpha = min(atan(|f_c| / (k_f Omega^2)), alpha_max)                       (Eq. 80)
//   gamma_dot = Omega min(1/sqrt(cos alpha), Lambda)                          (Eq. 80)
//   thrust    = k_f gamma_dot^2                                               (Eq. 81)
// Omega is the rotor speed giving the desired body-axis thrust (sqrt(m g / k_f) in hover,
// Eq. 79); the caller passes it in so a takeoff profile can scale it.
//
// Reconstruction note: Eq. 67/74 were partly garbled in the PDF text extraction. R^T is
// applied to the cross product (so w_r is a body-frame rate), and the w_s term of Eq. 67 is
// dropped as in Eq. 74 -- it lies along e_z and is removed by the "x e_z" of Eq. 71 anyway.
// The sign was checked by hand: a small tilt about world x gives w_r opposing it.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace monospinner
{

struct ReducedAttitudeParams
{
  Eigen::Vector3d P{1.0, 1.0, 1.0};      // attitude gain, diagonal           (Table 2: I)
  Eigen::Vector3d Kp{0.7, 0.7, 0.7};     // rate P gain, diagonal [N m s/rad] (Table 2)
  Eigen::Vector3d Kd{0.0, 0.0, 0.0};     // rate D gain, diagonal             (Table 2)
  Eigen::Vector3d Ki{0.0, 0.0, 0.0};     // rate I gain, diagonal             (Table 2)
  double h_P{0.083};                     // rotor height above F_B [m]        (Table 1)
  double k_f{5.24e-6};                   // thrust coefficient [kg m]         (Table 1)
  double alpha_max{0.5235987755982988};  // tilt limit, 30 deg [rad]          (Sec. 7.1)
  double lambda_max{1.2};                // rotor speed limit ratio Lambda    (Sec. 7.1)
};

struct ReducedAttitudeOutput
{
  double alpha{0.0};       // [rad] thrust tilt from body z
  double beta{0.0};        // [rad] tilt direction in body x-y, from body x towards body y
  double gamma_dot{0.0};   // [rad/s] rotor speed magnitude
  double thrust{0.0};      // [N] k_f gamma_dot^2
  Eigen::Vector3d n{0, 0, 1};      // body up axis, world
  Eigen::Vector3d m{0, 0, 1};      // middle normal vector, world
  Eigen::Vector3d m_bar{0, 0, 1};  // setpoint middle normal vector, world
  Eigen::Vector3d w_r{0, 0, 0};    // desired body rate, body
  Eigen::Vector3d tau_c{0, 0, 0};  // commanded torque, body
  Eigen::Vector3d f_c{0, 0, 0};    // lateral thrust, body
  bool singular{false};            // n or n_bar near -e_z, or m perpendicular to m_bar
};

class ReducedAttitudeController
{
public:
  explicit ReducedAttitudeController(const ReducedAttitudeParams & params = {})
  : _p(params) {}

  const ReducedAttitudeParams & params() const {return _p;}

  /// Forget the PID memory (integral, previous error for the derivative).
  void reset();

  /// \param q_world_body body(FLU) -> world(ENU) attitude
  /// \param w_body       body angular velocity, FLU [rad/s]
  /// \param n_bar        desired body up axis in world, unit vector
  /// \param omega        rotor speed for the desired body-axis thrust [rad/s]
  /// \param dt           time since the previous update [s]; <= 0 skips the D/I terms
  ReducedAttitudeOutput update(
    const Eigen::Quaterniond & q_world_body, const Eigen::Vector3d & w_body,
    const Eigen::Vector3d & n_bar, double omega, double dt);

  /// Desired body up axis from precession theta1 (tilt direction, from world x towards
  /// world y) and nutation theta2 (tilt magnitude), paper Eq. 93.
  static Eigen::Vector3d normalFromPrecessionNutation(double theta1, double theta2);

private:
  ReducedAttitudeParams _p;
  Eigen::Vector3d _integral{Eigen::Vector3d::Zero()};
  Eigen::Vector3d _prev_dw{Eigen::Vector3d::Zero()};
  bool _have_prev{false};
};

}  // namespace monospinner
