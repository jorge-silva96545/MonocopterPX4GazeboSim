#include "gz_monospinner_control/ReducedAttitudeController.hpp"

#include <algorithm>
#include <cmath>

namespace monospinner
{

namespace
{
// |n + e_z| below this means n is (nearly) upside down: m is undefined (Eq. 54 note).
constexpr double kMinHalfwayNorm = 1e-6;
// m . m_bar below this means the error is (nearly) 180 deg: Eq. 74 divides by it.
constexpr double kMinAlignment = 1e-3;
}  // namespace

void ReducedAttitudeController::reset()
{
  _integral.setZero();
  _prev_dw.setZero();
  _have_prev = false;
}

Eigen::Vector3d ReducedAttitudeController::normalFromPrecessionNutation(
  double theta1, double theta2)
{
  return {std::cos(theta1) * std::sin(theta2), std::sin(theta1) * std::sin(theta2),
    std::cos(theta2)};
}

ReducedAttitudeOutput ReducedAttitudeController::update(
  const Eigen::Quaterniond & q_world_body, const Eigen::Vector3d & w_body,
  const Eigen::Vector3d & n_bar, double omega, double dt)
{
  ReducedAttitudeOutput out;
  const Eigen::Vector3d ez = Eigen::Vector3d::UnitZ();
  const Eigen::Matrix3d R = q_world_body.normalized().toRotationMatrix();

  out.n = R * ez;
  const Eigen::Vector3d n_sum = out.n + ez;
  const Eigen::Vector3d nbar_sum = n_bar.normalized() + ez;
  if (n_sum.norm() < kMinHalfwayNorm || nbar_sum.norm() < kMinHalfwayNorm) {
    out.singular = true;
  } else {
    out.m = n_sum.normalized();
    out.m_bar = nbar_sum.normalized();
    const double alignment = out.m.dot(out.m_bar);
    if (alignment < kMinAlignment) {
      out.singular = true;
    } else {
      out.w_r = _p.P.cwiseProduct(R.transpose() * out.m.cross(out.m_bar)) / alignment;
    }
  }

  // Rate loop. With w_r = 0 (singular case) this still damps the body rates.
  const Eigen::Vector3d dw = out.w_r - w_body;
  Eigen::Vector3d d_dw = Eigen::Vector3d::Zero();
  if (dt > 0.0) {
    _integral += dw * dt;
    if (_have_prev) {
      d_dw = (dw - _prev_dw) / dt;
    }
  }
  _prev_dw = dw;
  _have_prev = true;
  out.tau_c = _p.Kp.cwiseProduct(dw) + _p.Kd.cwiseProduct(d_dw) + _p.Ki.cwiseProduct(_integral);

  // Torque -> lateral thrust through the lever arm only (Eq. 71): f_c = tau_c x e_z / h_P.
  out.f_c = out.tau_c.cross(ez) / _p.h_P;

  const double f_axial = _p.k_f * omega * omega;
  const double f_lateral = std::hypot(out.f_c.x(), out.f_c.y());
  out.beta = std::atan2(out.f_c.y(), out.f_c.x());
  out.alpha = f_axial > 0.0 ?
    std::min(std::atan(f_lateral / f_axial), _p.alpha_max) :
    0.0;
  out.gamma_dot = omega * std::min(1.0 / std::sqrt(std::cos(out.alpha)), _p.lambda_max);
  out.thrust = _p.k_f * out.gamma_dot * out.gamma_dot;
  return out;
}

}  // namespace monospinner
