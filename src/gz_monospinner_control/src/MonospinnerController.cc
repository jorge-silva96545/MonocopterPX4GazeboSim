// gz-sim system that closes the mono-spinner's attitude loop on PX4's EKF2 estimate, or on
// the true state (<state_source>).
//
//   <state_source>ekf2</state_source> (default):
//   PX4 EKF2 --uXRCE-DDS--> /fmu/out/vehicle_odometry (px4_msgs/VehicleOdometry, ROS 2)
//        --> this system (embedded rclcpp node; ReducedAttitudeController, paper Sec. 6)
//        --> /<model>/command/motor_speed (gz.msgs.Actuators, gz-transport)
//        --> ThrustVectorActuator
//
//   <state_source>ground_truth</state_source>:
//   <link_name>'s pose and angular velocity read from the ECM in this system's own
//   PreUpdate -- the exact state of the current step, no transport, no delay, no ROS -- then
//   the same controller and the same command topic. (The /model/<model>/odometry topic is
//   published from these same ECM values by OdometryPublisher; reading them here skips that
//   hop.) EKF2 keeps running in PX4 but no longer affects the flight, so it can be observed
//   as a passenger. The command still crosses gz-transport, so the loop is deterministic on
//   its input side only.
//
// NOT LOCKSTEPPED (ekf2 mode). The estimate arrives over DDS and the command leaves over gz-transport;
// neither is tied to a physics step (see the discussion in the package README). The age of
// every estimate is measured against sim time and published on the debug topic, and the
// attitude is optionally propagated forward by that age (<predict_attitude>), because the
// body spins at ~18 rad/s: 10 ms of lag rotates the commanded tilt direction by ~10 deg.
//
// Command format: exactly what ThrustVectorActuator::PreUpdate reads today
// (src/gz_monospinner_plugin/src/ThrustVectorActuator.cc):
//   position = [alpha, beta]  [rad]   thrust tilt and its direction, body FLU
//   velocity = [thrust]       [N]
// It is NOT the normalized channel mapping described in CLAUDE.md -- that decoding is still
// a TODO in the actuator plugin, which this package does not touch.
//
// Frames: PX4 reports body(FRD) -> NED attitude and FRD body rates (VehicleOdometry.msg).
// They are converted to body(FLU) -> ENU, the inverse of analysis/frames.py:
// gz_quat_to_px4() (which reproduces PX4-Autopilot/src/modules/simulation/gz_bridge/
// GZBridge.cpp:934-950):
//   q_enu_flu = q_enu_ned * q_ned_frd * q_frd_flu,   w_flu = (wx, -wy, -wz)
// Timestamps: with UXRCE_DDS_SYNCT 0 (config/monospinner.params) PX4 stamps /fmu/out/* with
// hrt_absolute_time(), which follows gz sim time (GZBridge::clockCallback,
// GZBridge.cpp:331-345), so estimate age = sim time - timestamp_sample.
//
// Sequence: once estimates arrive, wait <start_delay>, then a takeoff profile scales the
// hover rotor speed (rotor thrust (1 + a) m g for <takeoff_time>, then (1 - a) m g for the
// same time, then m g), with the attitude loop active throughout. Hover rotor speed is
// sqrt(m g / k_f) (paper Eq. 79), m and g taken from the simulated model and world unless
// <mass>/<gravity> override them. No altitude or position control, as in the paper.
//
// Debug topic (gz.msgs.Double_V, one message per control update), indices:
//    0 t_sim [s]                1 input sample time [s]           2 input age [s] (0 for ground truth)
//    3 phase (0 wait, 1 takeoff up, 2 takeoff brake, 3 hover, -1 estimate timeout)
//    4 Omega [rad/s]            5 alpha [rad]   6 beta [rad]   7 gamma_dot [rad/s]
//    8 thrust [N]               9-11 n (input, world ENU)      12-14 n_bar (world ENU)
//   15-17 w input (body FLU)    18-20 w_r (body FLU)           21-23 tau_c (body FLU)
//   24 singular (0/1)           25-28 q_enu_flu used by the controller (w, x, y, z)
//
// Setpoint topic (gz.msgs.Vector2d): x = precession theta1, y = nutation theta2 [rad]
// (paper Eq. 93): tilt direction from world x (east) towards world y (north), tilt magnitude.

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <gz/common/Console.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/msgs/double_v.pb.h>
#include <gz/msgs/vector2d.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>
#include <gz/sim/components/Gravity.hh>
#include <gz/sim/components/Inertial.hh>
#include <gz/transport/Node.hh>

#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <rclcpp/rclcpp.hpp>

#include "gz_monospinner_control/ReducedAttitudeController.hpp"

namespace monospinner
{

class MonospinnerController
  : public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  ~MonospinnerController() override;

  void Configure(
    const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
    gz::sim::EntityComponentManager & ecm, gz::sim::EventManager & event_mgr) override;

  void PreUpdate(
    const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm) override;

private:
  struct Estimate
  {
    Eigen::Quaterniond q_ned_frd{Eigen::Quaterniond::Identity()};
    Eigen::Vector3d w_frd{Eigen::Vector3d::Zero()};
    double t_sample{0.0};  // [s], PX4 hrt = gz sim time
  };

  void OnOdometry(const px4_msgs::msg::VehicleOdometry & msg);
  void OnSetpoint(const gz::msgs::Vector2d & msg);
  void Publish(double t, double alpha, double beta, double thrust);

  // configuration
  std::string _model_name;
  bool _use_ground_truth{false};
  gz::sim::Link _link;  // ground_truth mode only
  std::string _estimate_topic{"/fmu/out/vehicle_odometry"};
  double _period{1.0 / 500.0};
  double _start_delay{1.0};
  double _takeoff_accel{0.3};
  double _takeoff_time{1.5};
  bool _predict_attitude{true};
  double _max_prediction{0.05};
  double _estimate_timeout{0.2};
  double _omega_hover{0.0};
  ReducedAttitudeController _controller;

  // ROS 2 side (own executor thread)
  bool _owns_rclcpp_context{false};
  rclcpp::Node::SharedPtr _ros_node;
  rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr _odom_sub;
  std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> _executor;
  std::thread _spin_thread;
  std::mutex _estimate_mutex;
  std::optional<Estimate> _estimate;

  // gz-transport side
  gz::transport::Node _gz_node;
  gz::transport::Node::Publisher _cmd_pub;
  gz::transport::Node::Publisher _debug_pub;
  std::mutex _setpoint_mutex;
  double _theta1{0.0};
  double _theta2{0.0};

  // run state (PreUpdate thread only)
  bool _configured{false};
  double _next_update{0.0};
  double _last_update{-1.0};
  double _t_first_estimate{-1.0};
  double _next_status{0.0};
};

namespace
{
const Eigen::Quaterniond kQEnuNed(0.0, M_SQRT1_2, M_SQRT1_2, 0.0);  // 180 deg about (1,1,0)
const Eigen::Quaterniond kQFrdFlu(0.0, 1.0, 0.0, 0.0);              // 180 deg about x

Eigen::Vector3d Vec(const gz::math::Vector3d & v) {return {v.X(), v.Y(), v.Z()};}

template<typename T>
T Param(const std::shared_ptr<const sdf::Element> & sdf, const std::string & name, T fallback)
{
  return sdf->HasElement(name) ? sdf->Get<T>(name) : fallback;
}
}  // namespace

MonospinnerController::~MonospinnerController()
{
  if (_executor) {
    _executor->cancel();
  }
  if (_spin_thread.joinable()) {
    _spin_thread.join();
  }
  _odom_sub.reset();
  _executor.reset();
  _ros_node.reset();
  if (_owns_rclcpp_context && rclcpp::ok()) {
    rclcpp::shutdown();
  }
}

void MonospinnerController::Configure(
  const gz::sim::Entity & entity, const std::shared_ptr<const sdf::Element> & sdf,
  gz::sim::EntityComponentManager & ecm, gz::sim::EventManager &)
{
  // Attach to the model in an .sdf, or to the world (at runtime through
  // /world/<world>/entity/system/add) with <model_name> naming the vehicle.
  gz::sim::Model model(entity);
  if (!model.Valid(ecm)) {
    gz::sim::World world(entity);
    const std::string name = Param(sdf, "model_name", std::string("monospinner"));
    if (world.Valid(ecm)) {
      model = gz::sim::Model(world.ModelByName(ecm, name));
    }
    if (!model.Valid(ecm)) {
      gzerr << "MonospinnerController: attach it to a model, or to the world with "
            << "<model_name> naming a model (no model '" << name << "' found)." << std::endl;
      return;
    }
  }
  _model_name = model.Name(ecm);

  // --- state source ---
  const std::string source = Param(sdf, "state_source", std::string("ekf2"));
  if (source == "ground_truth") {
    _use_ground_truth = true;
    const std::string link_name = Param(sdf, "link_name", std::string("base_link"));
    _link = gz::sim::Link(model.LinkByName(ecm, link_name));
    if (!_link.Valid(ecm)) {
      gzerr << "MonospinnerController: no link '" << link_name << "' in model '"
            << _model_name << "'." << std::endl;
      return;
    }
    // Without this WorldAngularVelocity() returns nullopt forever (CLAUDE.md, Known traps).
    _link.EnableVelocityChecks(ecm, true);
  } else if (source != "ekf2") {
    gzerr << "MonospinnerController: <state_source> must be 'ekf2' or 'ground_truth', got '"
          << source << "'." << std::endl;
    return;
  }

  // --- controller parameters (defaults: paper Tables 1-2, Sec. 7.1) ---
  ReducedAttitudeParams p;
  const ReducedAttitudeParams defaults;
  p.P = Vec(Param(sdf, "P", gz::math::Vector3d(defaults.P.x(), defaults.P.y(), defaults.P.z())));
  p.Kp = Vec(Param(sdf, "Kp", gz::math::Vector3d(defaults.Kp.x(), defaults.Kp.y(),
    defaults.Kp.z())));
  p.Kd = Vec(Param(sdf, "Kd", gz::math::Vector3d::Zero));
  p.Ki = Vec(Param(sdf, "Ki", gz::math::Vector3d::Zero));
  p.h_P = Param(sdf, "h_P", defaults.h_P);
  p.k_f = Param(sdf, "k_f", defaults.k_f);
  p.alpha_max = Param(sdf, "alpha_max_deg", defaults.alpha_max * 180.0 / M_PI) * M_PI / 180.0;
  p.lambda_max = Param(sdf, "lambda_max", defaults.lambda_max);
  _controller = ReducedAttitudeController(p);

  _theta1 = Param(sdf, "precession_deg", 0.0) * M_PI / 180.0;
  _theta2 = Param(sdf, "nutation_deg", 0.0) * M_PI / 180.0;
  _period = 1.0 / std::max(1.0, Param(sdf, "update_rate", 500.0));
  _start_delay = Param(sdf, "start_delay", _start_delay);
  _takeoff_accel = Param(sdf, "takeoff_accel", _takeoff_accel);
  _takeoff_time = Param(sdf, "takeoff_time", _takeoff_time);
  _predict_attitude = Param(sdf, "predict_attitude", _predict_attitude);
  _max_prediction = Param(sdf, "max_prediction", _max_prediction);
  _estimate_timeout = Param(sdf, "estimate_timeout", _estimate_timeout);
  _estimate_topic = Param(sdf, "estimate_topic", _estimate_topic);
  const std::string cmd_topic =
    Param(sdf, "command_topic", "/" + _model_name + "/command/motor_speed");
  const std::string setpoint_topic =
    Param(sdf, "setpoint_topic", "/" + _model_name + "/controller/setpoint");
  const std::string debug_topic =
    Param(sdf, "debug_topic", "/" + _model_name + "/controller/debug");

  // --- hover rotor speed, Eq. 79: Omega = sqrt(m g / k_f) ---
  double mass = Param(sdf, "mass", -1.0);
  if (mass <= 0.0) {
    mass = 0.0;
    for (const auto & link : model.Links(ecm)) {
      const auto * inertial = ecm.Component<gz::sim::components::Inertial>(link);
      if (inertial) {
        mass += inertial->Data().MassMatrix().Mass();
      }
    }
  }
  double gravity = Param(sdf, "gravity", -1.0);
  if (gravity <= 0.0) {
    const auto * g = ecm.Component<gz::sim::components::Gravity>(gz::sim::worldEntity(ecm));
    gravity = g ? g->Data().Length() : 9.80665;
  }
  if (mass <= 0.0) {
    gzerr << "MonospinnerController: model mass is zero; set <mass>." << std::endl;
    return;
  }
  _omega_hover = std::sqrt(mass * gravity / p.k_f);

  // --- ROS 2: EKF2 estimate (ekf2 mode only) ---
  if (!_use_ground_truth) {
    if (!rclcpp::ok()) {
      // No signal handlers: gz-sim owns SIGINT/SIGTERM.
      rclcpp::init(0, nullptr, rclcpp::InitOptions(), rclcpp::SignalHandlerOptions::None);
      _owns_rclcpp_context = true;
    }
    _ros_node = std::make_shared<rclcpp::Node>("gz_monospinner_control");
    // PX4's uXRCE-DDS publishers are best effort (same profile as
    // scripts/record_vehicle_state.py's PX4_QOS).
    const auto qos = rclcpp::QoS(rclcpp::KeepLast(5)).best_effort().durability_volatile();
    _odom_sub = _ros_node->create_subscription<px4_msgs::msg::VehicleOdometry>(
      _estimate_topic, qos,
      [this](const px4_msgs::msg::VehicleOdometry & msg) {OnOdometry(msg);});
    _executor = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    _executor->add_node(_ros_node);
    _spin_thread = std::thread([this]() {_executor->spin();});
  }

  // --- gz-transport: command out, debug out, setpoint in ---
  _cmd_pub = _gz_node.Advertise<gz::msgs::Actuators>(cmd_topic);
  _debug_pub = _gz_node.Advertise<gz::msgs::Double_V>(debug_topic);
  _gz_node.Subscribe(setpoint_topic, &MonospinnerController::OnSetpoint, this);

  _configured = true;
  gzmsg << "MonospinnerController configured: model=" << _model_name
        << " state_source=" << (_use_ground_truth ? "ground_truth (ECM)" : _estimate_topic)
        << " command=" << cmd_topic
        << " setpoint=" << setpoint_topic << " debug=" << debug_topic
        << " rate=" << 1.0 / _period << " Hz"
        << " P=" << p.P.transpose() << " Kp=" << p.Kp.transpose()
        << " Kd=" << p.Kd.transpose() << " Ki=" << p.Ki.transpose()
        << " h_P=" << p.h_P << " k_f=" << p.k_f
        << " alpha_max=" << p.alpha_max * 180.0 / M_PI << " deg"
        << " lambda_max=" << p.lambda_max
        << " mass=" << mass << " g=" << gravity << " Omega_hover=" << _omega_hover
        << " precession=" << _theta1 * 180.0 / M_PI << " deg"
        << " nutation=" << _theta2 * 180.0 / M_PI << " deg"
        << " start_delay=" << _start_delay << " takeoff_accel=" << _takeoff_accel
        << " takeoff_time=" << _takeoff_time
        << " predict_attitude=" << _predict_attitude << std::endl;
}

void MonospinnerController::OnOdometry(const px4_msgs::msg::VehicleOdometry & msg)
{
  if (!std::isfinite(msg.q[0]) || !std::isfinite(msg.angular_velocity[0])) {
    return;
  }
  Estimate e;
  e.q_ned_frd = Eigen::Quaterniond(msg.q[0], msg.q[1], msg.q[2], msg.q[3]).normalized();
  e.w_frd = Eigen::Vector3d(msg.angular_velocity[0], msg.angular_velocity[1],
    msg.angular_velocity[2]);
  e.t_sample = static_cast<double>(msg.timestamp_sample) * 1e-6;
  std::lock_guard<std::mutex> lock(_estimate_mutex);
  _estimate = e;
}

void MonospinnerController::OnSetpoint(const gz::msgs::Vector2d & msg)
{
  std::lock_guard<std::mutex> lock(_setpoint_mutex);
  _theta1 = msg.x();
  _theta2 = msg.y();
  gzmsg << "MonospinnerController: setpoint precession=" << _theta1 * 180.0 / M_PI
        << " deg nutation=" << _theta2 * 180.0 / M_PI << " deg" << std::endl;
}

void MonospinnerController::Publish(double t, double alpha, double beta, double thrust)
{
  gz::msgs::Actuators cmd;
  cmd.mutable_header()->mutable_stamp()->set_sec(static_cast<int64_t>(std::floor(t)));
  cmd.mutable_header()->mutable_stamp()->set_nsec(
    static_cast<int32_t>((t - std::floor(t)) * 1e9));
  cmd.add_position(alpha);
  cmd.add_position(beta);
  cmd.add_velocity(thrust);
  _cmd_pub.Publish(cmd);
}

void MonospinnerController::PreUpdate(
  const gz::sim::UpdateInfo & info, gz::sim::EntityComponentManager & ecm)
{
  if (!_configured || info.paused) {
    return;
  }
  const double t = std::chrono::duration<double>(info.simTime).count();
  if (t + 1e-9 < _next_update) {
    return;
  }
  _next_update = std::max(_next_update + _period, t);

  // Controller input, body(FLU) -> world(ENU) attitude and FLU body rate.
  Eigen::Quaterniond q;
  Eigen::Vector3d w;
  double t_sample = t;
  if (_use_ground_truth) {
    const auto pose = _link.WorldPose(ecm);
    const auto w_world = _link.WorldAngularVelocity(ecm);
    if (!pose || !w_world) {
      return;
    }
    q = Eigen::Quaterniond(pose->Rot().W(), pose->Rot().X(), pose->Rot().Y(), pose->Rot().Z());
    w = q.conjugate() * Eigen::Vector3d(w_world->X(), w_world->Y(), w_world->Z());
  } else {
    std::optional<Estimate> est;
    {
      std::lock_guard<std::mutex> lock(_estimate_mutex);
      est = _estimate;
    }
    if (!est) {
      return;  // nothing to act on yet; publish nothing so the actuator plugin stays idle
    }
    q = kQEnuNed * est->q_ned_frd * kQFrdFlu;
    w = Eigen::Vector3d(est->w_frd.x(), -est->w_frd.y(), -est->w_frd.z());
    t_sample = est->t_sample;
  }
  if (_t_first_estimate < 0.0) {
    _t_first_estimate = t;
    gzmsg << "MonospinnerController: first state ("
          << (_use_ground_truth ? "ground truth" : "EKF2") << ") at t=" << t
          << " s; takeoff in " << _start_delay << " s" << std::endl;
  }

  const double age = t - t_sample;
  const double t_takeoff = _t_first_estimate + _start_delay;

  // Phase and rotor speed for the desired body-axis thrust.
  int phase = 0;
  double thrust_scale = 0.0;
  if (age > _estimate_timeout) {
    phase = -1;
  } else if (t >= t_takeoff + 2.0 * _takeoff_time) {
    phase = 3;
    thrust_scale = 1.0;
  } else if (t >= t_takeoff + _takeoff_time) {
    phase = 2;
    thrust_scale = 1.0 - _takeoff_accel;
  } else if (t >= t_takeoff) {
    phase = 1;
    thrust_scale = 1.0 + _takeoff_accel;
  }
  const double omega = _omega_hover * std::sqrt(thrust_scale);

  // EKF2 estimate: optionally propagated by its age at the estimated body rate.
  if (!_use_ground_truth && _predict_attitude) {
    const double dt_pred = std::clamp(age, 0.0, _max_prediction);
    const double angle = w.norm() * dt_pred;
    if (angle > 0.0) {
      q = q * Eigen::Quaterniond(Eigen::AngleAxisd(angle, w.normalized()));
    }
  }
  q.normalize();

  Eigen::Vector3d n_bar;
  {
    std::lock_guard<std::mutex> lock(_setpoint_mutex);
    n_bar = ReducedAttitudeController::normalFromPrecessionNutation(_theta1, _theta2);
  }

  ReducedAttitudeOutput out;
  if (phase <= 0) {
    // On the ground (or blind): rotor off, no tilt, no PID memory carried into takeoff.
    _controller.reset();
    out.n = q.toRotationMatrix() * Eigen::Vector3d::UnitZ();
    Publish(t, 0.0, 0.0, 0.0);
  } else {
    const double dt = _last_update >= 0.0 ? t - _last_update : 0.0;
    out = _controller.update(q, w, n_bar, omega, dt);
    Publish(t, out.alpha, out.beta, out.thrust);
  }
  _last_update = t;

  gz::msgs::Double_V dbg;  // no header in Double_V: t_sim is element 0
  for (double v : {t, t_sample, age, static_cast<double>(phase), omega, out.alpha,
      out.beta, out.gamma_dot, out.thrust})
  {
    dbg.add_data(v);
  }
  for (const Eigen::Vector3d & v : {out.n, n_bar, w, out.w_r, out.tau_c}) {
    for (int i = 0; i < 3; ++i) {
      dbg.add_data(v[i]);
    }
  }
  dbg.add_data(out.singular ? 1.0 : 0.0);
  for (double v : {q.w(), q.x(), q.y(), q.z()}) {
    dbg.add_data(v);
  }
  _debug_pub.Publish(dbg);

  if (t >= _next_status) {
    _next_status = t + 2.0;
    if (phase == -1) {
      gzwarn << "MonospinnerController: EKF2 estimate is " << age
             << " s old (timeout " << _estimate_timeout << " s); rotor off" << std::endl;
    } else {
      gzmsg << "MonospinnerController t=" << t << " phase=" << phase
            << " age=" << age * 1e3 << " ms tilt="
            << std::acos(std::clamp(out.n.z(), -1.0, 1.0)) * 180.0 / M_PI
            << " deg alpha=" << out.alpha * 180.0 / M_PI << " deg T=" << out.thrust
            << " N" << std::endl;
    }
  }
}

}  // namespace monospinner

GZ_ADD_PLUGIN(
  monospinner::MonospinnerController,
  gz::sim::System,
  monospinner::MonospinnerController::ISystemConfigure,
  monospinner::MonospinnerController::ISystemPreUpdate)
