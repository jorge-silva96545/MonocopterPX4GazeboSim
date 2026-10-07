#include "monospinner/ThrustVectorActuator.hh"

#include <chrono>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>
#include <gz/math/Helpers.hh> 
#include <cmath>

using namespace monospinner;

void ThrustVectorActuator::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{
  this->model = gz::sim::Model(_entity); // set the model
  if (!this->model.Valid(_ecm)) // check if the model is valid
  {
    gzerr << "ThrustVectorActuator plugin must be attached to a model entity."
          << std::endl;
    return;
  }

  if (_sdf->HasElement("link_name"))
    this->linkName = _sdf->Get<std::string>("link_name");
  if (_sdf->HasElement("rotor_visual_link"))
    this->rotorVisualLinkName = _sdf->Get<std::string>("rotor_visual_link");

  if (_sdf->HasElement("h_P"))
    this->hP = _sdf->Get<double>("h_P");
  if (_sdf->HasElement("h_S"))
    this->hS = _sdf->Get<double>("h_S");

  if (_sdf->HasElement("k_f"))
    this->kF = _sdf->Get<double>("k_f");
  if (_sdf->HasElement("k_tau"))
    this->kTau = _sdf->Get<double>("k_tau");
  if (_sdf->HasElement("k_ratio"))
    this->kRatio = _sdf->Get<double>("k_ratio");

  if (_sdf->HasElement("K_z"))
    this->kZ = _sdf->Get<double>("K_z");

  if (_sdf->HasElement("J_pz"))
    this->jPz = _sdf->Get<double>("J_pz");
  if (_sdf->HasElement("J_pd"))
    this->jPd = _sdf->Get<double>("J_pd");

  if (_sdf->HasElement("m_b"))
    this->mB = _sdf->Get<double>("m_b");
  if (_sdf->HasElement("m_s"))
    this->mS = _sdf->Get<double>("m_s");
  if (_sdf->HasElement("m_p"))
    this->mP = _sdf->Get<double>("m_p");

  if (_sdf->HasElement("alpha_max_deg"))
    this->alphaMaxDeg = _sdf->Get<double>("alpha_max_deg");
  if (_sdf->HasElement("lambda_max"))
    this->lambdaMax = _sdf->Get<double>("lambda_max");
  if (_sdf->HasElement("Omega_hover"))
    this->omegaHover = _sdf->Get<double>("Omega_hover");

  if (_sdf->HasElement("tau_thrust"))
    this->tauThrust = _sdf->Get<double>("tau_thrust");
  if (_sdf->HasElement("tau_tilt"))
    this->tauTilt = _sdf->Get<double>("tau_tilt");

  if (_sdf->HasElement("visual_spin_rate"))
    this->visualSpinRate = _sdf->Get<double>("visual_spin_rate");
  if (_sdf->HasElement("visual_tilt_gain"))
    this->visualTiltGain = _sdf->Get<double>("visual_tilt_gain");

  auto linkEntity = this->model.LinkByName(_ecm, this->linkName);
  if (linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "ThrustVectorActuator: could not find link [" << this->linkName
          << "] on model [" << this->model.Name(_ecm) << "]" << std::endl;
    return;
  }
  this->link = gz::sim::Link(linkEntity);
  // Without this, WorldAngularVelocity() returns nullopt forever, silently.
  this->link.EnableVelocityChecks(_ecm, true);

  auto rotorLinkEntity = this->model.LinkByName(_ecm, this->rotorVisualLinkName);
  if (rotorLinkEntity == gz::sim::kNullEntity)
  {
    gzerr << "ThrustVectorActuator: could not find rotor link ["
          << this->rotorVisualLinkName << "] on model [" << this->model.Name(_ecm)
          << "]" << std::endl;
    return;
  }
  this->rotorLink = gz::sim::Link(rotorLinkEntity);

  std::string modelName = this->model.Name(_ecm);
  // No leading "/model/" segment: PX4's GZMixingInterfaceESC (the interface that actually
  // publishes live rotor outputs -- see PX4-Autopilot/src/modules/simulation/gz_bridge/
  // GZMixingInterfaceESC.cpp) advertises on "/" + model_name + "/command/motor_speed".
  // GZMixingInterfaceWheel.cpp uses a "/model/"-prefixed variant of the same suffix, which
  // is easy to copy by mistake from the native gz-sim MulticopterMotorModel convention --
  // but that interface is for rovers and never actually publishes on our airframe, so a
  // subscriber there sees an advertised topic with no data. Verified live: capturing
  // "/monospinner/command/motor_speed" returns real (idle) ESC velocity messages;
  // "/model/monospinner/command/motor_speed" times out with none.
  std::string topic = "/" + modelName + "/command/motor_speed";
  this->node.Subscribe(
      topic, &ThrustVectorActuator::OnActuatorsMsg, this);

  gzmsg << "ThrustVectorActuator configured:"
        << " link=" << this->linkName
        << " rotor_visual_link=" << this->rotorVisualLinkName
        << " h_P=" << this->hP
        << " h_S=" << this->hS
        << " k_f=" << this->kF
        << " k_tau=" << this->kTau
        << " k_ratio=" << this->kRatio
        << " K_z=" << this->kZ
        << " J_pz=" << this->jPz
        << " J_pd=" << this->jPd
        << " m_b=" << this->mB
        << " m_s=" << this->mS
        << " m_p=" << this->mP
        << " alpha_max_deg=" << this->alphaMaxDeg
        << " lambda_max=" << this->lambdaMax
        << " Omega_hover=" << this->omegaHover
        << " tau_thrust=" << this->tauThrust
        << " tau_tilt=" << this->tauTilt
        << " visual_spin_rate=" << this->visualSpinRate
        << " visual_tilt_gain=" << this->visualTiltGain
        << " topic=" << topic
        << std::endl;
}

void ThrustVectorActuator::OnActuatorsMsg(const gz::msgs::Actuators &_msg)
{
  std::lock_guard<std::mutex> lock(this->commandMutex);
  this->lastCommand = _msg;
  this->haveCommand = true;
}

void ThrustVectorActuator::PreUpdate(
    const gz::sim::UpdateInfo &_info,
    gz::sim::EntityComponentManager &_ecm)
{
  this->dt = std::chrono::duration<double>(_info.dt).count();

  gz::msgs::Actuators command;
  bool haveCommandCopy;
  {
    std::lock_guard<std::mutex> lock(this->commandMutex);
    command = this->lastCommand;
    haveCommandCopy = this->haveCommand;
  }

  if (!haveCommandCopy)
    return;

  // Guard against the first messages off the wire, which can arrive with fewer than
  // 2 position / 1 velocity entries populated (e.g. before PX4's ESC interface has sent
  // a full command) -- indexing a protobuf repeated field out of bounds aborts the process.
  if (command.position_size() < 2 || command.velocity_size() < 1)
    return;

  std::cout << "running here\n";

  // TODO(physics): decode command.normalized() into (throttle, tilt_x, tilt_y) per the
  // actuator channel mapping in CLAUDE.md, clamp ||(tilt_x, tilt_y)|| <= 1, run through
  // the actuator lag model, compute the hub-offset thrust vector and the gyroscopic
  // coupling term, and apply via this->link.AddWorldForce() / AddWorldWrench().
  // TODO: Change the actuator message intepretation.
  this->actuatorTiltX = command.position(0); // corresponds to \alpha in the paper
  this->actuatorTiltY = command.position(1); // corresponds to \beta in the paper
  this->actuatorThrottle = command.velocity(0); // corresponds to actuator force, in newtons

  // TODO: for more realism, add a delay between the actuator setpoints and the actual setpoints
  // low pass filter from the actuator setpoint to the actuator state
  // auto thrustAlpha = this->dt / (this->dt + this->tauThrust);
  // auto tiltAlhpa = this->dt / (this->dt + this->tauTilt);
  // this->actuatorThrottle = thrustAlpha * thrustSetpoint + (1 - thrustAlpha) * this->actuatorThrottle; // implement low pass filter for the throttle
  // this->actuatorTiltX = tiltAlpha * tiltXSetpoint + (1 - tiltAlpha) * this->actuatorTiltX; // implement low pass filter for the throttle
  // this->actuatorTiltY = tiltAlpha * tiltXSetpoint + (1 - tiltAlpha) * this->actuatorTiltY; // implement low pass filter for the throttle  

  // for visual purposes, apply the new thrust orientation to the visual component of the rotor link

  // Apply physics
  // Total force applied at the propeller 
  auto bodyPose = this->link.WorldPose(_ecm); 
  auto bodyOmega = this->link.WorldAngularVelocity(_ecm);
  if (!bodyPose || !bodyOmega) return;  
  auto rotationWorldToBody = bodyPose->Rot(); // get current world boddy orientation (gz::math::Quaterniond)
  gz::math::Quaterniond rotationBodyToProp(0., this->actuatorTiltX, this->actuatorTiltY); // get current body prop orientation
  auto rotationWorldToProp = rotationWorldToBody * rotationBodyToProp; 
  gz::math::Vector3d thrustWorld = rotationWorldToProp * gz::math::Vector3d(0, 0, actuatorThrottle); 
  this->rotorLink.AddWorldForce(_ecm, thrustWorld); // apply force at the rotor link with the appropriate magnitude
  // Air drag reaction torque applied at the propeller (notice a pure moment can be translated freely)
  this->link.AddWorldWrench(_ecm, gz::math::Vector3d(0., 0., 0.), (this->kTau / this->kF) * thrustWorld); 
  // Gyroscopic torque 
  auto propAngSpeedBodyFrame = std::sqrt(this->actuatorThrottle/this->kF); // recover angular speed of propellers 
  auto gyroscopicTorque = - propAngSpeedBodyFrame * this->jPz * (bodyOmega->Cross(rotationWorldToProp * gz::math::Vector3d(0, 0, 1)));
  this->link.AddWorldWrench(_ecm, gz::math::Vector3d(0., 0., 0.), gyroscopicTorque);
  // Fin air drag torque 
  this->link.AddWorldWrench(_ecm, gz::math::Vector3d(0., 0., 0.), - this->kZ * (rotationWorldToBody * gz::math::Vector3d(0., 0., 1.)) * std::pow(bodyOmega->Z(),2) * gz::math::sgn(bodyOmega->Z())); 


}

GZ_ADD_PLUGIN(
    ThrustVectorActuator,
    gz::sim::System,
    ThrustVectorActuator::ISystemConfigure,
    ThrustVectorActuator::ISystemPreUpdate)
