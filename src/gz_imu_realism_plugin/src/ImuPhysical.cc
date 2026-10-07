#include "monospinner/ImuPhysical.hh"

#include <algorithm>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/msgs/convert/Quaternion.hh>
#include <gz/msgs/convert/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

using namespace monospinner;

void ImuPhysical::Configure(
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager & /*_eventMgr*/)
{

  //  Check if the passed model entity is valid
  gz::sim::Model model(_entity);
  if (!model.Valid(_ecm))
  {
    gzerr << "ImuPhysical plugin must be attached to a model entity." << std::endl;
    return;
  }

  // Store the link name associated with the IMU
  if (_sdf->HasElement("link_name"))
    this->linkName = _sdf->Get<std::string>("link_name");

  // store the link entity associated with link name
  auto linkEntity = model.LinkByName(_ecm, this->linkName);
  if (linkEntity == gz::sim::kNullEntity)
  {
    gzerr << "ImuPhysical: could not find link [" << this->linkName << "] on model ["
          << model.Name(_ecm) << "]" << std::endl;
    return;
  }
  this->link = gz::sim::Link(linkEntity);

  // Without these, WorldAngularVelocity()/WorldLinearAcceleration() return nullopt forever
  this->link.EnableVelocityChecks(_ecm, true);
  this->link.EnableAccelerationChecks(_ecm, true);

  this->worldEnt = gz::sim::worldEntity(_ecm);

  // Store topic to which information will be published
  if (_sdf->HasElement("topic"))
    this->topic = _sdf->Get<std::string>("topic");
  if (this->topic.empty())
  {
    gzerr << "ImuPhysical: <topic> is required -- refusing to guess PX4's hardcoded IMU "
          << "topic name (see GZBridge::subscribeImu(), PX4-Autopilot/src/modules/"
          << "simulation/gz_bridge/GZBridge.cpp). Verify it with `gz topic -l` before "
          << "trusting it." << std::endl;
    return;
  }

  // Store update rate of IMU
  if (_sdf->HasElement("update_rate"))
    this->updateRate = _sdf->Get<double>("update_rate");
  if (this->updateRate <= 0.0)
  {
    gzerr << "ImuPhysical: <update_rate> must be positive." << std::endl;
    return;
  }
  this->updatePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / this->updateRate));

  // arguments on bias
  if (_sdf->HasElement("gyro_bias_stddev"))
    this->gyroBiasStdDev = _sdf->Get<double>("gyro_bias_stddev");
  if (_sdf->HasElement("gyro_bias_correlation_time"))
    this->gyroBiasCorrTime = _sdf->Get<double>("gyro_bias_correlation_time");
  if (_sdf->HasElement("accel_bias_stddev"))
    this->accelBiasStdDev = _sdf->Get<double>("accel_bias_stddev");
  if (_sdf->HasElement("accel_bias_correlation_time"))
    this->accelBiasCorrTime = _sdf->Get<double>("accel_bias_correlation_time");

  // arguments on noise
  if (_sdf->HasElement("gyro_noise_stddev"))
    this->gyroNoiseStdDev = _sdf->Get<double>("gyro_noise_stddev");
  if (_sdf->HasElement("accel_noise_stddev"))
    this->accelNoiseStdDev = _sdf->Get<double>("accel_noise_stddev");

  // arguments on saturation
  if (_sdf->HasElement("gyro_saturation"))
    this->gyroSaturation = _sdf->Get<double>("gyro_saturation");
  if (_sdf->HasElement("accel_saturation"))
    this->accelSaturation = _sdf->Get<double>("accel_saturation");

  // arguments on publishing the ground truth 
  if (_sdf->HasElement("publish_ground_truth"))
    this->publishGroundTruth = _sdf->Get<bool>("publish_ground_truth"); 
  if (_sdf->HasElement("ground_truth_topic"))
    this->groundTruthTopic = _sdf->Get<std::string>("ground_truth_topic");
  if(this->publishGroundTruth)
    this->groundTruthPublisher = this->node.Advertise<gz::msgs::IMU>(this->groundTruthTopic);
  


  // Defaults to a fixed seed for reproducibility
  unsigned int seed = 0u;
  if (_sdf->HasElement("seed"))
    seed = _sdf->Get<unsigned int>("seed");
  this->rng.seed(seed);

  // initialize the bias to zero
  this->gyroBias = {0., 0., 0.}; // {this->SampleNormal(this->gyroBiasStdDev),
                    //this->SampleNormal(this->gyroBiasStdDev),
                    //this->SampleNormal(this->gyroBiasStdDev)};
  this->accelBias = {0., 0., 0.}; // {this->SampleNormal(this->accelBiasStdDev),
                     // this->SampleNormal(this->accelBiasStdDev),
                     // this->SampleNormal(this->accelBiasStdDev)};

  // advertise the topic to which the imu reading is published 
  this->publisher = this->node.Advertise<gz::msgs::IMU>(this->topic);
  if (!this->publisher)
  {
    gzerr << "ImuPhysical: failed to advertise topic [" << this->topic << "]" << std::endl;
    return;
  }

  gzmsg << "ImuPhysical configured:"
        << " link_name=" << this->linkName
        << " topic=" << this->topic
        << " update_rate=" << this->updateRate
        << " gyro_noise_stddev=" << this->gyroNoiseStdDev
        << " accel_noise_stddev=" << this->accelNoiseStdDev
        << " seed=" << seed
        << std::endl;
}

// sample normal distribution 
double ImuPhysical::SampleNormal(double _stdDev)
{
  if (_stdDev <= 0.0)
    return 0.0;
  return _stdDev * this->normalDist(this->rng);
}

// propagate the bias according to a Gauss-Markov process
void ImuPhysical::StepBias(BiasState &_bias, double _stdDev, double _corrTime, double _dt)
{
  if (_stdDev <= 0.0 || _dt <= 0.0)
    return;

  // Discretized Ornstein-Uhlenbeck / Gauss-Markov process.
  double driftGain = (_corrTime > 0.0) ? (_dt / _corrTime) : 0.0;
  double forcingGain = (_corrTime > 0.0)
      ? _stdDev * std::sqrt(2.0 * _dt / _corrTime)
      : _stdDev * std::sqrt(_dt);

  _bias.x += -driftGain * _bias.x + forcingGain * this->normalDist(this->rng);
  _bias.y += -driftGain * _bias.y + forcingGain * this->normalDist(this->rng);
  _bias.z += -driftGain * _bias.z + forcingGain * this->normalDist(this->rng);
}

// implement saturation 
double ImuPhysical::Saturate(double _value, double _limit) const
{
  if (_limit <= 0.0)
    return _value;
  return std::clamp(_value, -_limit, _limit);
}

void ImuPhysical::PostUpdate(
    const gz::sim::UpdateInfo &_info,
    const gz::sim::EntityComponentManager &_ecm)
{
  if (_info.paused)
    return;

  if (this->haveLastUpdateTime
      && (_info.simTime - this->lastUpdateTime) < this->updatePeriod)
    return;

  // read info from simulation
  auto pose = this->link.WorldPose(_ecm);
  auto angularVelWorld = this->link.WorldAngularVelocity(_ecm);
  auto linearAccelWorld = this->link.WorldLinearAcceleration(_ecm);
  auto gravity = gz::sim::World(this->worldEnt).Gravity(_ecm);

  // Velocity/acceleration checks take a step or two to start returning data after being
  // enabled in Configure(); nullopt here just means "not ready yet".
  if (!pose || !angularVelWorld || !linearAccelWorld || !gravity)
    return;

  double dt = 0.0;
  if (this->haveLastUpdateTime)
  {
    dt = std::chrono::duration<double>(_info.simTime - this->lastUpdateTime).count();
  }
  this->lastUpdateTime = _info.simTime;
  this->haveLastUpdateTime = true;

  gz::math::Quaterniond qInv = pose->Rot().Inverse();

  // Strapdown-IMU model: gyro reads angular velocity in the body frame; the accelerometer
  // reads specific force -- coordinate acceleration minus gravity, rotated into the body
  // frame -- so a stationary, level unit reads +g upward, matching a real sensor.
  gz::math::Vector3d gyroBody = qInv.RotateVector(*angularVelWorld);
  gz::math::Vector3d specificForceBody = qInv.RotateVector(*linearAccelWorld - *gravity);

  if (dt > 0.0)
  {
    this->StepBias(this->gyroBias, this->gyroBiasStdDev, this->gyroBiasCorrTime, dt);
    this->StepBias(this->accelBias, this->accelBiasStdDev, this->accelBiasCorrTime, dt);
  }

  // prepare imu message
  gz::msgs::IMU msg;

  auto simTimeSec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
  auto simTimeNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
      _info.simTime - simTimeSec);
  msg.mutable_header()->mutable_stamp()->set_sec(simTimeSec.count());
  msg.mutable_header()->mutable_stamp()->set_nsec(simTimeNsec.count());

  msg.mutable_angular_velocity()->set_x(this->Saturate(
      gyroBody.X() + this->gyroBias.x + this->SampleNormal(this->gyroNoiseStdDev),
      this->gyroSaturation));
  msg.mutable_angular_velocity()->set_y(this->Saturate(
      gyroBody.Y() + this->gyroBias.y + this->SampleNormal(this->gyroNoiseStdDev),
      this->gyroSaturation));
  msg.mutable_angular_velocity()->set_z(this->Saturate(
      gyroBody.Z() + this->gyroBias.z + this->SampleNormal(this->gyroNoiseStdDev),
      this->gyroSaturation));

  msg.mutable_linear_acceleration()->set_x(this->Saturate(
      specificForceBody.X() + this->accelBias.x
          + this->SampleNormal(this->accelNoiseStdDev),
      this->accelSaturation));
  msg.mutable_linear_acceleration()->set_y(this->Saturate(
      specificForceBody.Y() + this->accelBias.y
          + this->SampleNormal(this->accelNoiseStdDev),
      this->accelSaturation));
  msg.mutable_linear_acceleration()->set_z(this->Saturate(
      specificForceBody.Z() + this->accelBias.z
          + this->SampleNormal(this->accelNoiseStdDev),
      this->accelSaturation));

  // prepare vehicle state ground truth 
  // Noise-free IMU reading: the same body-frame rate and specific force fed into the
  // noisy message above, before bias/noise/saturation, plus the true world orientation.
  if (this->publishGroundTruth)
  {
    gz::msgs::IMU groundTruthMessage;
    *groundTruthMessage.mutable_header() = msg.header();

    gz::msgs::Set(groundTruthMessage.mutable_orientation(), pose->Rot());
    gz::msgs::Set(groundTruthMessage.mutable_angular_velocity(), gyroBody);
    gz::msgs::Set(groundTruthMessage.mutable_linear_acceleration(), specificForceBody);

    this->groundTruthPublisher.Publish(groundTruthMessage);
  }

  this->publisher.Publish(msg);
}

GZ_ADD_PLUGIN(
    ImuPhysical,
    gz::sim::System,
    ImuPhysical::ISystemConfigure,
    ImuPhysical::ISystemPostUpdate)
