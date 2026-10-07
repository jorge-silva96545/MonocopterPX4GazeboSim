#include "monospinner/MagnetometerPhysical.hh"

#include <algorithm>
#include <chrono>
#include <cmath>

#include <gz/common/Console.hh>
#include <gz/math/Matrix3.hh>
#include <gz/math/Quaternion.hh>
#include <gz/msgs/convert/Quaternion.hh>
#include <gz/msgs/convert/Vector3.hh>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Util.hh>
#include <gz/sim/World.hh>

using namespace monospinner;

void MagnetometerPhysical::Configure( // called once upon simulation startup
    const gz::sim::Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    gz::sim::EntityComponentManager &_ecm,
    gz::sim::EventManager &_eventMgr
){
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

    this->worldEnt = gz::sim::worldEntity(_ecm);

    // Store topic to which information will be published
    if (_sdf->HasElement("topic"))
        this->topic = _sdf->Get<std::string>("topic");
    if (this->topic.empty())
    {
        gzerr << "MagnetometerPhysical: <topic> is required -- refusing to guess PX4's "
            << "hardcoded magnetometer topic name (see GZBridge::subscribeMag(), "
            << "PX4-Autopilot/src/modules/simulation/gz_bridge/GZBridge.cpp:228). "
            << "Verify it with `gz topic -l` before trusting it." << std::endl;
        return;
    }

    // Store update rate of Magnetometer
    if (_sdf->HasElement("update_rate"))
        this->updateRate = _sdf->Get<double>("update_rate");
    if (this->updateRate <= 0.0)
    {
        gzerr << "ImuPhysical: <update_rate> must be positive." << std::endl;
        return;
    }

    this->updatePeriod = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(1.0 / this->updateRate));

    // arguments on bias and noise
    if (_sdf->HasElement("magnetometer_bias_stddev"))
        this->magnetometerBiasStdDev = _sdf->Get<double>("magnetometer_bias_stddev");
    if (_sdf->HasElement("magnetometer_bias_correlation_time"))
        this->magnetometerBiasCorrTime = _sdf->Get<double>("magnetometer_bias_correlation_time");
    if (_sdf->HasElement("magnetometer_noise_stddev"))
        this->magnetometerNoiseStdDev = _sdf->Get<double>("magnetometer_noise_stddev");

    // nominal Earth magnetic field in gz world coordinates (ENU), in gauss: "x y z"
    if (_sdf->HasElement("world_magnetic_field"))
        this->worldMagneticField = _sdf->Get<gz::math::Vector3d>("world_magnetic_field");

    this->magnetometerBias = {0., 0., 0.}; 

    // topic to which magnetometer reading will be published 
    this->publisher = this->node.Advertise<gz::msgs::Magnetometer>(this->topic);
}

void MagnetometerPhysical::PostUpdate(
    const gz::sim::UpdateInfo &_info, // only contains meta information about the simulation (for example, is it paused?)
    const gz::sim::EntityComponentManager &_ecm){

        // get current world state from info 
        if (_info.paused) return;

        // only publish at the configured update rate
        if (this->haveLastUpdateTime
            && (_info.simTime - this->lastUpdateTime) < this->updatePeriod)
            return;

        // read current pose of simulation
        auto pose = this->link.WorldPose(_ecm); // only this is relevant to the field read by the magnetometer
        if (!pose)
            return;

        // time elapsed since the last published sample (0 on the first one)
        double dt = 0.0;
        if (this->haveLastUpdateTime)
            dt = std::chrono::duration<double>(_info.simTime - this->lastUpdateTime).count();
        this->lastUpdateTime = _info.simTime;
        this->haveLastUpdateTime = true;

        // given the current pose of the magnetometer link, we compute the magnetometer reading
        gz::math::Quaterniond qWorldLink = pose->Rot();
        gz::math::Matrix3d rWorldLink(qWorldLink);
        gz::math::Vector3d nominalMagneticReading = rWorldLink.Transposed() * this->worldMagneticField; // rotate the world magnetic field into the link frame

        // propagate the magnetometer bias according to a Gauss-Markov process
        this->StepBias(this->magnetometerBias, this->magnetometerBiasStdDev, this->magnetometerBiasCorrTime, dt);

        // add the bias and the white noise to the nominal reading to get the final reading
        // (link frame, FLU: x forward, y left, z up)
        gz::math::Vector3d finalMagneticReading = nominalMagneticReading
            + gz::math::Vector3d(this->magnetometerBias.x, this->magnetometerBias.y, this->magnetometerBias.z)
            + gz::math::Vector3d(this->SampleNormal(this->magnetometerNoiseStdDev),
                                 this->SampleNormal(this->magnetometerNoiseStdDev),
                                 this->SampleNormal(this->magnetometerNoiseStdDev));

        // prepare the magnetometer message
        gz::msgs::Magnetometer msg;

        auto simTimeSec = std::chrono::duration_cast<std::chrono::seconds>(_info.simTime);
        auto simTimeNsec = std::chrono::duration_cast<std::chrono::nanoseconds>(
            _info.simTime - simTimeSec);
        msg.mutable_header()->mutable_stamp()->set_sec(simTimeSec.count());
        msg.mutable_header()->mutable_stamp()->set_nsec(simTimeNsec.count());

        // GZBridge::magnetometerCallback() (PX4-Autopilot/src/modules/simulation/gz_bridge/
        // GZBridge.cpp:399-404) builds PX4's body-frame FRD reading from this message as
        //   report = (-msg.y, -msg.x, msg.z)    and reads the values as gauss.
        // We want report = FRD = (b.x, -b.y, -b.z) of the FLU reading b, so publish the
        // pre-swapped vector msg = (b.y, -b.x, -b.z), which the bridge's swap undoes.
        msg.mutable_field_tesla()->set_x(finalMagneticReading.Y());
        msg.mutable_field_tesla()->set_y(-finalMagneticReading.X());
        msg.mutable_field_tesla()->set_z(-finalMagneticReading.Z());

        this->publisher.Publish(msg);
}

double MagnetometerPhysical::SampleNormal(double _stdDev)
{
    // zero-mean Gaussian sample with standard deviation _stdDev
    if (_stdDev <= 0.0)
        return 0.0;
    return _stdDev * this->normalDist(this->rng);
}

void MagnetometerPhysical::StepBias(BiasState &_bias, double _stdDev, double _corrTime, double _dt)
{
    if (_stdDev <= 0.0 || _dt <= 0.0)
        return;

    // Discretized first-order Gauss-Markov (Ornstein-Uhlenbeck) process: the bias decays
    // toward zero with time constant _corrTime while white noise keeps its steady-state
    // standard deviation at _stdDev. With no correlation time it reduces to a random walk.
    double driftGain = (_corrTime > 0.0) ? (_dt / _corrTime) : 0.0;
    double forcingGain = (_corrTime > 0.0)
        ? _stdDev * std::sqrt(2.0 * _dt / _corrTime)
        : _stdDev * std::sqrt(_dt);

    _bias.x += -driftGain * _bias.x + forcingGain * this->normalDist(this->rng);
    _bias.y += -driftGain * _bias.y + forcingGain * this->normalDist(this->rng);
    _bias.z += -driftGain * _bias.z + forcingGain * this->normalDist(this->rng);
}

GZ_ADD_PLUGIN(
    MagnetometerPhysical,
    gz::sim::System,
    MagnetometerPhysical::ISystemConfigure,
    MagnetometerPhysical::ISystemPostUpdate)
