#ifndef MONOSPINNER_IMU_PHYSICAL_HH_
#define MONOSPINNER_IMU_PHYSICAL_HH_

#include <random>
#include <string>

#include <gz/msgs/imu.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace monospinner
{

/// Replaces gz-sim-imu-system and the SDF <sensor type="imu"> element for this vehicle.
/// PX4's gz_bridge subscribes to a topic name it hardcodes regardless of whether a matching
/// <sensor> entity exists (GZBridge::subscribeImu(), PX4-Autopilot/src/modules/simulation/
/// gz_bridge/GZBridge.cpp), so nothing requires the stock sensor system -- this plugin reads
/// the link's rigid-body kinematics from the ECM every physics step, turns them into a
/// specific-force + angular-velocity IMU reading, layers on a Gauss-Markov bias / white
/// noise / saturation model, and publishes straight to the topic PX4 expects.
///
/// <topic> is mandatory and must match GZBridge's hardcoded string exactly -- this plugin
/// does not guess it (see CLAUDE.md's rule against inventing gz/PX4 interface details).
class ImuPhysical
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPostUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  public: void PostUpdate(
      const gz::sim::UpdateInfo &_info,
      const gz::sim::EntityComponentManager &_ecm) override;

  /// One channel's worth of running Gauss-Markov bias state.
  private: struct BiasState
  {
    double x{0.0};
    double y{0.0};
    double z{0.0};
  };

  private: void StepBias(BiasState &_bias, double _stdDev, double _corrTime, double _dt);

  private: double SampleNormal(double _stdDev);

  private: double Saturate(double _value, double _limit) const;

  private: gz::sim::Link link{gz::sim::kNullEntity};
  private: gz::sim::Entity worldEnt{gz::sim::kNullEntity};

  private: std::string linkName{"base_link"};

  // Mandatory: the exact topic GZBridge::subscribeImu() hardcodes. No default -- see class
  // comment.
  private: std::string topic;

  private: double updateRate{250.0};
  private: std::chrono::steady_clock::duration updatePeriod{0};
  private: std::chrono::steady_clock::duration lastUpdateTime{0};
  private: bool haveLastUpdateTime{false};

  // Steady-state stddev in the sensor's own units (rad/s gyro, m/s^2 accel);
  // corrTime <= 0 disables the drift term.
  private: double gyroBiasStdDev{0.0};
  private: double gyroBiasCorrTime{0.0};
  private: double accelBiasStdDev{0.0};
  private: double accelBiasCorrTime{0.0};

  private: double gyroNoiseStdDev{0.0};
  private: double accelNoiseStdDev{0.0};

  private: double gyroSaturation{0.0};
  private: double accelSaturation{0.0};

  private: BiasState gyroBias;
  private: BiasState accelBias;

  private: bool publishGroundTruth{false};
  private: std::string groundTruthTopic; 

  // Seeded from the <seed> SDF param, defaulting to 0 (not hardware entropy) so runs are
  // reproducible unless <seed> is overridden; see the seeding comment in Configure().
  private: std::mt19937 rng;
  private: std::normal_distribution<double> normalDist{0.0, 1.0};

  private: gz::transport::Node node;
  private: gz::transport::Node::Publisher publisher;
  private: gz::transport::Node::Publisher groundTruthPublisher; 
};

}  // namespace monospinner

#endif  // MONOSPINNER_IMU_PHYSICAL_HH_
