#ifndef MONOSPINNER_MAGNETOMETER_PHYSICAL_HH_
#define MONOSPINNER_MAGNETOMETER_PHYSICAL_HH_

#include <chrono>
#include <memory>
#include <random>
#include <string>

#include <gz/math/Vector3.hh>
#include <gz/msgs/magnetometer.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace monospinner
{

class MagnetometerPhysical
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

        private: struct BiasState
        {
            double x{0.0};
            double y{0.0};
            double z{0.0};
        };

        private: void StepBias(BiasState &_bias, double _stdDev, double _corrTime, double _dt);

        private: double SampleNormal(double _stdDev);

        // private: double Saturate(double _value, double _limit) const;

        private: gz::sim::Link link{gz::sim::kNullEntity};
        private: gz::sim::Entity worldEnt{gz::sim::kNullEntity};

        private: std::string linkName{"base_link"};

        // Mandatory: the exact topic GZBridge::subscribeImu() hardcodes. No default -- see class
        // comment.
        private: std::string topic;
        
        // keep track of time
        private: double updateRate{100.0};
        private: std::chrono::steady_clock::duration updatePeriod{0};
        private: std::chrono::steady_clock::duration lastUpdateTime{0};
        private: bool haveLastUpdateTime{false};
        
        // noise and bias parameters of the magnetometer
        private: double magnetometerBiasStdDev{0.0};
        private: double magnetometerBiasCorrTime{0.0};
        private: double magnetometerNoiseStdDev{0.0};
        
        // keeps track of the bias of the magnetometer
        private: BiasState magnetometerBias;

        // nominal Earth magnetic field at the world origin, in gz world coordinates (ENU:
        // x = East, y = North, z = Up), in gauss -- GZBridge forwards the published values
        // as gauss. Assumed constant: the vehicle moves too little for the field to change.
        private: gz::math::Vector3d worldMagneticField{0.0, 0.0, 0.0};

        // topic for publishing the magnetometer message
        private: std::mt19937 rng;
        private: std::normal_distribution<double> normalDist{0.0, 1.0};

        private: gz::transport::Node node;
        private: gz::transport::Node::Publisher publisher;
        private: gz::transport::Node::Publisher groundTruthPublisher; 
  };

}

#endif