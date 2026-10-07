#ifndef MONOSPINNER_THRUST_VECTOR_ACTUATOR_HH_
#define MONOSPINNER_THRUST_VECTOR_ACTUATOR_HH_

#include <mutex>
#include <string>

#include <gz/msgs/actuators.pb.h>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/transport/Node.hh>

namespace monospinner
{

/// Applies the mono-spinner's thrust vector (see CLAUDE.md) as a wrench on a single link.
/// No rotor link, no propeller joint -- the rotor is entirely a modelled force.
class ThrustVectorActuator
  : public gz::sim::System,
    public gz::sim::ISystemConfigure,
    public gz::sim::ISystemPreUpdate
{
  public: void Configure(
      const gz::sim::Entity &_entity,
      const std::shared_ptr<const sdf::Element> &_sdf,
      gz::sim::EntityComponentManager &_ecm,
      gz::sim::EventManager &_eventMgr) override;

  public: void PreUpdate(
      const gz::sim::UpdateInfo &_info,
      gz::sim::EntityComponentManager &_ecm) override;

  private: void OnActuatorsMsg(const gz::msgs::Actuators &_msg);

  private: gz::sim::Model model;
  private: gz::sim::Link link{gz::sim::kNullEntity};
  private: gz::sim::Link rotorLink{gz::sim::kNullEntity};

  // Defaults below mirror models/monospinner/model.sdf's own defaults; both are overridden
  // by whatever the <plugin> block in the SDF actually specifies.
  private: std::string linkName{"base_link"};
  private: std::string rotorVisualLinkName{"rotor"};

  // Geometry (paper Table 1)
  private: double hP{0.083};   // rotor above F_B, m
  private: double hS{0.145};   // battery below F_B, m

  // Rotor aerodynamic coefficients (paper Table 1, Eq. 24-25)
  private: double kF{5.2400e-6};     // f_p = k_f * gamma_dot^2, kg m
  private: double kTau{1.0800e-8};   // tau_p = k_tau * gamma_dot^2, kg m^2
  private: double kRatio{2.0611e-3}; // k = k_tau / k_f, m; tau_p = k * f_p

  // Body aerodynamic drag (paper Eq. 31-33)
  private: double kZ{2.80908e-5};    // tau = -sign(wz) * K_z * wz^2

  // Rotor inertia for the gyroscopic term (paper Eq. 44)
  private: double jPz{1.04e-5};
  private: double jPd{5.2e-6};

  // Masses (paper Table 1)
  private: double mB{0.2100};
  private: double mS{0.2007};
  private: double mP{0.0067};

  // Control set limits (paper Sec. 7.1, Eq. 80)
  private: double alphaMaxDeg{30.0}; // tilt cone half-angle
  private: double lambdaMax{1.2};    // max gamma_dot / Omega_hover
  private: double omegaHover{945.0};

  // Actuator dynamics (first-order placeholder; see CLAUDE.md)
  private: double tauThrust{0.05};
  private: double tauTilt{0.03};

  // Cosmetic rotor animation
  private: double visualSpinRate{25.0};
  private: double visualTiltGain{1.0};

  private: gz::transport::Node node;
  private: std::mutex commandMutex;
  private: gz::msgs::Actuators lastCommand;
  private: bool haveCommand{false};

  // Actuator internal dynamics state -- the lagged (post-tau_thrust/tau_tilt) output, as
  // opposed to lastCommand's raw setpoint. Persists across PreUpdate calls so the actuator
  // lag can be integrated one physics step at a time. Same [0,1]/[-1,1] channel ranges as
  // the commanded values (CLAUDE.md's actuator channel mapping), before conversion to a
  // physical thrust vector.
  private: double actuatorThrottle{0.0}; // [0, 1]
  private: double actuatorTiltX{0.0};    // [-1, 1]
  private: double actuatorTiltY{0.0};    // [-1, 1]

  // Physics step size (s) of the running simulation. Updated every PreUpdate call from
  // UpdateInfo::dt; needed to integrate the actuator dynamics above.
  private: double dt{0.0};
};

}  // namespace monospinner

#endif  // MONOSPINNER_THRUST_VECTOR_ACTUATOR_HH_
