// Custom PX4 ROS 2 mode for the mono-spinner. See CLAUDE.md: PX4's built-in multicopter
// controllers and control allocation are bypassed -- this mode drives the vehicle's three
// actuator channels (throttle, tilt x, tilt y) directly.

// This is the file where the control law that's to be validated will be implemented

#include <Eigen/Core>
#include <px4_ros2/components/mode.hpp>
#include <px4_ros2/components/node_with_mode.hpp>
#include <px4_ros2/control/setpoint_types/direct_actuators.hpp>
#include <px4_ros2/odometry/attitude.hpp>
#include <px4_ros2/odometry/local_position.hpp>
#include <rclcpp/rclcpp.hpp>

namespace monospinner
{

class MonospinnerMode : public px4_ros2::ModeBase
{
public:
  explicit MonospinnerMode(rclcpp::Node & node)
  : ModeBase(node, Settings{"Monospinner"})
  {
    _vehicle_local_position = std::make_shared<px4_ros2::OdometryLocalPosition>(*this);
    _vehicle_attitude = std::make_shared<px4_ros2::OdometryAttitude>(*this);
    _direct_actuators = std::make_shared<px4_ros2::DirectActuatorsSetpointType>(*this);
  }

  // No control-law state to (re)set yet -- see the TODO in updateSetpoint().
  void onActivate() override {}
  void onDeactivate() override {}

  void updateSetpoint(float /*dt_s*/) override
  {
    // TODO(control): decode the reduced-attitude control law and the mono-spinner actuator
    // channel mapping from CLAUDE.md (throttle, tilt x, tilt y in channels 0-2) into a real
    // motor command. For now this publishes a zero/idle command and nothing else.
    Eigen::Matrix<float, px4_ros2::DirectActuatorsSetpointType::kMaxNumMotors, 1> motor_commands =
      Eigen::Matrix<float, px4_ros2::DirectActuatorsSetpointType::kMaxNumMotors, 1>::Zero();
    _direct_actuators->updateMotors(motor_commands);
  }

private:
  std::shared_ptr<px4_ros2::OdometryLocalPosition> _vehicle_local_position;
  std::shared_ptr<px4_ros2::OdometryAttitude> _vehicle_attitude;
  std::shared_ptr<px4_ros2::DirectActuatorsSetpointType> _direct_actuators;
};

}  // namespace monospinner

using MonospinnerNode = px4_ros2::NodeWithMode<monospinner::MonospinnerMode>;

static const std::string kNodeName = "monospinner_control";
static const bool kEnableDebugOutput = true;

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MonospinnerNode>(kNodeName, kEnableDebugOutput));
  rclcpp::shutdown();
  return 0;
}
