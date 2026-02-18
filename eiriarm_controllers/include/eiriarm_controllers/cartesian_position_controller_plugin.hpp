#ifndef EIRIARM_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_PLUGIN_HPP_
#define EIRIARM_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_PLUGIN_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "eiriarm_controllers/eiriarm_dynamics.hpp"

namespace eiriarm_controllers
{

/**
 * @brief Cartesian Position Controller Plugin for ros2_control
 * 
 * This controller receives target end-effector poses and uses Pinocchio IK
 * to compute joint positions. The joint positions are sent as commands to
 * MuJoCo position actuators (via the effort command interface as data conduit).
 * 
 * Supports dual-arm control with independent target poses for each arm.
 * 
 * Command topics:
 *   ~/left_target_pose  (geometry_msgs/PoseStamped)
 *   ~/right_target_pose (geometry_msgs/PoseStamped)
 * 
 * Feedback topics:
 *   ~/joint_position_feedback (sensor_msgs/JointState)
 *   ~/cartesian_state         (geometry_msgs/PoseStamped) - current EE pose
 */
class CartesianPositionControllerPlugin : public controller_interface::ControllerInterface
{
public:
  CartesianPositionControllerPlugin();

  controller_interface::InterfaceConfiguration command_interface_configuration() const override;
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;
  controller_interface::CallbackReturn on_init() override;
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  void leftTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
  void rightTargetCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);

  // Algorithm library
  std::unique_ptr<eiriarm_dynamics::EiriarmDynamics> dynamics_;

  // Parameters
  std::vector<std::string> joint_names_;
  std::string left_ee_frame_;
  std::string right_ee_frame_;
  int ik_max_iter_ = 100;
  double ik_eps_ = 1e-4;
  double ik_damping_ = 1e-3;
  double ik_dt_ = 1.0;
  double position_interpolation_speed_ = 1.0;  // rad/s max joint speed for smooth motion

  // Frame IDs
  int left_frame_id_ = -1;
  int right_frame_id_ = -1;

  // State storage
  Eigen::VectorXd q_;           // current joint positions (full model)
  Eigen::VectorXd v_;           // current joint velocities (full model)
  Eigen::VectorXd q_desired_;   // desired joint positions from IK
  Eigen::VectorXd q_command_;   // smoothed command being sent

  // Cached mapping: joint_names_[i] -> velocity index in full Pinocchio model
  std::vector<int> joint_idx_v_;

  // Realtime buffers for target poses
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr> rt_left_target_;
  realtime_tools::RealtimeBuffer<geometry_msgs::msg::PoseStamped::SharedPtr> rt_right_target_;

  // Subscribers
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr left_target_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr right_target_sub_;

  // Publishers
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_pub_;

  // Flags
  bool left_target_received_ = false;
  bool right_target_received_ = false;
  bool initial_pose_captured_ = false;
};

}  // namespace eiriarm_controllers

#endif  // EIRIARM_CONTROLLERS__CARTESIAN_POSITION_CONTROLLER_PLUGIN_HPP_
