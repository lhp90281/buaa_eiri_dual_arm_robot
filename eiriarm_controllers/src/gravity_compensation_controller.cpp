#include "eiriarm_controllers/gravity_compensation_controller.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"

namespace eiriarm_controllers
{

GravityCompensationController::GravityCompensationController()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration 
GravityCompensationController::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request effort command interfaces for all joints
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration 
GravityCompensationController::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Request position and velocity state interfaces for all joints
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn GravityCompensationController::on_init()
{
  try {
    // Declare parameters
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<double>("max_effort", 50.0);
    auto_declare<double>("velocity_filter_alpha", 0.99);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  max_effort_ = get_node()->get_parameter("max_effort").as_double();
  velocity_filter_alpha_ = get_node()->get_parameter("velocity_filter_alpha").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified in 'joints' parameter!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "Configuring GravityCompensationController with %zu joints", joint_names_.size());

  // Get URDF from robot_description parameter
  std::string robot_description;
  if (!get_node()->get_parameter("robot_description", robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to get 'robot_description' parameter!");
    return controller_interface::CallbackReturn::ERROR;
  }

  if (robot_description.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "robot_description parameter is empty!");
    return controller_interface::CallbackReturn::ERROR;
  }

  // Initialize dynamics library
  dynamics_ = std::make_unique<eiriarm_dynamics::EiriarmDynamics>();
  if (!dynamics_->initFromURDF(robot_description)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Failed to initialize dynamics from URDF!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "Dynamics initialized. nq=%d, nv=%d", 
              dynamics_->getNq(), dynamics_->getNv());

  // Initialize state vectors
  q_ = Eigen::VectorXd::Zero(dynamics_->getNq());
  v_ = Eigen::VectorXd::Zero(dynamics_->getNv());
  v_filtered_ = Eigen::VectorXd::Zero(dynamics_->getNv());

  // Build joint name to velocity index mapping
  std::map<std::string, int> joint_name_to_idx_v;
  for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
    std::string name = dynamics_->getJointName(j);
    int idx_v = dynamics_->getJointIdxV(j);
    if (!name.empty() && idx_v >= 0) {
      joint_name_to_idx_v[name] = idx_v;
    }
  }

  // Load per-joint effort limits and verify all joints exist in model
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    const std::string& joint_name = joint_names_[i];
    
    // Check if joint exists in model
    if (joint_name_to_idx_v.find(joint_name) == joint_name_to_idx_v.end()) {
      RCLCPP_ERROR(get_node()->get_logger(), 
                   "Joint %s not found in dynamics model!", joint_name.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
    
    // Get effort limit from dynamics model (from URDF)
    double effort_limit = max_effort_;
    for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
      if (dynamics_->getJointName(j) == joint_name) {
        double urdf_limit = dynamics_->getEffortLimit(j);
        if (urdf_limit > 0) {
          effort_limit = urdf_limit;
        }
        break;
      }
    }
    
    // Store in map
    joint_gains_[joint_name] = {effort_limit};
    
    RCLCPP_INFO(get_node()->get_logger(), 
                "Joint %s (idx_v=%d): Effort Limit=%.2f (pure gravity compensation, Kp=0, Kd=0)",
                joint_name.c_str(), joint_name_to_idx_v[joint_name], effort_limit);
  }

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Verify that all required interfaces are available
  if (state_interfaces_.size() != joint_names_.size() * 2) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu state interfaces, got %zu",
                 joint_names_.size() * 2, state_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  if (command_interfaces_.size() != joint_names_.size()) {
    RCLCPP_ERROR(get_node()->get_logger(),
                 "Expected %zu command interfaces, got %zu",
                 joint_names_.size(), command_interfaces_.size());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "GravityCompensationController activated. Robot should feel weightless.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn GravityCompensationController::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Set all command interfaces to zero before releasing
  for (auto & command_interface : command_interfaces_) {
    command_interface.set_value(0.0);
  }

  RCLCPP_INFO(get_node()->get_logger(), "GravityCompensationController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type GravityCompensationController::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  // Read joint states from state_interfaces and map to full state vector
  // Note: q_ and v_ are size nq/nv (18), but we only control 14 joints
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    double q_current = state_interfaces_[2 * i].get_value();      // position
    double v_current = state_interfaces_[2 * i + 1].get_value();  // velocity
    
    // Find the velocity index in the full model
    int idx_v = -1;
    for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
      if (dynamics_->getJointName(j) == joint_names_[i]) {
        idx_v = dynamics_->getJointIdxV(j);
        break;
      }
    }
    
    if (idx_v >= 0 && idx_v < q_.size()) {
      q_[idx_v] = q_current;
      v_[idx_v] = v_current;
      
      // Apply low-pass filter to velocity (same as impedance controller for consistency)
      v_filtered_[idx_v] = (1.0 - velocity_filter_alpha_) * v_current + 
                           velocity_filter_alpha_ * v_filtered_[idx_v];
    }
  }

  // Update dynamics state with FILTERED velocity (consistent with impedance controller)
  dynamics_->updateState(q_, v_filtered_);

  // Compute gravity compensation (this is the ONLY torque component, Kp=0, Kd=0)
  Eigen::VectorXd tau_gravity = dynamics_->computeGravity();

  // Apply per-joint effort limits and write to command interfaces
  // Map from full tau_gravity vector to controlled joints
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    // Find the velocity index in the full model
    int idx_v = -1;
    for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
      if (dynamics_->getJointName(j) == joint_names_[i]) {
        idx_v = dynamics_->getJointIdxV(j);
        break;
      }
    }
    
    if (idx_v >= 0 && idx_v < tau_gravity.size()) {
      double effort_limit = joint_gains_[joint_names_[i]].effort_limit;
      double clamped_effort = std::max(-effort_limit, 
                                       std::min(tau_gravity[idx_v], effort_limit));
      
      command_interfaces_[i].set_value(clamped_effort);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }

  return controller_interface::return_type::OK;
}

}  // namespace eiriarm_controllers

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  eiriarm_controllers::GravityCompensationController,
  controller_interface::ControllerInterface)
