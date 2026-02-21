#include "eiriarm_controllers/cartesian_position_controller_plugin.hpp"
#include <algorithm>
#include <cmath>
#include <memory>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>

#include "controller_interface/helpers.hpp"
#include <Eigen/Geometry>

namespace eiriarm_controllers
{

CartesianPositionControllerPlugin::CartesianPositionControllerPlugin()
: controller_interface::ControllerInterface()
{
}

controller_interface::InterfaceConfiguration 
CartesianPositionControllerPlugin::command_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  // Use effort command interfaces as data conduit to MuJoCo d->ctrl
  // MuJoCo position actuators interpret d->ctrl as desired position
  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/effort");
  }

  return config;
}

controller_interface::InterfaceConfiguration 
CartesianPositionControllerPlugin::state_interface_configuration() const
{
  controller_interface::InterfaceConfiguration config;
  config.type = controller_interface::interface_configuration_type::INDIVIDUAL;

  for (const auto & joint_name : joint_names_) {
    config.names.push_back(joint_name + "/position");
    config.names.push_back(joint_name + "/velocity");
  }

  return config;
}

controller_interface::CallbackReturn CartesianPositionControllerPlugin::on_init()
{
  try {
    auto_declare<std::vector<std::string>>("joints", std::vector<std::string>());
    auto_declare<std::string>("left_ee_frame", "left_link_7");
    auto_declare<std::string>("right_ee_frame", "right_link_7");
    auto_declare<int>("ik_max_iter", 100);
    auto_declare<double>("ik_eps", 1e-4);
    auto_declare<double>("ik_damping", 1e-3);
    auto_declare<double>("ik_dt", 1.0);
    auto_declare<double>("position_interpolation_speed", 2.0);
    
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_node()->get_logger(), "Exception during on_init: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionControllerPlugin::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Get parameters
  joint_names_ = get_node()->get_parameter("joints").as_string_array();
  left_ee_frame_ = get_node()->get_parameter("left_ee_frame").as_string();
  right_ee_frame_ = get_node()->get_parameter("right_ee_frame").as_string();
  ik_max_iter_ = get_node()->get_parameter("ik_max_iter").as_int();
  ik_eps_ = get_node()->get_parameter("ik_eps").as_double();
  ik_damping_ = get_node()->get_parameter("ik_damping").as_double();
  ik_dt_ = get_node()->get_parameter("ik_dt").as_double();
  position_interpolation_speed_ = get_node()->get_parameter("position_interpolation_speed").as_double();

  if (joint_names_.empty()) {
    RCLCPP_ERROR(get_node()->get_logger(), "No joints specified in 'joints' parameter!");
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "Configuring CartesianPositionController with %zu joints", joint_names_.size());

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

  // Resolve end-effector frame IDs
  left_frame_id_ = dynamics_->getFrameId(left_ee_frame_);
  right_frame_id_ = dynamics_->getFrameId(right_ee_frame_);

  if (left_frame_id_ < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                 "Left EE frame '%s' not found in model!", left_ee_frame_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }
  if (right_frame_id_ < 0) {
    RCLCPP_ERROR(get_node()->get_logger(), 
                 "Right EE frame '%s' not found in model!", right_ee_frame_.c_str());
    return controller_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(get_node()->get_logger(), 
              "Left EE frame: %s (id=%d), Right EE frame: %s (id=%d)",
              left_ee_frame_.c_str(), left_frame_id_,
              right_ee_frame_.c_str(), right_frame_id_);

  // Initialize state vectors
  q_ = Eigen::VectorXd::Zero(dynamics_->getNq());
  v_ = Eigen::VectorXd::Zero(dynamics_->getNv());
  q_desired_ = Eigen::VectorXd::Zero(dynamics_->getNq());
  q_command_ = Eigen::VectorXd::Zero(dynamics_->getNq());

  // Build cached joint name -> Pinocchio velocity index mapping
  joint_idx_v_.resize(joint_names_.size(), -1);
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    for (int j = 1; j < dynamics_->getNumJoints() + 1; ++j) {
      if (dynamics_->getJointName(j) == joint_names_[i]) {
        joint_idx_v_[i] = dynamics_->getJointIdxV(j);
        break;
      }
    }
    if (joint_idx_v_[i] < 0) {
      RCLCPP_ERROR(get_node()->get_logger(),
                   "Joint '%s' not found in Pinocchio model!", joint_names_[i].c_str());
    } else {
      RCLCPP_INFO(get_node()->get_logger(), 
                  "Joint %s: idx_v=%d", joint_names_[i].c_str(), joint_idx_v_[i]);
    }
  }

  // Create subscribers for target poses with realtime buffer
  left_target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/left_target_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      this->leftTargetCallback(msg);
    });

  right_target_sub_ = get_node()->create_subscription<geometry_msgs::msg::PoseStamped>(
    "~/right_target_pose", 10,
    [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
      this->rightTargetCallback(msg);
    });

  // Create homing subscriber
  homing_sub_ = get_node()->create_subscription<std_msgs::msg::String>(
    "~/joint_homing", 10,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      this->homingCallback(msg);
    });

  // Create feedback publisher
  feedback_pub_ = get_node()->create_publisher<sensor_msgs::msg::JointState>(
    "~/joint_position_feedback", 10);
  homing_status_pub_ = get_node()->create_publisher<std_msgs::msg::String>(
    "~/homing_status", 10);
  left_homing_fk_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/left_homing_fk", 10);
  right_homing_fk_pub_ = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(
    "~/right_homing_fk", 10);

  // Initialize realtime buffers
  auto empty_msg = std::make_shared<geometry_msgs::msg::PoseStamped>();
  rt_left_target_.writeFromNonRT(empty_msg);
  rt_right_target_.writeFromNonRT(empty_msg);
  RCLCPP_INFO(get_node()->get_logger(), "CartesianPositionController configured successfully");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionControllerPlugin::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
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

  // Reset flags
  left_target_received_ = false;
  right_target_received_ = false;
  initial_pose_captured_ = false;
  homing_requested_.store(false);
  homing_left_ = false;
  homing_right_ = false;

  RCLCPP_INFO(get_node()->get_logger(), 
              "CartesianPositionController activated. Waiting to capture initial pose...");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn CartesianPositionControllerPlugin::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Hold current position on deactivate (don't zero - that would cause jump)
  RCLCPP_INFO(get_node()->get_logger(), "CartesianPositionController deactivated");
  return controller_interface::CallbackReturn::SUCCESS;
}

void CartesianPositionControllerPlugin::leftTargetCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  rt_left_target_.writeFromNonRT(msg);
  left_target_received_ = true;
}

void CartesianPositionControllerPlugin::rightTargetCallback(
  const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  rt_right_target_.writeFromNonRT(msg);
  right_target_received_ = true;
}

void CartesianPositionControllerPlugin::homingCallback(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg->data.empty()) {
    homing_request_arm_ = msg->data;  // "left", "right", or "both"
    homing_requested_.store(true);
  }
}

controller_interface::return_type CartesianPositionControllerPlugin::update(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & period)
{
  double dt = period.seconds();
  if (dt <= 0.0) dt = 0.002;  // fallback

  // Read joint states from state_interfaces and map to full state vector
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    double q_current = state_interfaces_[2 * i].get_value();      // position
    double v_current = state_interfaces_[2 * i + 1].get_value();  // velocity
    
    int idx_v = joint_idx_v_[i];
    if (idx_v >= 0 && idx_v < q_.size()) {
      q_[idx_v] = q_current;
      v_[idx_v] = v_current;
    }
  }

  // Capture initial pose on first update
  if (!initial_pose_captured_) {
    q_desired_ = q_;
    q_command_ = q_;
    q_home_ = q_;  // Save home configuration for joint-level homing
    initial_pose_captured_ = true;
    
    // Log initial EE poses
    dynamics_->updateState(q_, v_);
    try {
      pinocchio::SE3 left_pose = dynamics_->computeFK(left_frame_id_);
      pinocchio::SE3 right_pose = dynamics_->computeFK(right_frame_id_);
      RCLCPP_INFO(get_node()->get_logger(), 
                  "Initial left EE pos: [%.4f, %.4f, %.4f]",
                  left_pose.translation()[0], left_pose.translation()[1], left_pose.translation()[2]);
      RCLCPP_INFO(get_node()->get_logger(), 
                  "Initial right EE pos: [%.4f, %.4f, %.4f]",
                  right_pose.translation()[0], right_pose.translation()[1], right_pose.translation()[2]);
    } catch (const std::exception& e) {
      RCLCPP_WARN(get_node()->get_logger(), "FK failed at init: %s", e.what());
    }
    
    RCLCPP_INFO(get_node()->get_logger(), "Initial pose captured for Cartesian position control");
  }

  // Handle homing request
  if (homing_requested_.exchange(false)) {
    std::string arm = homing_request_arm_;
    bool do_left = (arm == "left" || arm == "both");
    bool do_right = (arm == "right" || arm == "both");
    if (do_left) {
      homing_left_ = true;
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i].find("left_") == 0) {
          int idx_v = joint_idx_v_[i];
          if (idx_v >= 0 && idx_v < q_desired_.size()) {
            q_desired_[idx_v] = q_home_[idx_v];
          }
        }
      }
    }
    if (do_right) {
      homing_right_ = true;
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i].find("right_") == 0) {
          int idx_v = joint_idx_v_[i];
          if (idx_v >= 0 && idx_v < q_desired_.size()) {
            q_desired_[idx_v] = q_home_[idx_v];
          }
        }
      }
    }
    RCLCPP_INFO(get_node()->get_logger(), "Joint homing started for arm: %s", arm.c_str());
    auto status_msg = std::make_unique<std_msgs::msg::String>();
    status_msg->data = arm;  // echo which arm is homing
    homing_status_pub_->publish(std::move(status_msg));
  }

  // Check homing completion per arm
  if (homing_left_) {
    bool at_home = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i].find("left_") == 0) {
        int idx_v = joint_idx_v_[i];
        if (idx_v >= 0 && std::abs(q_command_[idx_v] - q_home_[idx_v]) > 0.01) {
          at_home = false;
          break;
        }
      }
    }
    if (at_home) {
      homing_left_ = false;
      // Compute FK at home config and publish exact EE pose
      dynamics_->updateState(q_desired_, v_ * 0.0);
      try {
        pinocchio::SE3 fk_pose = dynamics_->computeFK(left_frame_id_);
        auto fk_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        fk_msg->header.stamp = get_node()->now();
        fk_msg->header.frame_id = "world";
        fk_msg->pose.position.x = fk_pose.translation()[0];
        fk_msg->pose.position.y = fk_pose.translation()[1];
        fk_msg->pose.position.z = fk_pose.translation()[2];
        Eigen::Quaterniond fk_quat(fk_pose.rotation());
        fk_msg->pose.orientation.x = fk_quat.x();
        fk_msg->pose.orientation.y = fk_quat.y();
        fk_msg->pose.orientation.z = fk_quat.z();
        fk_msg->pose.orientation.w = fk_quat.w();
        left_homing_fk_pub_->publish(std::move(fk_msg));
        RCLCPP_INFO(get_node()->get_logger(), "Left arm homing complete. FK: [%.4f, %.4f, %.4f]",
                    fk_pose.translation()[0], fk_pose.translation()[1], fk_pose.translation()[2]);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_node()->get_logger(), "FK failed after left homing: %s", e.what());
      }
      if (!homing_right_) {
        auto status_msg = std::make_unique<std_msgs::msg::String>();
        status_msg->data = "done";
        homing_status_pub_->publish(std::move(status_msg));
      }
    }
  }
  if (homing_right_) {
    bool at_home = true;
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      if (joint_names_[i].find("right_") == 0) {
        int idx_v = joint_idx_v_[i];
        if (idx_v >= 0 && std::abs(q_command_[idx_v] - q_home_[idx_v]) > 0.01) {
          at_home = false;
          break;
        }
      }
    }
    if (at_home) {
      homing_right_ = false;
      // Compute FK at home config and publish exact EE pose
      dynamics_->updateState(q_desired_, v_ * 0.0);
      try {
        pinocchio::SE3 fk_pose = dynamics_->computeFK(right_frame_id_);
        auto fk_msg = std::make_unique<geometry_msgs::msg::PoseStamped>();
        fk_msg->header.stamp = get_node()->now();
        fk_msg->header.frame_id = "world";
        fk_msg->pose.position.x = fk_pose.translation()[0];
        fk_msg->pose.position.y = fk_pose.translation()[1];
        fk_msg->pose.position.z = fk_pose.translation()[2];
        Eigen::Quaterniond fk_quat(fk_pose.rotation());
        fk_msg->pose.orientation.x = fk_quat.x();
        fk_msg->pose.orientation.y = fk_quat.y();
        fk_msg->pose.orientation.z = fk_quat.z();
        fk_msg->pose.orientation.w = fk_quat.w();
        right_homing_fk_pub_->publish(std::move(fk_msg));
        RCLCPP_INFO(get_node()->get_logger(), "Right arm homing complete. FK: [%.4f, %.4f, %.4f]",
                    fk_pose.translation()[0], fk_pose.translation()[1], fk_pose.translation()[2]);
      } catch (const std::exception& e) {
        RCLCPP_WARN(get_node()->get_logger(), "FK failed after right homing: %s", e.what());
      }
      if (!homing_left_) {
        auto status_msg = std::make_unique<std_msgs::msg::String>();
        status_msg->data = "done";
        homing_status_pub_->publish(std::move(status_msg));
      }
    }
  }

  // Update dynamics state for IK computations
  dynamics_->updateState(q_, v_);

  // Process left arm target (skip if left arm is homing)
  auto left_msg = rt_left_target_.readFromRT();
  if (!homing_left_ && left_msg && (*left_msg) && left_target_received_) {
    const auto& pose = (*left_msg)->pose;
    
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d trans(pose.position.x, pose.position.y, pose.position.z);
    if (quat.norm() < 1e-6) {
      quat = Eigen::Quaterniond::Identity();
    } else {
      quat.normalize();
    }

    pinocchio::SE3 target_pose(quat.toRotationMatrix(), trans);
    
    Eigen::VectorXd q_ik_result;
    bool ik_success = dynamics_->solveIK(
      left_frame_id_, target_pose, q_desired_,
      q_ik_result, ik_max_iter_, ik_eps_, ik_damping_, ik_dt_);
    
    if (ik_success) {
      // Only write back on convergence to prevent seed corruption
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i].find("left_") == 0) {
          int idx_v = joint_idx_v_[i];
          if (idx_v >= 0 && idx_v < q_ik_result.size()) {
            q_desired_[idx_v] = q_ik_result[idx_v];
          }
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Left arm IK did not converge. Target: [%.3f, %.3f, %.3f]",
                           trans[0], trans[1], trans[2]);
    }
  }

  // Process right arm target (skip if right arm is homing)
  auto right_msg = rt_right_target_.readFromRT();
  if (!homing_right_ && right_msg && (*right_msg) && right_target_received_) {
    const auto& pose = (*right_msg)->pose;
    
    Eigen::Quaterniond quat(pose.orientation.w, pose.orientation.x,
                             pose.orientation.y, pose.orientation.z);
    Eigen::Vector3d trans(pose.position.x, pose.position.y, pose.position.z);
    if (quat.norm() < 1e-6) {
      quat = Eigen::Quaterniond::Identity();
    } else {
      quat.normalize();
    }

    pinocchio::SE3 target_pose(quat.toRotationMatrix(), trans);
    
    Eigen::VectorXd q_ik_result;
    bool ik_success = dynamics_->solveIK(
      right_frame_id_, target_pose, q_desired_,
      q_ik_result, ik_max_iter_, ik_eps_, ik_damping_, ik_dt_);
    
    if (ik_success) {
      for (size_t i = 0; i < joint_names_.size(); ++i) {
        if (joint_names_[i].find("right_") == 0) {
          int idx_v = joint_idx_v_[i];
          if (idx_v >= 0 && idx_v < q_ik_result.size()) {
            q_desired_[idx_v] = q_ik_result[idx_v];
          }
        }
      }
    } else {
      RCLCPP_WARN_THROTTLE(get_node()->get_logger(), *get_node()->get_clock(), 2000,
                           "Right arm IK did not converge. Target: [%.3f, %.3f, %.3f]",
                           trans[0], trans[1], trans[2]);
    }
  }

  // Smooth interpolation: move q_command_ towards q_desired_ at limited speed
  double max_step = position_interpolation_speed_ * dt;
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    int idx_v = joint_idx_v_[i];
    if (idx_v >= 0 && idx_v < q_command_.size()) {
      double error = q_desired_[idx_v] - q_command_[idx_v];
      double step = std::max(-max_step, std::min(error, max_step));
      q_command_[idx_v] += step;
    }
  }

  // Write joint position commands to effort command interfaces
  // (effort interface is used as data conduit; MuJoCo position actuators interpret d->ctrl as position)
  for (size_t i = 0; i < joint_names_.size(); ++i) {
    int idx_v = joint_idx_v_[i];
    if (idx_v >= 0 && idx_v < q_command_.size()) {
      command_interfaces_[i].set_value(q_command_[idx_v]);
    } else {
      command_interfaces_[i].set_value(0.0);
    }
  }

  // Publish feedback
  if (feedback_pub_ && feedback_pub_->get_subscription_count() > 0) {
    auto feedback_msg = std::make_shared<sensor_msgs::msg::JointState>();
    feedback_msg->header.stamp = get_node()->now();
    feedback_msg->name = joint_names_;
    feedback_msg->position.resize(joint_names_.size());
    feedback_msg->velocity.resize(joint_names_.size());
    feedback_msg->effort.resize(joint_names_.size());
    
    for (size_t i = 0; i < joint_names_.size(); ++i) {
      int idx_v = joint_idx_v_[i];
      if (idx_v >= 0) {
        feedback_msg->position[i] = q_command_[idx_v];   // commanded position
        feedback_msg->velocity[i] = q_[idx_v];           // actual position
        feedback_msg->effort[i] = q_desired_[idx_v];     // IK target position
      }
    }
    
    feedback_pub_->publish(*feedback_msg);
  }

  return controller_interface::return_type::OK;
}

}  // namespace eiriarm_controllers

// Export the plugin
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  eiriarm_controllers::CartesianPositionControllerPlugin,
  controller_interface::ControllerInterface)
