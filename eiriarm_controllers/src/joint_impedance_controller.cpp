#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

class JointImpedanceController : public rclcpp::Node
{
public:
  JointImpedanceController()
  : Node("joint_impedance_controller")
  {
    // Global Parameters
    this->declare_parameter("urdf_path", "");
    this->declare_parameter("robot_description", "");
    this->declare_parameter("stiffness", 20.0); // Conservative default
    this->declare_parameter("damping", 5.0);    // Conservative default
    this->declare_parameter("control_frequency", 300.0);
    this->declare_parameter("max_effort", 50.0); // Safety limit
    this->declare_parameter("position_error_limit", 0.087); // ~5 degrees in radians
    this->declare_parameter("velocity_filter_alpha", 0.99); // Low-pass filter for velocity (0.99 = heavy filtering)
    this->declare_parameter("velocity_saturation", 5.0); // Saturate filtered velocity to prevent torque spikes (rad/s)

    std::string urdf_path = this->get_parameter("urdf_path").as_string();
    std::string robot_desc = this->get_parameter("robot_description").as_string();
    default_stiffness_ = this->get_parameter("stiffness").as_double();
    default_damping_ = this->get_parameter("damping").as_double();
    max_effort_ = this->get_parameter("max_effort").as_double();
    pos_error_limit_ = this->get_parameter("position_error_limit").as_double();
    velocity_filter_alpha_ = this->get_parameter("velocity_filter_alpha").as_double();
    velocity_saturation_ = this->get_parameter("velocity_saturation").as_double();
    double control_freq = this->get_parameter("control_frequency").as_double();

    // Load Pinocchio Model
    try {
      if (!robot_desc.empty()) {
        RCLCPP_INFO(this->get_logger(), "Loading model from robot_description parameter...");
        pinocchio::urdf::buildModelFromXML(robot_desc, model_);
      } else if (!urdf_path.empty()) {
        RCLCPP_INFO(this->get_logger(), "Loading URDF from file: %s", urdf_path.c_str());
        pinocchio::urdf::buildModel(urdf_path, model_);
      } else {
        RCLCPP_ERROR(this->get_logger(), "No URDF source provided. Set 'robot_description' or 'urdf_path'.");
        return;
      }
      
      data_ = std::make_unique<pinocchio::Data>(model_);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to build pinocchio model: %s", e.what());
      throw;
    }

    RCLCPP_INFO(this->get_logger(), "Model loaded. nq=%d, nv=%d", model_.nq, model_.nv);
    
    // CRITICAL: Explicitly set gravity vector
    model_.gravity.linear(Eigen::Vector3d(0.0, 0.0, -9.81));
    RCLCPP_INFO(this->get_logger(), "Gravity set to: [%.2f, %.2f, %.2f]", 
                model_.gravity.linear()[0], model_.gravity.linear()[1], model_.gravity.linear()[2]);
    
    // Initialize vectors
    q_ = pinocchio::neutral(model_);
    v_ = Eigen::VectorXd::Zero(model_.nv);
    v_filtered_ = Eigen::VectorXd::Zero(model_.nv);  // Filtered velocity
    q_d_ = pinocchio::neutral(model_);
    v_d_ = Eigen::VectorXd::Zero(model_.nv);
    
    // Per-joint gains map
    for (pinocchio::JointIndex joint_id = 1; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id) {
        if (model_.joints[joint_id].nv() > 0) {
            std::string name = model_.names[joint_id];
            
            // Declare and get initial values
            this->declare_parameter("gains." + name + ".stiffness", default_stiffness_);
            this->declare_parameter("gains." + name + ".damping", default_damping_);
            
            // Get effort limit from URDF
            double effort_limit = max_effort_;  // Default
            if (model_.joints[joint_id].nv() > 0) {
                // Pinocchio stores effort limits in model_.effortLimit
                int idx_v = model_.joints[joint_id].idx_v();
                if (idx_v >= 0 && idx_v < model_.effortLimit.size()) {
                    effort_limit = model_.effortLimit[idx_v];
                }
            }
            
            joint_gains_[name] = {
                this->get_parameter("gains." + name + ".stiffness").as_double(),
                this->get_parameter("gains." + name + ".damping").as_double(),
                effort_limit
            };
        }
    }

    // Parameter Callback
    param_callback_handle_ = this->add_on_set_parameters_callback(
        std::bind(&JointImpedanceController::on_parameter_change, this, std::placeholders::_1));

    // Subscribers
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&JointImpedanceController::joint_state_callback, this, std::placeholders::_1));

    target_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/impedance/target_joint_states", 10,
      std::bind(&JointImpedanceController::target_callback, this, std::placeholders::_1));

    // Publishers
    effort_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ctrl/effort", 10);
    feedback_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_impedance_controller/effort_feedback", 10);

    // Timer
    if (control_freq <= 0.0) control_freq = 300.0;
    std::chrono::microseconds period(static_cast<int>(1000000.0 / control_freq));
    
    RCLCPP_INFO(this->get_logger(), "Control loop running at %.1f Hz (velocity filter alpha=%.3f)", 
                control_freq, velocity_filter_alpha_);
    
    timer_ = this->create_wall_timer(
      period,
      std::bind(&JointImpedanceController::control_loop, this));
  }

private:
  struct JointGains {
      double k_p;
      double k_d;
      double effort_limit;  // Per-joint effort limit from URDF
  };
  std::map<std::string, JointGains> joint_gains_;

  rcl_interfaces::msg::SetParametersResult on_parameter_change(
      const std::vector<rclcpp::Parameter> & parameters)
  {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & param : parameters) {
          if (param.get_name() == "max_effort") {
              max_effort_ = param.as_double();
          }
          if (param.get_name() == "position_error_limit") {
              pos_error_limit_ = param.as_double();
          }
          // Check for gains updates
          // Name format: gains.<joint_name>.[stiffness|damping]
          if (param.get_name().find("gains.") == 0) {
              // Parse string to find joint name and type
              // Simplified: just re-read all params in loop or map update?
              // Optimization: Update the map directly
              // e.g. gains.left_joint_0.stiffness
              size_t first_dot = 5; // "gains." length
              size_t last_dot = param.get_name().rfind('.');
              if (last_dot != std::string::npos && last_dot > first_dot) {
                  std::string joint_name = param.get_name().substr(first_dot, last_dot - first_dot);
                  std::string type = param.get_name().substr(last_dot + 1);
                  
                  if (joint_gains_.count(joint_name)) {
                      if (type == "stiffness") joint_gains_[joint_name].k_p = param.as_double();
                      else if (type == "damping") joint_gains_[joint_name].k_d = param.as_double();
                  }
              }
          }
      }
      return result;
  }

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Check if velocity is present
    bool has_velocity = (msg->velocity.size() == msg->position.size());
    if (!has_velocity) {
        // Warn only once
        static bool warned_velocity = false;
        if (!warned_velocity) {
            RCLCPP_WARN(this->get_logger(), "JointState message has no velocity! Damping will be ineffective.");
            warned_velocity = true;
        }
    }

    // Update current state q_ and v_
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& name = msg->name[i];
      if (model_.existJointName(name)) {
        pinocchio::JointIndex id = model_.getJointId(name);
        int idx_q = model_.joints[id].idx_q();
        int idx_v = model_.joints[id].idx_v();
        
        if (idx_q >= 0 && idx_q < model_.nq) {
          q_[idx_q] = msg->position[i];
        }
        if (idx_v >= 0 && idx_v < model_.nv) {
            if (has_velocity) {
                double v_raw = msg->velocity[i];
                // Apply low-pass filter: v_filtered = (1-alpha)*v_filtered + alpha*v_raw
                v_filtered_[idx_v] = (1.0 - velocity_filter_alpha_) * v_filtered_[idx_v] + velocity_filter_alpha_ * v_raw;
                v_[idx_v] = v_raw;  // Keep raw velocity for reference
            } else {
                v_[idx_v] = 0.0;
                v_filtered_[idx_v] = 0.0;
            }
        }
      }
    }
    
    // If no target received yet, capture current state as target ONCE to hold position
    // Like Franka: capture immediately without waiting for robot to be stationary
    if (!target_received_ && !initial_pose_captured_) {
        // Capture as soon as we have valid joint state data
        if (last_joint_state_msg_) {
            q_d_ = q_;
            v_d_ = Eigen::VectorXd::Zero(model_.nv); 
            initial_pose_captured_ = true;
            RCLCPP_INFO(this->get_logger(), "Initial pose captured at current position. Impedance control active.");
        }
    }
    
    last_joint_state_msg_ = msg;
  }

  void target_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    target_received_ = true;
    
    // Update target state q_d_ and v_d_
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& name = msg->name[i];
      if (model_.existJointName(name)) {
        pinocchio::JointIndex id = model_.getJointId(name);
        int idx_q = model_.joints[id].idx_q();
        int idx_v = model_.joints[id].idx_v();
        
        if (idx_q >= 0 && idx_q < model_.nq && i < msg->position.size()) {
          q_d_[idx_q] = msg->position[i];
        }
        if (idx_v >= 0 && idx_v < model_.nv && i < msg->velocity.size()) {
          v_d_[idx_v] = msg->velocity[i];
        }
      }
    }
  }

  void control_loop()
  {
    if (!last_joint_state_msg_) {
        return; // Wait for first joint state
    }
    
    // Compute Generalized Gravity g(q)
    // Note: computeGeneralizedGravity internally updates kinematics as needed
    pinocchio::computeGeneralizedGravity(model_, *data_, q_);
    
    // CRITICAL: If initial pose not captured yet, ONLY publish gravity compensation
    // This prevents controller from trying to drive to neutral position while robot is moving
    if (!initial_pose_captured_ && !target_received_) {
        sensor_msgs::msg::JointState gravity_only_msg;
        gravity_only_msg.header.stamp = this->now();
        
        for (pinocchio::JointIndex joint_id = 1; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id) {
            if (model_.joints[joint_id].nv() > 0) {
                std::string name = model_.names[joint_id];
                int idx_v = model_.joints[joint_id].idx_v();
                double tau_g = data_->g[idx_v];
                
                gravity_only_msg.name.push_back(name);
                gravity_only_msg.effort.push_back(tau_g);
            }
        }
        
        effort_pub_->publish(gravity_only_msg);
        return;
    }
    
    // Calculate Torques
    sensor_msgs::msg::JointState total_effort_msg;
    sensor_msgs::msg::JointState feedback_effort_msg;
    
    total_effort_msg.header.stamp = this->now();
    feedback_effort_msg.header.stamp = total_effort_msg.header.stamp;

    for (pinocchio::JointIndex joint_id = 1; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id) {
        if (model_.joints[joint_id].nv() > 0) {
            std::string name = model_.names[joint_id];
            int idx_q = model_.joints[joint_id].idx_q();
            int idx_v = model_.joints[joint_id].idx_v();
            
            // Get per-joint gains and effort limit
            double k_p = default_stiffness_;
            double k_d = default_damping_;
            double joint_effort_limit = max_effort_;
            
            if (joint_gains_.count(name)) {
                k_p = joint_gains_[name].k_p;
                k_d = joint_gains_[name].k_d;
                joint_effort_limit = joint_gains_[name].effort_limit;
            }
            
            // Gravity Term
            double tau_g = data_->g[idx_v];
            
            // Impedance Term: Kp * clamp(qd - q) + Kd * (vd - v)
            double q_curr = q_[idx_q];
            double v_curr = v_[idx_v];
            double q_des = q_d_[idx_q];
            double v_des = v_d_[idx_v];
            
            double pos_error = q_des - q_curr;
            // Clamp error magnitude
            double clamped_pos_error = std::max(-pos_error_limit_, std::min(pos_error, pos_error_limit_));

            // Use FILTERED velocity for damping term
            double v_curr_filtered = v_filtered_[idx_v];
            
            // Simple impedance control: tau = Kp*(qd-q) + Kd*(vd-v) + g(q)
            double tau_pos = k_p * clamped_pos_error;
            double tau_vel = k_d * (v_des - v_curr_filtered);
            double tau_imp = tau_pos + tau_vel;
            
            // Total Torque
            double tau_total = tau_g + tau_imp;
            
            // Safety Clamp using per-joint limit
            double clamped_effort = std::max(-joint_effort_limit, std::min(tau_total, joint_effort_limit));
            
            // Continuous debug output for left_joint_2 to diagnose oscillation
            if (name == "left_joint_2") {
                static int debug_counter = 0;
                static double prev_q = q_curr;
                static double prev_v = v_curr;
                
                // Detect position jumps (> 0.1 rad in one step = 50 rad/s at 500Hz)
                double q_jump = std::abs(q_curr - prev_q);
                double v_jump = std::abs(v_curr - prev_v);
                bool has_jump = (q_jump > 0.1 || v_jump > 20.0);
                
                if (debug_counter++ % 100 == 0 || has_jump) {  // Print every 100 iterations or on jump
                    bool is_clamped = (std::abs(tau_total) > joint_effort_limit);
                    
                    RCLCPP_INFO(this->get_logger(), 
                        "[%s] q=%.3f->%.3f(err=%.3f) Δq=%.3f | v_raw=%.2f Δv=%.2f v_filt=%.2f | "
                        "tau: pos=%.1f vel=%.1f grav=%.1f => total=%.1f %s%s | Kp=%.1f Kd=%.1f",
                        name.c_str(), q_curr, q_des, pos_error, q_jump, v_curr, v_jump, v_curr_filtered,
                        tau_pos, tau_vel, tau_g, tau_total,
                        is_clamped ? "CLAMP" : "     ",
                        has_jump ? " JUMP!" : "",
                        k_p, k_d);
                }
                
                prev_q = q_curr;
                prev_v = v_curr;
            }
            
            // Warning for any joint if torque exceeds limit
            if (std::abs(tau_total) > joint_effort_limit) {
                 RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                     "Torque limit exceeded on %s: Total=%.2f (Grav=%.2f, Imp=%.2f). q=%.3f, qd=%.3f, v_raw=%.3f, v_filt=%.3f, Kp=%.1f, Kd=%.1f. Limit=%.1f, Clamped to %.2f",
                     name.c_str(), tau_total, tau_g, tau_imp, q_curr, q_des, v_curr, v_curr_filtered, k_p, k_d, joint_effort_limit, clamped_effort);
            }

            total_effort_msg.name.push_back(name);
            total_effort_msg.effort.push_back(clamped_effort);
            
            feedback_effort_msg.name.push_back(name);
            feedback_effort_msg.effort.push_back(tau_imp);
        }
    }
    
    effort_pub_->publish(total_effort_msg);
    feedback_pub_->publish(feedback_effort_msg);
  }

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  
  Eigen::VectorXd q_, v_, v_filtered_;
  Eigen::VectorXd q_d_, v_d_;
  
  bool target_received_ = false;
  bool initial_pose_captured_ = false;
  double default_stiffness_;
  double default_damping_;
  double max_effort_;
  double pos_error_limit_;
  double velocity_filter_alpha_;
  double velocity_saturation_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr target_sub_;
  
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr effort_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr feedback_pub_;
  
  rclcpp::TimerBase::SharedPtr timer_;
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg_;
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr param_callback_handle_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<JointImpedanceController>());
  rclcpp::shutdown();
  return 0;
}
