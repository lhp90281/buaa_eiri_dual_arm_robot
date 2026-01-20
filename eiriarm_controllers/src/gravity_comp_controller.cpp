#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

class GravityCompController : public rclcpp::Node
{
public:
  GravityCompController()
  : Node("gravity_comp_controller")
  {
    // Parameters
    this->declare_parameter("urdf_path", "");
    this->declare_parameter("robot_description", "");
    this->declare_parameter("gravity_gain", 0.9);
    
    std::string urdf_path = this->get_parameter("urdf_path").as_string();
    std::string robot_desc = this->get_parameter("robot_description").as_string();
    gravity_gain_ = this->get_parameter("gravity_gain").as_double();

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

    // Initialize q vector
    q_ = pinocchio::neutral(model_);

    // Subscriber
    joint_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10,
      std::bind(&GravityCompController::joint_state_callback, this, std::placeholders::_1));

    // Publisher
    effort_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("/ctrl/effort", 10);

    // Timer (300Hz)
    // 1.0 / 300.0 = 0.003333... seconds
    timer_ = this->create_wall_timer(
      std::chrono::microseconds(3333),
      std::bind(&GravityCompController::control_loop, this));
  }

private:
  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    // Update current joint configuration q_
    // We need to map msg->name to model joint names
    
    // Create map on first run or assuming names don't change often?
    // Doing linear search every callback might be slow if many joints, but for 20 joints it's fine.
    // Better: use a map.
    
    for (size_t i = 0; i < msg->name.size(); ++i) {
      const std::string& name = msg->name[i];
      if (model_.existJointName(name)) {
        pinocchio::JointIndex id = model_.getJointId(name);
        // Note: For simple revolute/prismatic joints, joint_id maps to 1 q index.
        // model_.joints[id].idx_q() gives the index in q.
        int idx_q = model_.joints[id].idx_q();
        if (idx_q >= 0 && idx_q < model_.nq) {
            q_[idx_q] = msg->position[i];
        }
      }
    }
    
    last_joint_state_msg_ = msg; // Store names for publishing if needed, or we publish all actuated
  }

  void control_loop()
  {
    if (!last_joint_state_msg_) {
        return; // Wait for first joint state
    }

    // Compute gravity compensation torques (g(q))
    pinocchio::computeGeneralizedGravity(model_, *data_, q_);

    // Apply gain
    data_->g *= gravity_gain_;

    // Publish efforts
    sensor_msgs::msg::JointState effort_msg;
    effort_msg.header.stamp = this->now();
    
    // We can either publish efforts for all joints in the model, or just the ones we received.
    // Ideally we publish for all actuated joints.
    // Iterate over all joints in model (skipping universe which is 0)
    for (pinocchio::JointIndex joint_id = 1; joint_id < (pinocchio::JointIndex)model_.njoints; ++joint_id) {
        // Check if joint is actuated (has DoF)
        if (model_.joints[joint_id].nv() > 0) {
            std::string name = model_.names[joint_id];
            
            // Get torque index
            int idx_v = model_.joints[joint_id].idx_v();
            
            // Gravity torque
            double gravity_torque = data_->g[idx_v];

            // Debug print for first few joints to analyze gravity compensation distribution
            if (name.find("left_joint_") != std::string::npos) {
                int joint_num = -1;
                try {
                    joint_num = std::stoi(name.substr(11)); // "left_joint_" is 11 chars
                } catch (...) {}
                
                if (joint_num >= 0 && joint_num <= 5) {
                    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500, 
                        "Joint %s (idx_v=%d) Gravity Torque: %f (Gain: %.2f)", name.c_str(), idx_v, gravity_torque, gravity_gain_);
                }
            }

            effort_msg.name.push_back(name);
            effort_msg.effort.push_back(gravity_torque);
        }
    }

    effort_pub_->publish(effort_msg);
  }

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  Eigen::VectorXd q_;
  double gravity_gain_;
  
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr effort_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  
  sensor_msgs::msg::JointState::SharedPtr last_joint_state_msg_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<GravityCompController>());
  rclcpp::shutdown();
  return 0;
}
