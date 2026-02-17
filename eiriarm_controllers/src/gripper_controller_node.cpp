#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>

#include <string>
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>

namespace eiriarm_controllers
{

/**
 * @brief Velocity-actuator Gripper Controller
 *
 * Works with MuJoCo <velocity> actuators on gripper joints.
 * Publishes desired velocity (not effort) via /ctrl/effort topic.
 * MuJoCo's velocity servo internally computes: force = kv * (ctrl - vel).
 *
 * States:
 *   HOLD:         Send vel = kp_hold * (target_pos - pos), clamped
 *   MOVING_CLOSE: Send vel = +close_speed
 *   MOVING_OPEN:  Send vel = -open_speed
 *
 * Auto-stop (MOVING → HOLD):
 *   - |force_sensor| > force_threshold (gripped object or hit limit)
 *   - Position reaches limit (safety backup)
 *   - Grace period after move start to avoid false force triggers
 */
class GripperController : public rclcpp::Node
{
public:
  enum class GripperState {
    HOLD,
    FORCE_HOLD,
    MOVING_CLOSE,
    MOVING_OPEN
  };

  struct GripperData {
    std::string name;
    std::string finger_joint;
    GripperState state = GripperState::HOLD;

    double position = 0.0;
    double velocity = 0.0;
    double effort = 0.0;        // Force sensor reading

    double target_position = 0.0;  // Used in HOLD mode
    int move_cycles = 0;           // Cycles since movement started
    double force_integral = 0.0;   // Integral of force error for PI control

    bool joint_data_received = false;
  };

  GripperController()
  : Node("gripper_controller")
  {
    // Declare parameters
    this->declare_parameter<double>("open_speed", 0.045);
    this->declare_parameter<double>("close_speed", 0.045);
    this->declare_parameter<double>("kp_hold", 50.0);
    this->declare_parameter<double>("max_hold_vel", 0.1);
    this->declare_parameter<double>("force_threshold", 10.0);
    this->declare_parameter<double>("grip_force", 5.0);
    this->declare_parameter<double>("kp_force", 0.005);
    this->declare_parameter<double>("ki_force", 0.02);
    this->declare_parameter<double>("max_force_integral", 0.05);
    this->declare_parameter<int>("force_delay_cycles", 50);
    this->declare_parameter<double>("position_open", 0.0);
    this->declare_parameter<double>("position_close", 0.045);
    this->declare_parameter<double>("control_rate", 500.0);
    this->declare_parameter<std::string>("left_finger_joint", "left_gripper_right_finger_joint");
    this->declare_parameter<std::string>("right_finger_joint", "right_gripper_right_finger_joint");
    this->declare_parameter<std::string>("joint_state_topic", "/joint_states");
    this->declare_parameter<std::string>("effort_command_topic", "/ctrl/effort");

    // Get parameters
    open_speed_ = this->get_parameter("open_speed").as_double();
    close_speed_ = this->get_parameter("close_speed").as_double();
    kp_hold_ = this->get_parameter("kp_hold").as_double();
    max_hold_vel_ = this->get_parameter("max_hold_vel").as_double();
    force_threshold_ = this->get_parameter("force_threshold").as_double();
    grip_force_ = this->get_parameter("grip_force").as_double();
    kp_force_ = this->get_parameter("kp_force").as_double();
    ki_force_ = this->get_parameter("ki_force").as_double();
    max_force_integral_ = this->get_parameter("max_force_integral").as_double();
    dt_ = 1.0 / this->get_parameter("control_rate").as_double();
    force_delay_cycles_ = this->get_parameter("force_delay_cycles").as_int();
    position_open_ = this->get_parameter("position_open").as_double();
    position_close_ = this->get_parameter("position_close").as_double();
    double control_rate = this->get_parameter("control_rate").as_double();

    // Setup gripper data
    left_gripper_.name = "left";
    left_gripper_.finger_joint = this->get_parameter("left_finger_joint").as_string();
    left_gripper_.target_position = position_open_;

    right_gripper_.name = "right";
    right_gripper_.finger_joint = this->get_parameter("right_finger_joint").as_string();
    right_gripper_.target_position = position_open_;

    // Subscribe to joint states
    std::string joint_state_topic = this->get_parameter("joint_state_topic").as_string();
    joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      joint_state_topic, 10,
      std::bind(&GripperController::jointStateCallback, this, std::placeholders::_1));

    // Subscribe to gripper commands
    left_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "~/left_gripper/command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        this->commandCallback(left_gripper_, msg);
      });

    right_cmd_sub_ = this->create_subscription<std_msgs::msg::String>(
      "~/right_gripper/command", 10,
      [this](const std_msgs::msg::String::SharedPtr msg) {
        this->commandCallback(right_gripper_, msg);
      });

    // Publisher (sends velocity commands via .effort field to MuJoCo velocity actuators)
    std::string effort_topic = this->get_parameter("effort_command_topic").as_string();
    cmd_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(effort_topic, 10);

    // Status publishers
    left_status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "~/left_gripper/status", 10);
    right_status_pub_ = this->create_publisher<std_msgs::msg::String>(
      "~/right_gripper/status", 10);

    // Control timer
    control_timer_ = this->create_wall_timer(
      std::chrono::duration<double>(1.0 / control_rate),
      std::bind(&GripperController::controlLoop, this));

    RCLCPP_INFO(this->get_logger(),
      "GripperController [velocity actuator mode]");
    RCLCPP_INFO(this->get_logger(),
      "  Speeds: open=%.4f close=%.4f m/s | kp_hold=%.1f | force_thr=%.1f N | grip=%.1f N",
      open_speed_, close_speed_, kp_hold_, force_threshold_, grip_force_);
    RCLCPP_INFO(this->get_logger(),
      "  Joints: L=%s R=%s",
      left_gripper_.finger_joint.c_str(), right_gripper_.finger_joint.c_str());
  }

private:
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    for (size_t i = 0; i < msg->name.size(); ++i) {
      GripperData* g = nullptr;
      if (msg->name[i] == left_gripper_.finger_joint)       g = &left_gripper_;
      else if (msg->name[i] == right_gripper_.finger_joint) g = &right_gripper_;
      if (!g) continue;

      if (i < msg->position.size()) g->position = msg->position[i];
      if (i < msg->velocity.size()) g->velocity = msg->velocity[i];
      if (i < msg->effort.size())   g->effort   = msg->effort[i];
      if (!g->joint_data_received) {
        g->target_position = g->position;
        g->joint_data_received = true;
        RCLCPP_INFO(this->get_logger(),
          "%s gripper: initial pos %.4f m", g->name.c_str(), g->position);
      }
    }
  }

  void commandCallback(GripperData& gripper, const std_msgs::msg::String::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    const std::string& cmd = msg->data;

    if (cmd == "open") {
      gripper.state = GripperState::MOVING_OPEN;
      gripper.move_cycles = 0;
      gripper.force_integral = 0.0;
      //RCLCPP_INFO(this->get_logger(), "%s gripper: MOVING_OPEN", gripper.name.c_str());
    }
    else if (cmd == "close") {
      gripper.state = GripperState::MOVING_CLOSE;
      gripper.move_cycles = 0;
      gripper.force_integral = 0.0;
      //RCLCPP_INFO(this->get_logger(), "%s gripper: MOVING_CLOSE", gripper.name.c_str());
    }
    else if (cmd == "hold") {
      gripper.state = GripperState::HOLD;
      gripper.target_position = std::clamp(gripper.position, position_open_, position_close_);
      /*RCLCPP_INFO(this->get_logger(), "%s gripper: HOLD at %.4f m",
        gripper.name.c_str(), gripper.target_position);*/
    }
    else {
      RCLCPP_WARN(this->get_logger(),
        "%s gripper: unknown command '%s' (use: open/close/hold)",
        gripper.name.c_str(), cmd.c_str());
    }
  }

  void controlLoop()
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    if (!left_gripper_.joint_data_received && !right_gripper_.joint_data_received) return;

    auto cmd_msg = std::make_shared<sensor_msgs::msg::JointState>();
    cmd_msg->header.stamp = this->get_clock()->now();

    if (left_gripper_.joint_data_received) {
      cmd_msg->name.push_back(left_gripper_.finger_joint);
      cmd_msg->effort.push_back(computeCommand(left_gripper_));
    }
    if (right_gripper_.joint_data_received) {
      cmd_msg->name.push_back(right_gripper_.finger_joint);
      cmd_msg->effort.push_back(computeCommand(right_gripper_));
    }

    if (!cmd_msg->name.empty()) cmd_pub_->publish(*cmd_msg);

    publishStatus(left_gripper_, left_status_pub_);
    publishStatus(right_gripper_, right_status_pub_);
  }

  /**
   * @brief Compute velocity command for one gripper
   *
   * Returns desired velocity (m/s), sent to MuJoCo velocity actuator via d->ctrl.
   * MuJoCo internally computes: force = kv_mujoco * (ctrl - joint_vel)
   */
  double computeCommand(GripperData& gripper)
  {
    switch (gripper.state) {
      case GripperState::HOLD: {
        // Near joint limits: passive damping hold (vel_cmd=0)
        // MuJoCo velocity servo at ctrl=0 provides: force = -kv * vel (pure damping)
        // This prevents oscillation from constraint bouncing at limits
        constexpr double limit_margin = 0.002;  // 2mm
        if (gripper.target_position >= position_close_ - limit_margin ||
            gripper.target_position <= position_open_  + limit_margin) {
          return 0.0;
        }
        // Mid-range: active position → velocity P control
        double vel_cmd = kp_hold_ * (gripper.target_position - gripper.position);
        if (gripper.position >= position_close_ && vel_cmd > 0) vel_cmd = 0.0;
        if (gripper.position <= position_open_  && vel_cmd < 0) vel_cmd = 0.0;
        return std::clamp(vel_cmd, -max_hold_vel_, max_hold_vel_);
      }

      case GripperState::FORCE_HOLD: {
        // PI force control: maintain constant grip force
        double force_error = grip_force_ - std::abs(gripper.effort);

        // Integrate with anti-windup (clamp integral)
        gripper.force_integral += force_error * dt_;
        gripper.force_integral = std::clamp(gripper.force_integral,
                                            -max_force_integral_, max_force_integral_);

        double vel_cmd = kp_force_ * force_error + ki_force_ * gripper.force_integral;
        return std::clamp(vel_cmd, -max_hold_vel_, max_hold_vel_);
      }

      case GripperState::MOVING_CLOSE: {
        gripper.move_cycles++;

        // Auto-stop: force exceeded (after grace period) → FORCE_HOLD
        if (gripper.move_cycles > force_delay_cycles_ &&
            std::abs(gripper.effort) > force_threshold_) {
          gripper.state = GripperState::FORCE_HOLD;
          RCLCPP_INFO(this->get_logger(),
            "%s gripper: force %.2f N > %.2f N -> FORCE_HOLD (target %.1f N)",
            gripper.name.c_str(), std::abs(gripper.effort),
            force_threshold_, grip_force_);
          return 0.0;
        }

        // Auto-stop: position limit → HOLD (passive)
        if (gripper.position >= position_close_) {
          gripper.state = GripperState::HOLD;
          gripper.target_position = position_close_;
          // RCLCPP_INFO(this->get_logger(),
          //   "%s gripper: close limit -> HOLD at %.4f m",
          //   gripper.name.c_str(), gripper.target_position);
          return 0.0;
        }

        return close_speed_;
      }

      case GripperState::MOVING_OPEN: {
        gripper.move_cycles++;

        // Auto-stop: force exceeded (after grace period)
        if (gripper.move_cycles > force_delay_cycles_ &&
            std::abs(gripper.effort) > force_threshold_) {
          gripper.state = GripperState::HOLD;
          gripper.target_position = std::clamp(gripper.position, position_open_, position_close_);
          RCLCPP_INFO(this->get_logger(),
            "%s gripper: force %.2f N > %.2f N -> HOLD at %.4f m",
            gripper.name.c_str(), std::abs(gripper.effort),
            force_threshold_, gripper.target_position);
          return 0.0;
        }

        // Auto-stop: position limit
        if (gripper.position <= position_open_) {
          gripper.state = GripperState::HOLD;
          gripper.target_position = position_open_;
          // RCLCPP_INFO(this->get_logger(),
          //   "%s gripper: open limit -> HOLD at %.4f m",
          //   gripper.name.c_str(), gripper.target_position);
          return 0.0;
        }

        return -open_speed_;
      }
    }
    return 0.0;
  }

  void publishStatus(const GripperData& gripper,
                     const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& pub)
  {
    if (!gripper.joint_data_received || pub->get_subscription_count() == 0) return;

    std::string state_str;
    switch (gripper.state) {
      case GripperState::HOLD:         state_str = "hold"; break;
      case GripperState::FORCE_HOLD:   state_str = "force_hold"; break;
      case GripperState::MOVING_CLOSE: state_str = "closing"; break;
      case GripperState::MOVING_OPEN:  state_str = "opening"; break;
    }
    char buf[128];
    std::snprintf(buf, sizeof(buf), "state:%s pos:%.4f force:%.2f",
      state_str.c_str(), gripper.position, gripper.effort);

    auto status_msg = std::make_shared<std_msgs::msg::String>();
    status_msg->data = buf;
    pub->publish(*status_msg);
  }

  // Gripper data
  GripperData left_gripper_;
  GripperData right_gripper_;

  // Parameters
  double open_speed_;
  double close_speed_;
  double kp_hold_;          // Position→velocity gain for HOLD (1/s)
  double max_hold_vel_;     // Max velocity command in HOLD mode (m/s)
  double force_threshold_;
  double grip_force_;       // Target force for FORCE_HOLD (N)
  double kp_force_;         // Force→velocity P gain (m/s per N)
  double ki_force_;         // Force→velocity I gain (m/s per N·s)
  double max_force_integral_;  // Anti-windup limit for integral (m/s)
  double dt_;               // Control period (s)
  int force_delay_cycles_;  // Grace period before force checking
  double position_open_;
  double position_close_;

  // ROS interfaces
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr left_cmd_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr right_cmd_sub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr cmd_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr left_status_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr right_status_pub_;
  rclcpp::TimerBase::SharedPtr control_timer_;

  std::mutex data_mutex_;
};

}  // namespace eiriarm_controllers


int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<eiriarm_controllers::GripperController>();
  RCLCPP_INFO(node->get_logger(), "\033[32mGripper controller started [velocity actuator mode]\033[0m");
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
