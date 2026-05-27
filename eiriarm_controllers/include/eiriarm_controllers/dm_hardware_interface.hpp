#ifndef EIRIARM_CONTROLLERS__DM_HARDWARE_INTERFACE_HPP_
#define EIRIARM_CONTROLLERS__DM_HARDWARE_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "usb2can/msg/motor_command_array.hpp"
#include "usb2can/msg/motor_enable_array.hpp"
#include "usb2can/msg/motor_state_array.hpp"

namespace eiriarm_controllers
{

/**
 * @brief ros2_control SystemInterface that talks directly to DM motors over usb2can.
 *
 * State interfaces (per joint, in URDF frame):
 *   - position
 *   - velocity
 *   - effort
 *
 * Command interfaces (per joint, in URDF frame, MIT full set):
 *   - position
 *   - velocity
 *   - effort        (interpreted as feed-forward torque tau_ff)
 *   - stiffness     (kp, Nm/rad)
 *   - damping       (kd, Nm/(rad/s))
 *
 * Controllers may write only a subset; unused commands should be set to 0.0
 * by the controller (or left at the default 0.0 set in on_activate()).
 *
 * URDF ros2_control snippet (per joint):
 *   <joint name="joint1">
 *     <param name="channel">1</param>
 *     <param name="slot">0</param>
 *     <param name="motor_type">DM8009</param>
 *     <param name="urdf_lower">-3.14</param>
 *     <param name="urdf_upper">3.14</param>
 *     <command_interface name="effort"/>
 *     <command_interface name="position"/>
 *     <command_interface name="velocity"/>
 *     <command_interface name="stiffness"/>
 *     <command_interface name="damping"/>
 *     <state_interface  name="position"/>
 *     <state_interface  name="velocity"/>
 *     <state_interface  name="effort"/>
 *   </joint>
 *
 * Hardware-level params (in <hardware> block):
 *   <param name="offsets_yaml">/abs/path/to/joint_offsets.yaml</param>
 *   <param name="auto_enable">true</param>           (default true)
 *   <param name="motor_topic_ns">/motor</param>      (default /motor)
 */
class DMHardwareInterface : public hardware_interface::SystemInterface
{
public:
  DMHardwareInterface();
  ~DMHardwareInterface() override = default;

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  struct JointCfg
  {
    std::string name;
    int channel{1};
    int slot{0};
    std::string motor_type;
    // calibration (from offsets yaml)
    double zero_offset{0.0};
    double axis_sign{1.0};
    // URDF range (from xacro <param>)
    double urdf_lower{-M_PI};
    double urdf_upper{M_PI};
    double range_center{0.0};
    bool wrap_safe{true};
    // motor saturation (raw motor frame); from motor_type lookup
    double pos_max{12.5};
    double vel_max{30.0};
    double tor_max{10.0};
  };

  // ---- helpers ----
  static double wrap_to_window(double x, double center);
  double raw_to_urdf_pos(const JointCfg & j, double raw) const;
  double urdf_to_raw_pos(const JointCfg & j, double urdf_pos) const;
  // Multi-turn-aware inverse of raw_to_urdf_pos. When wrap_safe is true,
  // raw_to_urdf_pos folds the linear conversion into (range_center - pi,
  // range_center + pi], so urdf_to_raw_pos by itself only recovers the
  // canonical revolution. If the motor is actually in a different
  // revolution (e.g. its multi-turn counter wrapped), commanding the
  // canonical raw value would force the motor to spin a full 2 pi to
  // reach the target. This variant picks the integer k that minimises
  // |linear + k*2pi - current_raw|, snapping the command to the same
  // revolution the motor is currently in.
  double urdf_to_raw_pos_near(const JointCfg & j, double urdf_pos,
                              double current_raw) const;
  bool load_offsets_yaml(const std::string & path);
  void on_motor_state(int channel, const usb2can::msg::MotorStateArray::SharedPtr msg);
  void publish_enable_all(bool enable);
  void publish_zero_command_all();

  // ---- joint config (parallel arrays sized n_joints_) ----
  std::vector<JointCfg> joints_;
  std::unordered_map<std::string, size_t> joint_name_to_index_;
  std::vector<int> unique_channels_;
  std::map<int, std::vector<size_t>> joints_by_channel_;  // channel -> joint indices

  // ---- state storage (URDF frame, ros2_control reads from these) ----
  std::vector<double> state_pos_;
  std::vector<double> state_vel_;
  std::vector<double> state_eff_;

  // ---- command storage (URDF frame, ros2_control writes into these) ----
  std::vector<double> cmd_pos_;
  std::vector<double> cmd_vel_;
  std::vector<double> cmd_eff_;
  std::vector<double> cmd_kp_;
  std::vector<double> cmd_kd_;

  // ---- ROS plumbing ----
  rclcpp::Node::SharedPtr node_;
  std::map<int, rclcpp::Subscription<usb2can::msg::MotorStateArray>::SharedPtr> state_subs_;
  std::map<int, rclcpp::Publisher<usb2can::msg::MotorCommandArray>::SharedPtr> cmd_pubs_;
  std::map<int, rclcpp::Publisher<usb2can::msg::MotorEnableArray>::SharedPtr> enable_pubs_;
  std::map<int, usb2can::msg::MotorStateArray::SharedPtr> latest_state_;
  std::map<int, bool> state_seen_;
  // Per-joint MotorState.err code from the last *non-sentinel* frame.
  // -1 means "no real state observed yet". DM motor state code:
  //   0 = disabled, 1 = enabled (MIT mode), 8..14 = various error states
  //   (overvoltage / undervoltage / overcurrent / overtemp / comm-lost /
  //   overload). on_activate / on_deactivate poll this to confirm that
  //   the motor actually entered / exited MIT mode -- pos/vel/tor are
  //   reported by the bridge regardless of enable state (DM is purely
  //   request-response), so err is the only reliable indicator.
  std::vector<int> last_err_;

  // Last *non-sentinel* raw position (motor frame, multi-turn, NOT wrapped)
  // per joint. write() uses this to mirror pos_cmd <- pos_actual whenever
  // the controller runs in pure-torque mode (kp==0 && kd==0), so that the
  // STM32 watchdog fallback (kp=1, kd=1, last pos_cmd) cannot snap the
  // motor toward a stale target on a comm drop. Initialized to 0 -- safe
  // because on_activate() only succeeds after every motor has reported
  // err=1, which means at least one non-sentinel state frame was received
  // and this vector has been populated for every joint.
  std::vector<double> last_pos_raw_;

  // ---- behaviour params ----
  std::string offsets_yaml_path_;
  std::string motor_topic_ns_{"/motor"};
  bool auto_enable_{true};

  // When set to true by on_deactivate, write() emits zero-cmd frames
  // instead of the live cmd_* values. Once the motor is disabled it stops
  // responding to cmd; this flag just avoids racing the disable with a
  // non-zero command from a still-active controller.
  bool quiescing_{false};
};

}  // namespace eiriarm_controllers

#endif  // EIRIARM_CONTROLLERS__DM_HARDWARE_INTERFACE_HPP_
