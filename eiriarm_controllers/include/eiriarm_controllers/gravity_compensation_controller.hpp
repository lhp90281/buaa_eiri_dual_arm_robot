#ifndef EIRIARM_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_
#define EIRIARM_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_

#include <memory>
#include <string>
#include <vector>
#include <map>

#include "controller_interface/controller_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "eiriarm_controllers/eiriarm_dynamics.hpp"

namespace eiriarm_controllers
{

/**
 * @brief Gravity Compensation Controller for ros2_control
 * 
 * This controller implements pure gravity compensation (zero stiffness, zero damping).
 * Formula: tau = g(q)
 * 
 * This is essentially the impedance controller with Kp=0 and Kd=0.
 * The robot will feel "weightless" and can be freely moved by hand.
 */
class GravityCompensationController : public controller_interface::ControllerInterface
{
public:
  GravityCompensationController();

  /**
   * @brief Get command interface configuration
   */
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  /**
   * @brief Get state interface configuration
   */
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  /**
   * @brief Initialize the controller
   */
  controller_interface::CallbackReturn on_init() override;

  /**
   * @brief Configure the controller
   */
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the controller
   */
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the controller
   */
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Update control command (real-time loop)
   */
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Per-joint gains structure
  struct JointGains {
    double effort_limit;
  };

  // Algorithm library
  std::unique_ptr<eiriarm_dynamics::EiriarmDynamics> dynamics_;

  // Parameters
  std::vector<std::string> joint_names_;
  std::map<std::string, JointGains> joint_gains_;
  double max_effort_ = 50.0;
  double velocity_filter_alpha_ = 0.99;

  // State storage
  Eigen::VectorXd q_;
  Eigen::VectorXd v_;
  Eigen::VectorXd v_filtered_;
};

}  // namespace eiriarm_controllers

#endif  // EIRIARM_CONTROLLERS__GRAVITY_COMPENSATION_CONTROLLER_HPP_
