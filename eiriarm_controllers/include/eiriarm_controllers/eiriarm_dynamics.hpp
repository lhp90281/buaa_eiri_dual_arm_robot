#ifndef EIRIARM_CONTROLLERS__EIRIARM_DYNAMICS_HPP_
#define EIRIARM_CONTROLLERS__EIRIARM_DYNAMICS_HPP_

#include <memory>
#include <string>
#include <Eigen/Dense>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace eiriarm_dynamics
{

/**
 * @brief Pure algorithm class for robot dynamics computation using Pinocchio
 * 
 * This class encapsulates all Pinocchio-related computations (gravity, impedance, etc.)
 * and provides a clean interface for controllers to use.
 * It is designed to be real-time safe and independent of ROS infrastructure.
 */
class EiriarmDynamics
{
public:
  /**
   * @brief Constructor
   */
  EiriarmDynamics() = default;

  /**
   * @brief Initialize the dynamics model from URDF string
   * @param urdf_string URDF content as XML string
   * @return true if successful, false otherwise
   */
  bool initFromURDF(const std::string& urdf_string);

  /**
   * @brief Update the current joint state
   * @param q Joint positions (size must match model.nq)
   * @param v Joint velocities (size must match model.nv)
   */
  void updateState(const Eigen::VectorXd& q, const Eigen::VectorXd& v);

  /**
   * @brief Compute generalized gravity torque g(q)
   * @return Gravity torque vector (size model.nv)
   */
  Eigen::VectorXd computeGravity();

  /**
   * @brief Compute impedance control torque
   * @param q_desired Desired joint positions
   * @param v_desired Desired joint velocities
   * @param stiffness Diagonal stiffness matrix (Kp)
   * @param damping Diagonal damping matrix (Kd)
   * @return Impedance torque vector: tau = Kp*(qd-q) + Kd*(vd-v)
   */
  Eigen::VectorXd computeImpedance(
    const Eigen::VectorXd& q_desired,
    const Eigen::VectorXd& v_desired,
    const Eigen::VectorXd& stiffness,
    const Eigen::VectorXd& damping);

  /**
   * @brief Get the number of configuration variables (nq)
   */
  int getNq() const { return model_.nq; }

  /**
   * @brief Get the number of velocity variables (nv)
   */
  int getNv() const { return model_.nv; }

  /**
   * @brief Get the number of joints (excluding universe)
   */
  int getNumJoints() const { return model_.njoints - 1; }

  /**
   * @brief Check if model is initialized
   */
  bool isInitialized() const { return initialized_; }

  /**
   * @brief Get joint name by index
   * @param joint_id Joint index (1-based, 0 is universe)
   */
  std::string getJointName(int joint_id) const;

  /**
   * @brief Get joint configuration index
   * @param joint_id Joint index
   */
  int getJointIdxQ(int joint_id) const;

  /**
   * @brief Get joint velocity index
   * @param joint_id Joint index
   */
  int getJointIdxV(int joint_id) const;

  /**
   * @brief Get effort limit for a joint
   * @param joint_id Joint index
   */
  double getEffortLimit(int joint_id) const;

  /**
   * @brief Check if a joint has DoF
   * @param joint_id Joint index
   */
  bool isActuated(int joint_id) const;

  /**
   * @brief Get neutral configuration
   */
  Eigen::VectorXd getNeutralConfiguration() const;

private:
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  
  Eigen::VectorXd q_current_;
  Eigen::VectorXd v_current_;
  
  bool initialized_ = false;
};

}  // namespace eiriarm_dynamics

#endif  // EIRIARM_CONTROLLERS__EIRIARM_DYNAMICS_HPP_
