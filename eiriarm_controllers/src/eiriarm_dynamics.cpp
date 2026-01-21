#include "eiriarm_controllers/eiriarm_dynamics.hpp"
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/rnea.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <stdexcept>

namespace eiriarm_dynamics
{

bool EiriarmDynamics::initFromURDF(const std::string& urdf_string)
{
  if (urdf_string.empty()) {
    return false;
  }

  try {
    // Build model from URDF XML string
    pinocchio::urdf::buildModelFromXML(urdf_string, model_);
    
    // Create data structure
    data_ = std::make_unique<pinocchio::Data>(model_);
    
    // Set gravity vector (standard: [0, 0, -9.81] in world frame)
    model_.gravity.linear(Eigen::Vector3d(0.0, 0.0, -9.81));
    
    // Initialize state vectors
    q_current_ = pinocchio::neutral(model_);
    v_current_ = Eigen::VectorXd::Zero(model_.nv);
    
    initialized_ = true;
    return true;
    
  } catch (const std::exception& e) {
    initialized_ = false;
    return false;
  }
}

void EiriarmDynamics::updateState(const Eigen::VectorXd& q, const Eigen::VectorXd& v)
{
  if (!initialized_) {
    throw std::runtime_error("EiriarmDynamics not initialized");
  }
  
  if (q.size() != model_.nq || v.size() != model_.nv) {
    throw std::runtime_error("State vector size mismatch");
  }
  
  q_current_ = q;
  v_current_ = v;
}

Eigen::VectorXd EiriarmDynamics::computeGravity()
{
  if (!initialized_) {
    throw std::runtime_error("EiriarmDynamics not initialized");
  }
  
  // Compute generalized gravity g(q)
  pinocchio::computeGeneralizedGravity(model_, *data_, q_current_);
  
  return data_->g;
}

Eigen::VectorXd EiriarmDynamics::computeImpedance(
  const Eigen::VectorXd& q_desired,
  const Eigen::VectorXd& v_desired,
  const Eigen::VectorXd& stiffness,
  const Eigen::VectorXd& damping)
{
  if (!initialized_) {
    throw std::runtime_error("EiriarmDynamics not initialized");
  }
  
  if (q_desired.size() != model_.nq || v_desired.size() != model_.nv ||
      stiffness.size() != model_.nv || damping.size() != model_.nv) {
    throw std::runtime_error("Input vector size mismatch");
  }
  
  // Compute impedance torque: tau = Kp*(qd-q) + Kd*(vd-v)
  Eigen::VectorXd tau_impedance = Eigen::VectorXd::Zero(model_.nv);
  
  // Position error (for revolute joints, this is simple subtraction)
  Eigen::VectorXd q_error = q_desired - q_current_;
  
  // Velocity error
  Eigen::VectorXd v_error = v_desired - v_current_;
  
  // Compute torque with diagonal gains
  tau_impedance = stiffness.cwiseProduct(q_error) + damping.cwiseProduct(v_error);
  
  return tau_impedance;
}

std::string EiriarmDynamics::getJointName(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return "";
  }
  return model_.names[joint_id];
}

int EiriarmDynamics::getJointIdxQ(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return -1;
  }
  return model_.joints[joint_id].idx_q();
}

int EiriarmDynamics::getJointIdxV(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return -1;
  }
  return model_.joints[joint_id].idx_v();
}

double EiriarmDynamics::getEffortLimit(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return 0.0;
  }
  
  int idx_v = model_.joints[joint_id].idx_v();
  if (idx_v >= 0 && idx_v < model_.effortLimit.size()) {
    return model_.effortLimit[idx_v];
  }
  
  return 0.0;
}

bool EiriarmDynamics::isActuated(int joint_id) const
{
  if (!initialized_ || joint_id < 1 || joint_id >= model_.njoints) {
    return false;
  }
  return model_.joints[joint_id].nv() > 0;
}

Eigen::VectorXd EiriarmDynamics::getNeutralConfiguration() const
{
  if (!initialized_) {
    return Eigen::VectorXd();
  }
  return pinocchio::neutral(model_);
}

}  // namespace eiriarm_dynamics
