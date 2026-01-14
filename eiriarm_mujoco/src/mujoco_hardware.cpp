#include "eiriarm_mujoco/mujoco_hardware.hpp"
#include "pluginlib/class_list_macros.hpp"
#include <filesystem>

namespace eiriarm_mujoco {

hardware_interface::CallbackReturn MujocoHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) !=
      hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 1. 获取 MJCF 模型路径 (需要在 ros2_control xacro 中配置)
  std::string model_path;
  if (info.hardware_parameters.count("model_path") > 0) {
    model_path = info.hardware_parameters.at("model_path");
  } else {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoHardware"), "Parameter 'model_path' not set");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // 2. 加载 MuJoCo 模型
  char error[1000] = "Could not load binary model";
  // 关键：处理文件路径，避免 chdir 影响 ROS
  // 建议在 URDF 中使用绝对路径，或者在这里处理路径逻辑
  if (!std::filesystem::exists(model_path)) {
      RCLCPP_ERROR(rclcpp::get_logger("MujocoHardware"), "File not found: %s", model_path.c_str());
      return hardware_interface::CallbackReturn::ERROR;
  }
  
  m_ = mj_loadXML(model_path.c_str(), 0, error, 1000);
  if (!m_) {
    RCLCPP_FATAL(rclcpp::get_logger("MujocoHardware"), "Load error: %s", error);
    return hardware_interface::CallbackReturn::ERROR;
  }
  d_ = mj_makeData(m_);

  // 3. 初始化 Joint 缓冲区
  size_t num_joints = info.joints.size();
  hw_positions_.resize(num_joints, 0.0);
  hw_velocities_.resize(num_joints, 0.0);
  hw_efforts_.resize(num_joints, 0.0);
  hw_commands_.resize(num_joints, 0.0);

  // 4. 建立映射 (ROS Joint Name -> MuJoCo ID)
  // 参考您的 Python 代码逻辑
  for (const auto & joint : info.joints) {
    int joint_id = mj_name2id(m_, mjOBJ_JOINT, joint.name.c_str());
    if (joint_id == -1) {
      RCLCPP_ERROR(rclcpp::get_logger("MujocoHardware"), "Joint '%s' not found in MJCF", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }
    joint_name_to_id_[joint.name] = joint_id;

    // 查找对应的 Actuator (假设 Actuator 名字和 Joint 名字有关联，或者通过 trnid 查找)
    // 简单起见，这里假设 actuator 驱动对应的 joint
    int actuator_id = -1;
    for(int i=0; i<m_->nu; ++i) {
        int trnid_joint = m_->actuator_trnid[i*2]; // 获取 actuator 对应的 joint id
        if (trnid_joint == joint_id) {
            actuator_id = i;
            break;
        }
    }
    if (actuator_id != -1) {
        joint_name_to_actuator_id_[joint.name] = actuator_id;
    } else {
         RCLCPP_WARN(rclcpp::get_logger("MujocoHardware"), "No actuator found for joint '%s'", joint.name.c_str());
    }
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Reset simulation if needed
  mj_resetData(m_, d_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  // Sync positions from simulation to ROS start state
  read(rclcpp::Time(0), rclcpp::Duration(0,0));
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn MujocoHardware::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type MujocoHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 从 MuJoCo 读取状态到 ROS
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    std::string name = info_.joints[i].name;
    int id = joint_name_to_id_[name];
    
    // 获取 qpos/qvel 的地址
    int qpos_adr = m_->jnt_qposadr[id];
    int qvel_adr = m_->jnt_dofadr[id];

    hw_positions_[i] = d_->qpos[qpos_adr];
    hw_velocities_[i] = d_->qvel[qvel_adr];
    // hw_efforts_ 可以从传感器读取，这里暂设为0
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type MujocoHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 1. 将 ROS 命令写入 MuJoCo
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    std::string name = info_.joints[i].name;
    if (joint_name_to_actuator_id_.count(name)) {
        int act_id = joint_name_to_actuator_id_[name];
        d_->ctrl[act_id] = hw_commands_[i];
    }
  }

  // 2. 物理步进 (这是替代您 Python 中 Timer 的部分)
  // 如果 ROS 频率与 MuJoCo timestep 不匹配，可以在这里循环多次 mj_step
  // 假设 ROS loop 足够快
  mj_step(m_, d_);

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> MujocoHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> MujocoHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < info_.joints.size(); ++i) {
    // 假设是 Effort 控制接口，如果是 Position 控制，需要添加 PID 逻辑（参考 openarm 的 write 函数）
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_commands_[i]));
  }
  return command_interfaces;
}

} // namespace eiriarm_mujoco

PLUGINLIB_EXPORT_CLASS(eiriarm_mujoco::MujocoHardware, hardware_interface::SystemInterface)