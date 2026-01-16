#pragma once
#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include "mujoco/mujoco.h"
#include "glfw_adapter.h"
#include "simulate.h"
#include "array_safety.h"
#include <tabulate/tabulate.hpp>
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_srvs/srv/empty.hpp"
#include "yaml-cpp/yaml.h"
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <cmath>

#include <urdf/model.h>
#include <kdl_parser/kdl_parser.hpp>
#include <robot_state_publisher/robot_state_publisher.hpp>

/**
 * @brief 模型参数结构体
 * 
 */
struct modelParam{
    std::string modelName;
    float timeStep;
    std::vector<std::string> jointName,linkNames;
    std::vector<double> jointFri,jointDamp, linkMass;
    std::vector<std::pair<double,double>> jointPosRange,jointTorqueRange;
    std::vector<std::vector<std::string>> sensorType;
    std::vector<int> jntIdqpos, jntIdqvel, jntIddctl;
    int keyFrameCount = 0;
    size_t jointPosHeadID = 99999;
    size_t jointVelHeadID = 99999;
    size_t jointTorHeadID = 99999;
    std::vector<int> jointPosSensorIdx;
    std::vector<int> jointVelSensorIdx;
    std::vector<int> jointEffortSensorIdx;
    std::vector<int> jointToActuatorIdx;
    size_t imuQuatHeadID = 99999;
    size_t imuGyroHeadID = 99999;
    size_t imuAccHeadId = 99999;
    size_t realPosHeadID = 99999;
    size_t realVelHeadID = 99999;
    bool readErrorFlag = false;

};


using std::placeholders::_1;
using std::placeholders::_2;

/**
 * @brief mujoco与ROS2消息交互的类
 * 
 */
class SimulateBridge : public rclcpp::Node
{
private:
    // mujoco指针
    mjData* mj_data_ = nullptr;
    mjModel* mj_model_ = nullptr; 
    mujoco::Simulate& mj_sim_;
    // ROS2通信接口
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointCommandsSub_;
    rclcpp::Service<std_srvs::srv::Empty>::SharedPtr UnPauseServer_;
    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr jointStatePub_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> worldFramePub_;

    sensor_msgs::msg::JointState jointCommands_;
    // ROS2通信名称
    std::string jointCommandsTopic_;
    std::string UnPauseServiceService_;
    // 模型参数
    modelParam modelParam_;
    // 标志位
    bool initPauseFlag_ = false;
    bool modelTableFlag_ = true;
    // int cmdCount_ = 0;

    template <typename T> T ReadRosParam_(const std::string& param_name, const T& default_value);
    void JointCommandSubCallBack(const sensor_msgs::msg::JointState::SharedPtr jointCommand);
    void UnPauseServiceCallBack(const std::shared_ptr<std_srvs::srv::Empty::Request> request,
        std::shared_ptr<std_srvs::srv::Empty::Response> response);
    void ReadModel();
    void ShowModel();
    
public:
    RCLCPP_SMART_PTR_DEFINITIONS(SimulateBridge); // 用于生成智能指针 
    void JointStatePublish();

    SimulateBridge(mjData* d, mjModel* m, mujoco::Simulate& sim);
    ~SimulateBridge();

    sensor_msgs::msg::JointState GetJointCommands() {return jointCommands_;};
    int GetSimRun() {return mj_sim_.run;};
};

/**
 * @brief 统一读取ros参数的函数
 * 
 * @tparam T 读取参数的类型
 * @param param_name 参数的名称
 * @param default_value 参数默认值
 * @return T 读取到参数的值
 */
template <typename T>
T SimulateBridge::ReadRosParam_(const std::string& param_name, const T& default_value) 
{
    T value;
    
    // 声明参数并设置默认值
    this->declare_parameter<T>(param_name, default_value);
    
    // 尝试获取参数并进行参数检查
    if (!this->get_parameter(param_name, value)) {
        RCLCPP_ERROR(this->get_logger(), "参数 [%s] 未找到，使用默认值", param_name.c_str());
        return default_value;
    }
    
    return value;
}