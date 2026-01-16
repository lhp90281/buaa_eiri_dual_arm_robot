#include "eiriarm_mujoco/simulate_bridge.hpp"

/**
 * @brief 仿真器类的构造函数
 * 
 * @param d mujoco模型数据指针
 * @param m mujoco模型指针
 * @param sim 仿真UI类的引用
 */
SimulateBridge::SimulateBridge(mjData* d, mjModel* m, mujoco::Simulate& sim) : Node("mujoco_simulator_node"),mj_sim_(sim)
{
    // FIXME:解决穿模问题
    // 将模型数据指针复制进来
    mj_data_ = d;
    mj_model_ = m;

    // 修改仿真参数
    mj_model_->opt.timestep = 0.001; // 修改仿真频率为1KHz

    // 从yaml中读取参数
    const std::string pkg_path = ament_index_cpp::get_package_share_directory("eiriarm_mujoco"); // 获取包路径
    const std::string yaml_path = pkg_path + "/config/simulate.yaml"; // 拼接yaml路径
    YAML::Node config = YAML::LoadFile(yaml_path)["mujoco_simulator"];
    jointCommandsTopic_ = config["jointCommandsTopic"].as<std::string>();
    UnPauseServiceService_ = config["unPauseService"].as<std::string>();
    initPauseFlag_ = config["initPauseFlag"].as<bool>();
    modelTableFlag_ = config["modelTableFlag"].as<bool>();
    
    // 如有设置,则暂停仿真
    if(initPauseFlag_) mj_sim_.run = 0;

    // 读取模型内容参数
    ReadModel();
    // 输出相关的ID
    ShowModel();

    // 创建话题通信接口
    jointCommandsSub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        jointCommandsTopic_,
        1,
        std::bind(&SimulateBridge::JointCommandSubCallBack, this, _1)
    );
    jointStatePub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        "/joint_states",
        1
    );
    worldFramePub_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    // 初始化服务通信接口
    UnPauseServer_ = this->create_service<std_srvs::srv::Empty>(
        UnPauseServiceService_,
        std::bind(&SimulateBridge::UnPauseServiceCallBack, this, _1, _2)
    );

    // 初始化通信vector长度
    jointCommands_.name.resize(mj_model_->nu);
    jointCommands_.position.resize(mj_model_->nu, 0.0);
    jointCommands_.velocity.resize(mj_model_->nu, 0.0);
    jointCommands_.effort.resize(mj_model_->nu, 0.0);
}

/**
 * @brief 电机命令接收回调
 * 
 * @param jointCommand 接收到的电机命令
 */
void SimulateBridge::JointCommandSubCallBack(const sensor_msgs::msg::JointState::SharedPtr jointCommand){

    // 如果模型不符合需求,则不执行操作
    if(modelParam_.readErrorFlag) return;
    // 收到的命令长度与模型关节数不匹配时进行处理
    // if(jointCommand->effort.size() > (size_t)mj_model_->nu){
    //     RCLCPP_ERROR(this->get_logger(), "命令长度大于模型关节数,请检查");
    //     return; 
    // }
    // // 将命令值保存到成员变量
    // jointCommands_ = *jointCommand;
    
    // 建立临时名字到ID映射，如果还没有的话 (或者可以存成成员变量优化)
    static std::map<std::string, int> jointNameToIdx;
    if(jointNameToIdx.empty()) {
        for(size_t i=0; i<modelParam_.jointName.size(); i++) {
            jointNameToIdx[modelParam_.jointName[i]] = i;
        }
    }

    // 遍历收到的命令
    for(size_t i=0; i<jointCommand->name.size(); i++) {
        std::string name = jointCommand->name[i];
        if(jointNameToIdx.find(name) == jointNameToIdx.end()) continue; // 未知关节
        
        int jointIdx = jointNameToIdx[name];
        int actuatorIdx = modelParam_.jointToActuatorIdx[jointIdx];
        
        if(actuatorIdx >= 0 && actuatorIdx < mj_model_->nu) {
            // 确保有对应的effort值
            if(i < jointCommand->effort.size()) {
                jointCommands_.effort[actuatorIdx] = jointCommand->effort[i];
            }
        }
    }
}

/**
 * @brief 继续mujoco物理仿真的服务回调
 * 
 * @param request 
 * @param response 
 */
void SimulateBridge::UnPauseServiceCallBack(
    const std::shared_ptr<std_srvs::srv::Empty::Request> request,
    std::shared_ptr<std_srvs::srv::Empty::Response> response
) {
    // 避免未使用变量警告
    (void)request; 
    (void)response;
    // 如果初始暂停了物理仿真,则继续
    if(initPauseFlag_ && mj_sim_.run == 0) mj_sim_.run = 1;

    // 应用第一个关键帧作为初始位置
    if(modelParam_.keyFrameCount > 0 ){ 
        mj_resetDataKeyframe(mj_model_, mj_data_, 0);
    }
    
}

void SimulateBridge::JointStatePublish() {
    // 如果模型不符合需求,则不执行操作
    if(modelParam_.readErrorFlag) return;

    // 发布关节信息
    sensor_msgs::msg::JointState jointState;
    jointState.header.stamp = this->get_clock()->now();
    size_t num_joints = modelParam_.jointName.size();
    jointState.name.resize(num_joints);
    jointState.position.resize(num_joints);
    jointState.velocity.resize(num_joints);
    jointState.effort.resize(num_joints);

    for (size_t i = 0; i < num_joints; i++) {
        jointState.name[i] = modelParam_.jointName[i];
        
        // 使用映射表查找传感器索引
        int pos_idx = modelParam_.jointPosSensorIdx[i];
        int vel_idx = modelParam_.jointVelSensorIdx[i];
        int tor_idx = modelParam_.jointEffortSensorIdx[i];

        if (pos_idx >= 0 && pos_idx < mj_model_->nsensor)
            jointState.position[i] = mj_data_->sensordata[pos_idx];
        else
            jointState.position[i] = 0;

        if (vel_idx >= 0 && vel_idx < mj_model_->nsensor)
            jointState.velocity[i] = mj_data_->sensordata[vel_idx];
        else
            jointState.velocity[i] = 0;

        if (tor_idx >= 0 && tor_idx < mj_model_->nsensor)
            jointState.effort[i]   = mj_data_->sensordata[tor_idx];
        else 
            jointState.effort[i]   = 0;
    }
    jointStatePub_->publish(jointState);

    // 发布世界坐标信息
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->get_clock()->now();
    transform.header.frame_id = "world";
    transform.child_frame_id = "base_link";
    if(modelParam_.realPosHeadID != 99999) {
        transform.transform.translation.x = mj_data_->sensordata[modelParam_.realPosHeadID+0];
        transform.transform.translation.y = mj_data_->sensordata[modelParam_.realPosHeadID+1];
        transform.transform.translation.z = mj_data_->sensordata[modelParam_.realPosHeadID+2];
    }
    if(modelParam_.imuQuatHeadID != 99999) {
        transform.transform.rotation.w = mj_data_->sensordata[modelParam_.imuQuatHeadID + 0];
        transform.transform.rotation.x = mj_data_->sensordata[modelParam_.imuQuatHeadID + 1];
        transform.transform.rotation.y = mj_data_->sensordata[modelParam_.imuQuatHeadID + 2];
        transform.transform.rotation.z = mj_data_->sensordata[modelParam_.imuQuatHeadID + 3];
    }
    worldFramePub_->sendTransform(transform);
}

/**
 * @brief 输出读取到的MJCF文件信息
 * 
 */
void SimulateBridge::ShowModel(){

    using namespace tabulate;

    // 总表
    Table modelInfo;
    // 子项
    Table allJointsInfo;
    Table allLinksInfo;
    Table allSensorsInfo;
    // 子项中的表
    Table jointInfo;
    Table linkInfo;
    Table sensorInfo;

    // 总表头
    modelInfo.add_row({"model information"});
    modelInfo.add_row(RowStream{} << "model name: " + modelParam_.modelName
                                  + " time step: " + std::to_string(modelParam_.timeStep) + "s");
    modelInfo.format().font_align(FontAlign::center);
    modelInfo[1].format().hide_border_top().hide_border_bottom().font_align(FontAlign::center);


    // 输出joint信息
    allJointsInfo.add_row({"joints information"});
    allJointsInfo.format().font_align(FontAlign::center);
    allJointsInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    jointInfo.add_row({"ID", "name", "posLimit(rad)", "torLimit(Nm)", "friction", "damping"});
    jointInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.jointName.size(); i++){
        // 组织限幅字符串
        std::ostringstream posLimitStr;
        std::ostringstream torLimitStr;
        posLimitStr << std::fixed << std::setprecision(2) << std::fixed << modelParam_.jointPosRange[i].first << " ~ " << modelParam_.jointPosRange[i].second;
        torLimitStr << std::fixed << std::setprecision(2) << std::fixed << modelParam_.jointTorqueRange[i].first << " ~ " << modelParam_.jointTorqueRange[i].second;
        jointInfo.add_row(RowStream{} 
            << std::fixed << std::setprecision(2) << i << modelParam_.jointName[i] << posLimitStr.str() << torLimitStr.str() 
            << modelParam_.jointFri[i] << modelParam_.jointDamp[i]);
            jointInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) jointInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allJointsInfo.add_row({jointInfo});
    allJointsInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 输出link信息
    allLinksInfo.add_row({"links information"});
    allLinksInfo.format().font_align(FontAlign::center);
    allLinksInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    linkInfo.add_row({"ID", "name", "mass(kg)"});
    linkInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.linkNames.size(); i++){
        linkInfo.add_row(RowStream{} 
            << std::fixed << std::setprecision(2) << i << modelParam_.linkNames[i] << modelParam_.linkMass[i]);
        linkInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) linkInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allLinksInfo.add_row({linkInfo});
    allLinksInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 输出sensor信息
    allSensorsInfo.add_row({"sensors information"});
    allSensorsInfo.format().font_align(FontAlign::center);
    allSensorsInfo[0].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();
    sensorInfo.add_row({"ID", "name", "type", "attach", "head"});
    sensorInfo[0].format().font_align(FontAlign::center).font_color(Color::yellow).hide_border_bottom();
    for(size_t i = 0; i<modelParam_.sensorType.size(); i++){
        std::string headIDName;
        if(i == modelParam_.jointPosHeadID) headIDName = "joint pos head";
        else if(i == modelParam_.jointVelHeadID) headIDName = "joint vel head";
        else if(i == modelParam_.jointTorHeadID) headIDName = "joint torque head";
        else if(i == modelParam_.imuQuatHeadID) headIDName = "imu quat head";
        else if(i == modelParam_.imuGyroHeadID) headIDName = "imu gyro head";
        else if(i == modelParam_.imuAccHeadId) headIDName = "imu acc head";
        else if(i == modelParam_.realPosHeadID) headIDName = "real pos head";
        else if(i == modelParam_.realVelHeadID) headIDName = "real vel head";
        else headIDName = "";
        sensorInfo.add_row(RowStream{} 
            << i << modelParam_.sensorType[i][0] << modelParam_.sensorType[i][1] << modelParam_.sensorType[i][2] << headIDName);
        sensorInfo[i].format().font_align(FontAlign::center);
        if(i >= 1) sensorInfo[i+1].format().hide_border_top().font_align(FontAlign::center);
    }
    allSensorsInfo.add_row({sensorInfo});
    allSensorsInfo[1].format().hide_border_top().hide_border_bottom().hide_border_left().hide_border_right();

    // 将子表加入总表
    modelInfo.add_row({allJointsInfo});
    modelInfo.add_row({allLinksInfo});
    modelInfo.add_row({allSensorsInfo});

    // 输出模型信息
    if(modelTableFlag_){
        std::cout << "------------读取到的环境与模型信息如下------------" << std::endl;
        std::cout << modelInfo << std::endl;
        std::cout << "如果仿真遇到问题,请检查上述信息是否正确,物理仿真进行中..." << std::endl;
    }


    if(modelParam_.keyFrameCount == 0 ){ 
        RCLCPP_WARN(this->get_logger(), "未发现keyframe,请检查模型");
    }

    // 传感器错误检查
    if(modelParam_.jointPosHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节位置传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.jointVelHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.jointTorHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现关节力矩传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuQuatHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现四元数传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuGyroHeadID==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现角速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.imuAccHeadId==99999){
        RCLCPP_ERROR(this->get_logger(), "未发现线加速度传感器,请检查模型");
        modelParam_.readErrorFlag = true;
    }
    if(modelParam_.readErrorFlag){
        RCLCPP_ERROR(this->get_logger(), "传感器参数缺失,将不会进行ROS通信");
    }


}

/**
 * @brief 读取加载的模型信息并保存到类成员中
 * 
 */
void SimulateBridge::ReadModel(){

    // 读取加载的模型名称
    modelParam_.modelName = mj_model_->names;
    // 读取加载的模型时间步长
    modelParam_.timeStep = mj_model_->opt.timestep;
    // 加载关键帧个数
    modelParam_.keyFrameCount = mj_model_->nkey;
    if(modelParam_.keyFrameCount > 0 ){ // 应用第一个关键帧作为初始位置
        mj_resetDataKeyframe(mj_model_, mj_data_, 0);
    }

    // 遍历所有joint,读取参数
    for(int i = 0; i<mj_model_->njnt; i++){
        if(mj_model_->jnt_type[i] == mjJNT_FREE) continue;// 注意:这里删去了free joint
        modelParam_.jointName.push_back(mj_id2name(mj_model_,mjOBJ_JOINT,i)); // 关节名称
        modelParam_.jointPosRange.push_back(std::make_pair(mj_model_->jnt_range[2*i],mj_model_->jnt_range[2*i+1])); // 关节位置限幅
        modelParam_.jointTorqueRange.push_back(std::make_pair(mj_model_->jnt_actfrcrange[2*i],mj_model_->jnt_actfrcrange[2*i+1])); // 关节速度限幅
        int joint_dofadr = mj_model_->jnt_dofadr[i];
        modelParam_.jointFri.push_back(mj_model_->dof_frictionloss[joint_dofadr]); // 关节摩擦系数
        modelParam_.jointDamp.push_back(mj_model_->dof_damping[joint_dofadr]); // 关节阻尼
    }

    // 遍历所有link,读取参数
    for(int i = 0; i<mj_model_->nbody; i++){
        if(std::string(mj_id2name(mj_model_,mjOBJ_BODY,i)) == "world")  continue;// 忽略world link
        modelParam_.linkNames.push_back(mj_id2name(mj_model_,mjOBJ_BODY,i));
        modelParam_.linkMass.push_back(mj_model_->body_mass[i]);
    }

    // 遍历所有sensor,读取参数并建立映射
    modelParam_.jointPosSensorIdx.assign(mj_model_->njnt, -1);
    modelParam_.jointVelSensorIdx.assign(mj_model_->njnt, -1);
    modelParam_.jointEffortSensorIdx.assign(mj_model_->njnt, -1);
    modelParam_.jointToActuatorIdx.assign(mj_model_->njnt, -1);

    // 建立执行器映射
    for(int i=0; i<mj_model_->nu; i++) {
        // 假设 actuator_trntype 是 mjTRN_JOINT
        int joint_id = mj_model_->actuator_trnid[2*i];
        if(joint_id >= 0 && joint_id < mj_model_->njnt) {
             modelParam_.jointToActuatorIdx[joint_id] = i;
        }
    }

    // 首先建立关节名到索引的临时映射，加速查找
    std::map<std::string, int> jointNameToIdx;
    for(size_t i=0; i<modelParam_.jointName.size(); i++) {
        jointNameToIdx[modelParam_.jointName[i]] = i;
    }

    for(size_t i = 0; i < static_cast<size_t>(mj_model_ -> nsensor); i++){
        // 创建临时变量
        std::string tempName;
        std::string tempType;
        std::string tempAttch;
        // 获取sensor名称
        tempName = mj_id2name(mj_model_, mjOBJ_SENSOR, i);
        // 根据不同类型的sensor进行不同的处理
        if(mj_model_->sensor_type[i] == mjSENS_JOINTPOS){ // 关节位置
            tempType = "joint pos";
            if(modelParam_.jointPosHeadID == 99999) modelParam_.jointPosHeadID = modelParam_.sensorType.size();
            // 记录索引
            tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, mj_model_->sensor_objid[i]);
            if(jointNameToIdx.find(tempAttch) != jointNameToIdx.end()){
                modelParam_.jointPosSensorIdx[jointNameToIdx[tempAttch]] = i;
            }
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_JOINTVEL) { // 关节速度
            tempType = "joint vel";
            if(modelParam_.jointVelHeadID == 99999) modelParam_.jointVelHeadID = modelParam_.sensorType.size();
            // 记录索引
            tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, mj_model_->sensor_objid[i]);
            if(jointNameToIdx.find(tempAttch) != jointNameToIdx.end()){
                modelParam_.jointVelSensorIdx[jointNameToIdx[tempAttch]] = i;
            }
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_ACTUATORFRC) { // 关节力矩 (actuator force)
            tempType = "joint torque";
            if(modelParam_.jointTorHeadID == 99999) modelParam_.jointTorHeadID = modelParam_.sensorType.size();
            
            // For actuator force sensors, the object ID refers to the actuator.
            // We need to find which joint this actuator drives.
            int actuator_id = mj_model_->sensor_objid[i];
            int joint_id = mj_model_->actuator_trnid[2*actuator_id]; // Assuming joint joint transmission
            
            // If trntype is mjTRN_JOINT (0) or mjTRN_JOINTINPARENT (1), trnid[0] is the joint id
            if (mj_model_->actuator_trntype[actuator_id] == mjTRN_JOINT) {
                 tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, joint_id);
                 if(jointNameToIdx.find(tempAttch) != jointNameToIdx.end()){
                    modelParam_.jointEffortSensorIdx[jointNameToIdx[tempAttch]] = i;
                 }
            } else {
                tempAttch = mj_id2name(mj_model_, mjOBJ_ACTUATOR, actuator_id); // Fallback
            }
            
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName,tempType,tempAttch});
            continue;
        }
            
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMEQUAT) { // imu四元数
            tempType = "imu quat";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuQuatHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_w",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_GYRO) { // imu角速度
            tempType = "imu gyro";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuGyroHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_ACCELEROMETER) { // imu线加速度
            tempType = "imu linear acc";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.imuAccHeadId = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMEPOS) { // 实际位置
            tempType = "real position";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.realPosHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else if(mj_model_->sensor_type[i] == mjSENS_FRAMELINVEL) { // 实际速度
            tempType = "real velocity";
            tempAttch = mj_id2name(mj_model_, mjOBJ_BODY, mj_model_->sensor_objid[i]+1);
            modelParam_.realVelHeadID = modelParam_.sensorType.size();
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_x",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_y",tempType,tempAttch});
            modelParam_.sensorType.push_back(std::vector<std::string>{tempName+"_z",tempType,tempAttch});
            continue;
        }
        else tempType = "unknown";

        tempAttch = mj_id2name(mj_model_, mjOBJ_JOINT, mj_model_->sensor_objid[i]);
        modelParam_.sensorType.push_back(std::vector<std::string>{tempName,tempType,tempAttch});
    }

}

/**
 * @brief 析构函数,释放消息发布资源
 * 
 */
SimulateBridge::~SimulateBridge()
{
}
