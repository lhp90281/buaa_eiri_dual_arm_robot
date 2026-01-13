#include <cstdio>
#include <cstring>
#include <chrono>
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <thread>
#include <mutex>
#include <filesystem>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"

#include <mujoco/mujoco.h>

using namespace std::chrono_literals;

// Globals for simple callback access
mjModel* m = nullptr;
mjData* d = nullptr;
std::map<std::string, int> joint_to_actuator_map;
std::map<std::string, int> actuator_map;

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("mujoco_bridge");

    // 1. Parameters
    node->declare_parameter<std::string>("model_file", "");
    std::string model_file_path;
    node->get_parameter("model_file", model_file_path);

    if (model_file_path.empty()) {
        RCLCPP_ERROR(node->get_logger(), "Parameter 'model_file' is empty.");
        return 1;
    }

    // 2. Load MuJoCo
    std::filesystem::path p(model_file_path);
    std::string model_dir = p.parent_path().string();
    std::string model_filename = p.filename().string();
    
    RCLCPP_INFO(node->get_logger(), "Changing dir to: %s", model_dir.c_str());
    if (chdir(model_dir.c_str()) != 0) {
        RCLCPP_ERROR(node->get_logger(), "Failed to change directory");
        return 1;
    }
    
    char error[1000] = "Could not load binary model";
    m = mj_loadXML(model_filename.c_str(), 0, error, 1000);
    if (!m) {
        RCLCPP_ERROR(node->get_logger(), "Load model error: %s", error);
        return 1;
    }
    d = mj_makeData(m);
    if (!d) {
        RCLCPP_ERROR(node->get_logger(), "Failed to create mjData");
        return 1;
    }
    RCLCPP_INFO(node->get_logger(), "MuJoCo loaded. nq=%d, nv=%d", m->nq, m->nv);

    // 3. Init Maps
    if (m->nu > 0) {
        for (int i = 0; i < m->nu; ++i) {
            const char* name = mj_id2name(m, mjOBJ_ACTUATOR, i);
            if (name) {
                actuator_map[std::string(name)] = i;
                int joint_id = m->actuator_trnid[i * 2];
                if (joint_id >= 0 && joint_id < m->njnt) {
                    const char* joint_name = mj_id2name(m, mjOBJ_JOINT, joint_id);
                    if (joint_name) joint_to_actuator_map[std::string(joint_name)] = i;
                }
            }
        }
    }
    
    std::vector<std::string> joint_names;
    std::vector<int> joint_qpos_ids;
    std::vector<int> joint_qvel_ids;
    if (m->njnt > 0) {
        for (int i = 0; i < m->njnt; ++i) {
            const char* name = mj_id2name(m, mjOBJ_JOINT, i);
            if (name) {
                joint_names.push_back(name);
                joint_qpos_ids.push_back(m->jnt_qposadr[i]);
                joint_qvel_ids.push_back(m->jnt_dofadr[i]);
            }
        }
    }

    // 4. ROS Interfaces
    auto pub = node->create_publisher<sensor_msgs::msg::JointState>("/joint_states", 10);
    
    auto sub = node->create_subscription<sensor_msgs::msg::JointState>(
        "/ctrl/effort", 10,
        [&](const sensor_msgs::msg::JointState::SharedPtr msg) {
            // Simple logic: direct write to d->ctrl. 
            // Since this runs in spin_some() on the main thread, it is SEQUENTIAL with mj_step.
            // No mutex needed!
            if (msg->name.size() != msg->effort.size()) return;
            for (size_t i = 0; i < msg->name.size(); ++i) {
                const auto& name = msg->name[i];
                double effort = msg->effort[i];
                if (joint_to_actuator_map.count(name)) {
                    int id = joint_to_actuator_map[name];
                    if(id < m->nu) d->ctrl[id] = effort;
                }
                else if (actuator_map.count(name)) {
                    int id = actuator_map[name];
                    if(id < m->nu) d->ctrl[id] = effort;
                }
            }
        });

    RCLCPP_INFO(node->get_logger(), "Starting main loop...");

    // 5. Main Loop
    rclcpp::Rate rate(100);
    while (rclcpp::ok()) {
        // Step Physics
        mj_step(m, d);

        // Publish State
        auto msg = sensor_msgs::msg::JointState();
        msg.header.stamp = node->now();
        
        for (size_t i = 0; i < joint_names.size(); ++i) {
            msg.name.push_back(joint_names[i]);
            int qpos_idx = joint_qpos_ids[i];
            int qvel_idx = joint_qvel_ids[i];
            
            if (qpos_idx >= 0 && qpos_idx < m->nq) msg.position.push_back(d->qpos[qpos_idx]);
            else msg.position.push_back(0.0);
            
            if (qvel_idx >= 0 && qvel_idx < m->nv) msg.velocity.push_back(d->qvel[qvel_idx]);
            else msg.velocity.push_back(0.0);
            
            msg.effort.push_back(0.0);
        }
        pub->publish(msg);

        // Handle ROS callbacks
        rclcpp::spin_some(node);
        rate.sleep();
    }

    // Cleanup
    if (d) mj_deleteData(d);
    if (m) mj_deleteModel(m);
    rclcpp::shutdown();
    return 0;
}
