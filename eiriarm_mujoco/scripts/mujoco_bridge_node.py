#!/usr/bin/env python3
import os
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from ament_index_python.packages import get_package_share_directory
import mujoco
import mujoco.viewer
import numpy as np

class MujocoBridgeNode(Node):
    def __init__(self):
        super().__init__('mujoco_bridge_node')
        print("\n\n >>>>>> MUJOCO BRIDGE NODE STARTING (WITH VIEWER) <<<<<< \n\n")
        
        # 1. 参数声明
        self.declare_parameter('model_file', '')
        # 注意：use_sim_time 通常由 Launch 系统自动管理，无需手动 declare，否则会报错 ParameterAlreadyDeclaredException
        # self.declare_parameter('use_sim_time', False)
        
        # 2. 获取模型路径
        model_path = self.get_parameter('model_file').get_parameter_value().string_value
        if not model_path:
            self.get_logger().error("Parameter 'model_file' is empty!")
            raise ValueError("model_file parameter is required")

        self.get_logger().info(f"Loading MuJoCo model from: {model_path}")

        # 3. 加载 MuJoCo 模型
        try:
            # 关键：MuJoCo 加载 xml 时需要正确的工作目录以寻找 mesh
            model_dir = os.path.dirname(model_path)
            model_filename = os.path.basename(model_path)
            
            # 记录当前目录以便稍后恢复（虽然 Node 退出也无所谓）
            cwd = os.getcwd()
            os.chdir(model_dir)
            self.get_logger().info(f"Switched working directory to: {model_dir}")
            
            self.model = mujoco.MjModel.from_xml_path(model_path)
            self.data = mujoco.MjData(self.model)
            
            # 恢复目录
            os.chdir(cwd)
            self.get_logger().info("MuJoCo Model loaded successfully.")
            
        except Exception as e:
            self.get_logger().error(f"Failed to load MuJoCo model: {e}")
            raise e
        
        # 4. 启动 Viewer (Passive)
        self.get_logger().info("Attempting to launch passive viewer...")
        try:
            self.viewer = mujoco.viewer.launch_passive(self.model, self.data)
            self.get_logger().info("MuJoCo Viewer launched successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to launch viewer: {e}")
            self.viewer = None

        # 5. 初始化映射 (Joint Name -> ID, Actuator Name -> ID)
        self._init_maps()

        # 6. 创建发布者和订阅者
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)
        self.cmd_sub = self.create_subscription(JointState, '/ctrl/effort', self.cmd_callback, 10)

        # 7. 定时器 (300Hz)
        self.timer_period = 1.0 / 400.0
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # 渲染控制：限制 Viewer 刷新率以避免卡顿
        self.last_render_time = time.time()
        self.render_fps = 30.0 # 目标渲染帧率
        
        # 仿真步长
        self.dt = self.model.opt.timestep
        self.get_logger().info(f"Simulation timestep: {self.dt}, Timer period: {self.timer_period}")

    def __del__(self):
        if hasattr(self, 'viewer') and self.viewer:
            self.viewer.close()

    def _init_maps(self):
        """建立名称到索引的映射"""
        self.joint_names = []
        self.joint_qpos_ids = []
        self.joint_qvel_ids = []
        
        self.joint_to_actuator = {} # joint_name -> actuator_id
        self.actuator_map = {}      # actuator_name -> actuator_id

        # 1. 遍历所有关节
        # MuJoCo 的 Joint ID 是从 0 到 model.njnt-1
        for i in range(self.model.njnt):
            # 获取关节名称
            name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name:
                self.joint_names.append(name)
                # qpos 和 qvel 的起始地址
                self.joint_qpos_ids.append(self.model.jnt_qposadr[i])
                self.joint_qvel_ids.append(self.model.jnt_dofadr[i])
        
        self.get_logger().info(f"Found {len(self.joint_names)} joints: {self.joint_names}")

        # 2. 遍历所有 Actuator
        for i in range(self.model.nu):
            act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if act_name:
                self.actuator_map[act_name] = i
                
            # 尝试找到该 Actuator 控制的 Joint
            # trnid[i, 0] 通常是 joint id
            joint_id = self.model.actuator_trnid[i, 0]
            if 0 <= joint_id < self.model.njnt:
                joint_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, joint_id)
                if joint_name:
                    self.joint_to_actuator[joint_name] = i
                    
        self.get_logger().info(f"Mapped {len(self.actuator_map)} actuators.")

    def cmd_callback(self, msg):
        """处理控制指令"""
        if len(msg.name) != len(msg.effort):
            return
            
        for name, effort in zip(msg.name, msg.effort):
            # 优先通过 Joint 名字查找对应的 Actuator
            if name in self.joint_to_actuator:
                act_id = self.joint_to_actuator[name]
                self.data.ctrl[act_id] = effort
            # 也可以直接通过 Actuator 名字查找
            elif name in self.actuator_map:
                act_id = self.actuator_map[name]
                self.data.ctrl[act_id] = effort

    def timer_callback(self):
        """物理仿真步进 + 状态发布"""
        # 1. Step Physics
        # 可以选择步进多次以匹配实时性，这里简单起见每次 timer 步进一次
        # 如果需要严格实时，可以计算 wall time diff
        mujoco.mj_step(self.model, self.data)
        
        # 2. Update Viewer (With FPS Limit)
        if self.viewer and self.viewer.is_running():
            current_time = time.time()
            if current_time - self.last_render_time >= (1.0 / self.render_fps):
                self.viewer.sync()
                self.last_render_time = current_time
        else:
            # 如果 viewer 被用户关闭了，也许我们应该关闭节点？或者继续跑 headless?
            # 这里选择继续跑
            pass

        # 3. Publish Joint States
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        msg.name = self.joint_names
        msg.position = []
        msg.velocity = []
        msg.effort = [] # 如果有力传感器可以填

        for i in range(len(self.joint_names)):
            qpos_idx = self.joint_qpos_ids[i]
            qvel_idx = self.joint_qvel_ids[i]
            
            # 注意：对于 free joint (floating base)，qpos 有 7 个值 (xyz + quat)，qvel 有 6 个
            # 这里简化处理，假设都是 1-DOF 关节 (Hinge/Slide)
            # 如果你的机械臂基座是固定的，这没问题。
            
            msg.position.append(self.data.qpos[qpos_idx])
            msg.velocity.append(self.data.qvel[qvel_idx])
            msg.effort.append(0.0)

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MujocoBridgeNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
