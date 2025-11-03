Writer: Liu Yongxue
email: 805110687@qq.com
#!/usr/bin/env python3
"""
swarm_single_file.py

Single-file PX4 swarm controller (ROS2, Python).
- Reads config 'swarm.yaml' from same directory.
- Creates N agent nodes (agents 1..N).
- Each agent subscribes to /px4_j/fmu/out/vehicle_odometry for neighbors,
  and if configured (b[i]==1) subscribes to /fmu/out/vehicle_odometry (virtual leader).
- Computes distributed consensus formation control and publishes TrajectorySetpoint
  to /px4_i/fmu/in/trajectory_setpoint.

Run:
    python3 swarm_single_file.py

单文件 PX4 集群控制器（ROS2，Python）
- 从相同目录读取配置文件 'swarm.yaml'
- 创建 N 个智能体节点（智能体 1..N）
- 每个智能体订阅 /px4_j/fmu/out/vehicle_odometry 获取邻居信息，
  如果配置了 (b[i]==1) 则订阅 /fmu/out/vehicle_odometry（虚拟领航者）
- 计算分布式一致性编队控制，并发布 TrajectorySetpoint 到 /px4_i/fmu/in/trajectory_setpoint

运行：
    python3 swarm_single_file.py
"""
import os
import sys
import yaml
import math
import time
import threading
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

# px4_msgs messages
from px4_msgs.msg import VehicleOdometry, OffboardControlMode
from px4_msgs.msg import TrajectorySetpoint
from px4_msgs.msg import VehicleCommand

# ---------------------------
# Helper: Consensus Controller
# 辅助类：一致性控制器
# ---------------------------
class ConsensusController:
    def __init__(self, A, b, formation=None):
        """
        A: adjacency matrix (list of lists) size NxN for agents indexed 1..N
        b: pinning vector length N, b[i]==1 means agent i+1 can hear leader
        formation: optional list of N [x,y] offsets (d_i)
        
        A: 邻接矩阵（列表的列表）大小为 NxN，对应索引为 1..N 的智能体
        b: 牵引向量，长度为 N，b[i]==1 表示智能体 i+1 能接收到领航者信息
        formation: 可选的 N 个 [x,y] 偏移量列表 (d_i)
        """
        self.A = np.array(A, dtype=float)  # 邻接矩阵
        self.b = np.array(b, dtype=float)   # 牵引向量
        self.n = self.A.shape[0]  # number of agents 智能体数量
        
        # 线程锁，用于保护打印输出（避免多线程打印混乱）
        self._print_lock = threading.Lock()
        
        # 设置编队构型
        if formation is not None:
            self.formation = np.array(formation, dtype=float)
            if self.formation.shape[0] != self.n:
                raise ValueError("formation length != n")
        else:
            # default formation: line along x with spacing 1.0
            # 默认编队：沿x轴排列，间距0.8
            self.formation = np.zeros((self.n, 2))
            for i in range(self.n):
                self.formation[i] = np.array([i * 0.8, 0.0])

        # pinning gain g_i: use b[i] as binary, map to gain value
        # if b[i]==1 -> g_i = g_pin else 0
        # 牵引增益 g_i：使用 b[i] 作为二进制标志，映射到增益值
        # 如果 b[i]==1 -> g_i = g_pin 否则为 0
        self.g_pin = 0.10      # 牵引增益系数
        self.gains = self.b * self.g_pin  # 计算每个智能体的实际增益

        # control gain on consensus term
        # 一致性项的控制增益
        self.k_consensus = 0.20

    def compute_control(self, idx, positions, leader_pos, leader_vel, have_leader=False):
        """
        idx: agent index in [0..n-1] corresponding to agent IDs 1..N
        positions: Nx2 array of current agent positions (may contain zeros initially)
        leader_pos, leader_vel: 2-vectors (if have_leader True)
        have_leader: whether this agent currently has leader data
        returns: u (2-vector) -- desired horizontal velocity vector
        
        idx: 智能体索引 [0..n-1]，对应智能体ID 1..N
        positions: Nx2数组，当前智能体位置（初始可能包含零）
        leader_pos, leader_vel: 二维向量（如果have_leader为True）
        have_leader: 该智能体当前是否有领航者数据
        返回: u (2维向量) -- 期望的水平速度向量
        """
        p_i = positions[idx].astype(float)  # 当前智能体位置
        d_i = self.formation[idx].astype(float)  # 当前智能体在编队中的期望偏移

        sum_term = np.zeros(2, dtype=float)
        # consensus sum over neighbors j
        # 对邻居j的一致性求和
        for j in range(self.n):
            aij = self.A[idx, j]  # 邻接矩阵元素
            if aij != 0:
                p_j = positions[j].astype(float)  # 邻居位置
                d_j = self.formation[j].astype(float)  # 邻居期望偏移
                sum_term += aij * (p_i - p_j)  # 一致性项

        # leader parts
        # 领航者部分
        if have_leader:
            p0 = np.array(leader_pos, dtype=float)  # 领航者位置
            v0 = np.array(leader_vel, dtype=float)  # 领航者速度
        else:
            # if no leader info, use zero leader vel and a virtual p0 at origin (or previous)
            # 如果没有领航者信息，使用零速度和原点处的虚拟位置
            p0 = np.zeros(2, dtype=float)
            v0 = np.zeros(2, dtype=float)

        g_i = float(self.gains[idx])  # 当前智能体的牵引增益
        # 控制律计算：u = v0 - k_consensus * 一致性项 - g_i * 位置误差
        u = v0 - self.k_consensus * sum_term - g_i * ((p_i - p0))
        
        # 打印调试信息
        with self._print_lock:
            print(f"\n[DEBUG] Agent {idx+1} control computation:")
            print(f"  p_i = {p_i}")
            print(f"  d_i = {d_i}")
            print(f"  Leader pos = {leader_pos}, vel = {leader_vel}, have_leader = {have_leader}")
            print(f"  sum_term = {sum_term}")
            print(f"  g_i = {g_i}")
            print(f"  u (desired vel) = {u}\n")
        return u

# ---------------------------
# PX4 Agent Node (per agent)
# PX4 智能体节点（每个智能体一个）
# ---------------------------
class PX4Agent(Node):
    def __init__(self, agent_index, cfg, controller):
        """
        agent_index: 0..N-1 corresponds to physical px4 instance /px4_{id+1}/...
        cfg: configuration dict
        controller: ConsensusController instance
        
        agent_index: 0..N-1 对应物理PX4实例 /px4_{id+1}/...
        cfg: 配置字典
        controller: ConsensusController实例
        """
        node_name = f'px4_agent_{agent_index+1}'  # 节点名称
        super().__init__(node_name)
        self.agent_index = agent_index  # 智能体索引
        self.controller = controller    # 一致性控制器
        self.cfg = cfg                  # 配置
        self.n = controller.n           # 智能体数量

        # state arrays for neighbors (agents 1..N)
        # 邻居状态数组（智能体 1..N）
        self.positions = np.zeros((self.n, 2), dtype=float)  # 智能体位置
        self.velocities = np.zeros((self.n, 2), dtype=float)  # 智能体速度
        self.have_pos = [False] * self.n  # 位置数据有效性标志

        # leader info (virtual leader p0)
        # 领航者信息（虚拟领航者 p0）
        self.can_hear_leader = bool(cfg.get('b', [0]*self.n)[agent_index])  # 是否能听到领航者
        self.leader_pos = np.zeros(2, dtype=float)  # 领航者位置
        self.leader_vel = np.zeros(2, dtype=float)  # 领航者速度
        self.have_leader = False  # 领航者数据有效性标志
        self.odom_cb_is_working = False  # 里程计回调工作标志
        self.leader_odom_cb_is_working = False  # 领航者里程计回调工作标志

        # logging throttle
        # 日志节流控制
        self._last_log = {}

        # QoS for PX4 - BEST_EFFORT + VOLATILE
        # PX4的QoS配置 - 尽力而为 + 易失性
        self.qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # 可靠性：尽力而为
            durability=DurabilityPolicy.VOLATILE,       # 持久性：易失的
            history=HistoryPolicy.KEEP_LAST,            # 历史策略：保持最后
            depth=10                                    # 队列深度
        )

        # Configure QoS profile for publishing and subscribing
        # 配置发布和订阅的QoS配置文件
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,   # 可靠性：尽力而为
            durability=DurabilityPolicy.TRANSIENT_LOCAL, # 持久性：瞬态本地
            history=HistoryPolicy.KEEP_LAST,            # 历史策略：保持最后
            depth=1                                      # 队列深度
        )

        # 仅订阅邻居（根据邻接矩阵判断）
        neighbors = [j for j in range(self.n) if self.controller.A[self.agent_index, j] != 0]
        # ✅ 加上自身（总是订阅自己的里程计）
        if self.agent_index not in neighbors:
           neighbors.append(self.agent_index)
           
        # 为每个邻居（包括自己）创建里程计订阅
        for j in neighbors:
            topic = f'/px4_{j+1}/fmu/out/vehicle_odometry'  # 邻居里程计话题
            # 使用lambda函数捕获j值，为每个邻居创建独立的回调
            self.create_subscription(VehicleOdometry, topic, lambda msg, j=j: self.odom_cb(msg, j), self.qos)

        # subscribe to leader odom only if this agent can hear leader
        # 只有能听到领航者的智能体才订阅领航者里程计
        if self.can_hear_leader:
            # leader publishes to '/fmu/out/vehicle_odometry'
            # 领航者发布到 '/fmu/out/vehicle_odometry'
            self.create_subscription(VehicleOdometry, '/fmu/out/vehicle_odometry', self.leader_odom_cb, self.qos)
            self.get_logger().info(f"[Agent {self.agent_index+1}] Subscribed to leader odom on /fmu/out/vehicle_odometry")

        # publishers: offboard heartbeat, goto_setpoint for this agent
        # 发布器：Offboard心跳、轨迹设定点（针对本智能体）
        self.offboard_pub = self.create_publisher(OffboardControlMode, f'/px4_{self.agent_index+1}/fmu/in/offboard_control_mode', self.qos)
        self.traj_pub = self.create_publisher(TrajectorySetpoint, f'/px4_{self.agent_index+1}/fmu/in/trajectory_setpoint', self.qos)
        
        # 添加 VehicleCommand 发布器
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, f'/px4_{self.agent_index+1}/fmu/in/vehicle_command', self.qos)

        # control params
        # 控制参数
        self.max_speed = cfg.get('max_speed', 2.0)  # m/s limit 最大速度限制

        # start background thread to publish heartbeat & setpoints
        # 启动后台线程发布心跳和设定点
        self._stop_thread = False
        t = threading.Thread(target=self.publish_loop, daemon=True)
        t.start()

        # wait until more than 10 OffboardControlMode heartbeats have been published
        # 等待至少10个OffboardControlMode心跳发布
        time.sleep(1)
        
        # 启动 Offboard 模式
        self.start_offboard()

        self.get_logger().info(f"[Agent {self.agent_index+1}] Node created. can_hear_leader={self.can_hear_leader}")

    def publish_vehicle_command(self, command, **params) -> None:
        """Publish a vehicle command.
        发布车辆命令"""
        msg = VehicleCommand()
        msg.command = command  # 命令类型
        # 设置命令参数
        msg.param1 = params.get("param1", 0.0)
        msg.param2 = params.get("param2", 0.0)
        msg.param3 = params.get("param3", 0.0)
        msg.param4 = params.get("param4", 0.0)
        msg.param5 = params.get("param5", 0.0)
        msg.param6 = params.get("param6", 0.0)
        msg.param7 = params.get("param7", 0.0)
        msg.target_system = 1 + self.agent_index + 1  # 目标系统对应 1 + px4_{id+1}
        msg.target_component = 1    # 目标组件
        msg.source_system = 1       # 源系统
        msg.source_component = 1    # 源组件
        msg.from_external = True     # 来自外部
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # 时间戳
        self.vehicle_command_pub.publish(msg)

    def engage_offboard_mode(self):
        """Switch to offboard mode.
        切换到Offboard模式"""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
        self.get_logger().info("Switching to offboard mode")

    def start_offboard(self):
        """初始化 Offboard 模式切换
        Initialize Offboard mode switching"""
        # 启动前确保心跳已发布
        self.engage_offboard_mode()
        self.throttle_log(1.0, f"[Agent {self.agent_index+1}] Initiated Offboard mode", tag=f"offboard_{self.agent_index+1}")

    def odom_cb(self, msg: VehicleOdometry, j: int):
        """里程计回调函数：处理智能体j的里程计信息
        Odometry callback: process odometry information for agent j"""
        self.odom_cb_is_working = True
        try:
            # 从消息中提取位置和速度
            pos = np.array([float(msg.position[0]), float(msg.position[1])])
            vel = np.array([float(msg.velocity[0]), float(msg.velocity[1])])
        except Exception:
            # message may have NaNs, ignore
            # 消息可能包含NaN，忽略
            return

        # 存储邻居信息
        self.positions[j] = pos
        self.velocities[j] = vel
        self.have_pos[j] = True

        # debug throttle
        # 调试信息节流输出
        self.throttle_log(2.0, f"[Agent {self.agent_index+1}] saw agent {j+1} pos={pos} vel={vel}", tag=f"odom_{self.agent_index+1}_{j+1}")

    def leader_odom_cb(self, msg: VehicleOdometry):
        """领航者里程计回调函数
        Leader odometry callback function"""
        self.leader_odom_cb_is_working = True
        try:
            # 提取领航者位置和速度
            pos = np.array([float(msg.position[0]), float(msg.position[1])])
            vel = np.array([float(msg.velocity[0]), float(msg.velocity[1])])
        except Exception:
            return
        self.leader_pos = pos
        self.leader_vel = vel
        self.have_leader = True
        self.throttle_log(1.0, f"[Agent {self.agent_index+1}] can hear leader, Leader pos={pos} vel={vel}", tag=f"leader_{self.agent_index+1}")

    def publish_loop(self):
        """发布循环：在后台线程中持续发布心跳和设定点
        Publish loop: continuously publish heartbeat and setpoints in background thread"""
        rate_hz = 10.0  # 发布频率
        period = 1.0 / rate_hz
        while rclpy.ok() and not self._stop_thread:
            try:
                # publish offboard control mode (heartbeat)
                # 发布Offboard控制模式（心跳）
                self.publish_offboard_control_heartbeat_signal(control_mode='velocity')

                # compute and publish setpoint
                # 计算并发布设定点
                self.publish_current_setpoint()
                time.sleep(period)
            except Exception as e:
                # ensure background thread doesn't crash silently
                # 确保后台线程不会静默崩溃
                self.get_logger().error(f"[Agent {self.agent_index+1}] publish loop exception: {e}")
                time.sleep(0.1)

    def publish_offboard_control_heartbeat_signal(self, control_mode='velocity'):
        """发布Offboard控制模式心跳信号
        Publish Offboard control mode heartbeat signal"""
        msg = OffboardControlMode()
        # ensure fields deterministic
        # 确保字段确定性
        msg.position = False
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        # 根据控制模式设置相应字段
        if control_mode == 'position':
            msg.position = True
        elif control_mode == 'velocity':
            msg.velocity = True
        elif control_mode == 'acceleration':
            msg.acceleration = True
        elif control_mode == 'attitude':
            msg.attitude = True
        elif control_mode == 'body_rate':
            msg.body_rate = True
            
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # 时间戳
        self.offboard_pub.publish(msg)  # 发布消息

    def apply_deadzone(self, u, threshold=0.2):
        """
        对二维速度向量 u 应用死区，过滤小幅噪声
        Apply deadzone to 2D velocity vector u to filter small noise
        """
        u_filtered = np.zeros_like(u)
        for i in range(len(u)):
            if abs(u[i]) >= threshold:
                u_filtered[i] = u[i]  # 超过阈值则保留
            else:
                u_filtered[i] = 0.0   # 低于阈值则置零
        return u_filtered

    def publish_current_setpoint(self):
        """发布当前设定点
        Publish current setpoint"""
        idx = self.agent_index
        # require at least own position known
        # 至少需要知道自己的位置
        if not self.have_pos[idx]:
            self.throttle_log(2.0, f"[Agent {idx+1}] waiting for own odometry...", tag=f"wait_pos_{idx+1}")
            return

        # Compute desired horizontal velocity from consensus controller
        # 从一致性控制器计算期望水平速度
        u = self.controller.compute_control(
            idx, 
            self.positions, 
            self.leader_pos, 
            self.leader_vel, 
            have_leader=self.have_leader
        )

        # Clip speed to maximum allowed
        # 限制速度不超过最大值
        speed = math.hypot(u[0], u[1])  # 计算速度大小
        if speed > self.max_speed and speed > 1e-6:
            scale = self.max_speed / speed  # 计算缩放比例
            u = u * scale  # 等比例缩放速度向量

        # 应用死区过滤噪声
        u = self.apply_deadzone(u, threshold=0.05)

        # Create TrajectorySetpoint message
        # 创建轨迹设定点消息
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)  # 时间戳

        # Position: only control Z (hover height), leave X/Y as NaN (don't control position)
        # 位置：只控制Z轴（悬停高度），X/Y设为NaN（不控制位置）
        msg.position = [float('nan'), float('nan'), float(-self.cfg.get('hover_z', -3.0))]

        # Velocity: control horizontal velocity from consensus, vertical = 0
        # 速度：控制一致性计算的水平速度，垂直速度为0
        msg.velocity = [float(u[0]), float(u[1]), 0.0]

        # Acceleration: not used (set to NaN)
        # 加速度：未使用（设为NaN）
        msg.acceleration = [float('nan')] * 3

        # Yaw: optional, e.g., keep current yaw or point forward
        # 偏航角：可选，例如保持当前偏航或指向前方
        msg.yaw = 1.57     # 90度偏航角
        msg.yawspeed = 0.0  # 偏航角速度为0

        # Publish the message
        # 发布消息
        self.traj_pub.publish(msg)

        # 节流日志输出
        self.throttle_log(
            0.5,
            f"[Agent {idx+1}] publish traj_setpoint: vel_xy={u}, z={msg.position[2]}",
            tag=f"set_{idx+1}"
        )

    def throttle_log(self, interval_sec: float, msg: str, level="info", tag="default"):
        """节流日志输出，避免过于频繁的日志
        Throttled log output to avoid overly frequent logging"""
        now = time.time()
        last = self._last_log.get(tag, 0.0)  # 获取上次记录时间
        if (now - last) > interval_sec:  # 如果超过时间间隔
            getattr(self.get_logger(), level)(msg)  # 记录日志
            self._last_log[tag] = now  # 更新最后记录时间

    def destroy(self):
        """销毁节点
        Destroy node"""
        self._stop_thread = True  # 停止后台线程
        super().destroy_node()

# ---------------------------
# Main: read YAML, create nodes, spin
# 主函数：读取YAML，创建节点，运行
# ---------------------------
def main():
    # locate swarm.yaml in same directory
    # 定位同目录下的swarm.yaml文件
    script_dir = os.path.dirname(os.path.realpath(__file__))
    cfg_path = os.path.join(script_dir, 'swarm.yaml')
    if not os.path.exists(cfg_path):
        print(f"ERROR: config file not found: {cfg_path}")
        print("Create swarm.yaml in same directory with fields: n, A, b, (optional) formation")
        sys.exit(1)

    # 读取YAML配置文件
    with open(cfg_path, 'r') as f:
        cfg = yaml.safe_load(f)

    # validate configuration
    # 验证配置
    N = int(cfg.get('n', 0))  # 智能体数量
    A = cfg.get('A', None)    # 邻接矩阵
    b = cfg.get('b', None)    # 牵引向量
    formation = cfg.get('formation', None)  # 编队构型
    if A is None or b is None:
        print("ERROR: swarm.yaml must contain 'A' and 'b'")
        sys.exit(1)
    if len(A) != N or len(b) != N:
        print("ERROR: mismatch lengths in swarm.yaml (n vs A vs b)")
        sys.exit(1)

    # optionally use formation from yaml, else default created in controller
    # 可选使用YAML中的编队配置，否则使用控制器的默认配置
    rclpy.init()  # 初始化ROS2

    # 创建一致性控制器
    controller = ConsensusController(A, b, formation)

    # create agent nodes 1..N
    # 创建智能体节点 1..N
    nodes = []
    for i in range(N):
        node = PX4Agent(i, cfg, controller)
        nodes.append(node)

    # Single process multi-threaded executor
    # 单进程多线程执行器
    executor = MultiThreadedExecutor(num_threads=max(4, N))
    for node in nodes:
        executor.add_node(node)  # 添加节点到执行器

    print("Swarm controller started. Agents created:", N)
    try:
        executor.spin()  # 开始执行
    except KeyboardInterrupt:
        print("Shutting down swarm controller...")
    finally:
        # cleanup
        # 清理资源
        for node in nodes:
            try:
                executor.remove_node(node)  # 从执行器移除节点
                node.destroy()  # 销毁节点
            except Exception:
                pass
        rclpy.shutdown()  # 关闭ROS2

if __name__ == '__main__':
    main()
