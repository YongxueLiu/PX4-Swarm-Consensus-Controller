PX4-Swarm-Consensus-Controller / PX4集群一致性控制器

📋 Project Overview

A distributed multi-UAV swarm control system based on ROS2 and PX4, implementing consensus-based formation control algorithms. This project enables coordinated flight of multiple drones with virtual leader following capabilities.

✨ Key Features

• Distributed Control: Each UAV makes autonomous decisions based on neighbor information

• Consensus Algorithm: Implements robust formation control with mathematical guarantees

• Flexible Topology: Configurable communication graphs via YAML configuration

• Virtual Leader Support: Optional virtual leader for guided swarm movement

• PX4 Integration: Seamless integration with PX4 autopilot via ROS2

• Real-time Performance: Multi-threaded architecture for high-frequency control

🚀 Quick Start

Prerequisites

• ROS2

• PX4 Autopilot (v1.13+)

• Python 3.8+

• NumPy, PyYAML

Installation

git clone https://github.com/your-username/PX4-Swarm-Consensus-Controller.git
cd PX4-Swarm-Consensus-Controller


Configuration

Create swarm.yaml in the project root:
n: 3                    # Number of UAVs
max_speed: 2.0         # Maximum speed (m/s)
hover_z: -3.0          # Hover altitude

# Adjacency matrix (communication topology)
A: 
  - [0, 1, 0]          # UAV1 connected to UAV2
  - [1, 0, 1]          # UAV2 connected to UAV1 and UAV3
  - [0, 1, 0]          # UAV3 connected to UAV2

# Pinning vector (leader connection)
b: [1, 0, 0]           # Only UAV1 follows virtual leader

# Formation offsets (relative to leader)
formation:
  - [0.0, 0.0]         # UAV1 position
  - [1.0, 0.0]         # UAV2 position  
  - [2.0, 0.0]         # UAV3 position


Running the Swarm

# Terminal 1: Start ROS2
source /opt/ros/humble/setup.bash

# Terminal 2: Run the controller
python3 swarm_single_file.py


🏗️ System Architecture


Virtual Leader
     ↓ (odometry)
UAV1 (Pinning Node) ↔ UAV2 ↔ UAV3
     Consensus Control
     ↓
Formation Flight


📁 Project Structure





🔧 Advanced Configuration

Custom Communication Topologies

Modify the adjacency matrix in swarm.yaml to create different communication patterns:

Line Formation:
A:
  - [0, 1, 0, 0]
  - [1, 0, 1, 0] 
  - [0, 1, 0, 1]
  - [0, 0, 1, 0]


Star Topology:
A:
  - [0, 1, 1, 1]
  - [1, 0, 0, 0]
  - [1, 0, 0, 0]
  - [1, 0, 0, 0]


Control Parameters

Adjust control gains in ConsensusController class:
self.g_pin = 0.10        # Leader tracking gain
self.k_consensus = 0.20  # Consensus gain


🐛 Troubleshooting

Common Issues:
• Config file not found: Create swarm.yaml in the same directory

• ROS2 node not found: Source your ROS2 workspace properly

• PX4 not responding: Check microROS agent and PX4-SITL connection

Debug Mode:
Enable detailed logging by setting debug flags in the controller class.

🤝 Contributing

We welcome contributions! Please see our CONTRIBUTING.md for details.

📄 License

This project is licensed under the MIT License - see the LICENSE file for details.

📚 References

• Consensus-based formation control algorithms

• PX4 Offboard mode documentation

• ROS2 multi-node architecture best practices

<a name="中文"></a>
📋 项目概述

基于ROS2和PX4的分布式多无人机集群控制系统，实现了基于一致性算法的编队控制。该项目支持多无人机协同飞行，具备虚拟领航者跟随能力。

✨ 核心特性

• 分布式控制：每个无人机基于邻居信息自主决策

• 一致性算法：实现具有数学保证的鲁棒编队控制

• 灵活拓扑：通过YAML配置文件定义通信图

• 虚拟领航者：支持虚拟领航者引导集群运动

• PX4集成：通过ROS2与PX4飞控无缝集成

• 实时性能：多线程架构支持高频控制

🚀 快速开始

环境要求

• ROS2

• PX4飞控系统 (v1.13+)

• Python 3.8+

• NumPy, PyYAML

安装

git clone https://github.com/your-username/PX4-Swarm-Consensus-Controller.git
cd PX4-Swarm-Consensus-Controller


配置

在项目根目录创建 swarm.yaml：
n: 3                    # 无人机数量
max_speed: 2.0         # 最大速度 (米/秒)
hover_z: -3.0          # 悬停高度

# 邻接矩阵 (通信拓扑)
A: 
  - [0, 1, 0]          # UAV1 连接 UAV2
  - [1, 0, 1]          # UAV2 连接 UAV1 和 UAV3
  - [0, 1, 0]          # UAV3 连接 UAV2

# 牵引向量 (领航者连接)
b: [1, 0, 0]           # 仅UAV1跟随虚拟领航者

# 编队偏移 (相对于领航者)
formation:
  - [0.0, 0.0]         # UAV1位置
  - [1.0, 0.0]         # UAV2位置
  - [2.0, 0.0]         # UAV3位置


运行集群

# 终端1: 启动ROS2
source /opt/ros/humble/setup.bash

# 终端2: 运行控制器
python3 swarm_single_file.py


🏗️ 系统架构


虚拟领航者
     ↓ (里程计)
UAV1 (牵引节点) ↔ UAV2 ↔ UAV3
     一致性控制
     ↓
编队飞行


📁 项目结构


PX4-Swarm-Consensus-Controller/
├── swarm_single_file.py     # 主集群控制器
├── swarm.yaml              # 配置文件 (需创建)
├── requirements.txt        # Python依赖
├── launch/                 # ROS2启动文件
├── config/                 # 额外配置
├── README.md              # 本文件
└── docs/                  # 文档


🔧 高级配置

自定义通信拓扑

修改 swarm.yaml 中的邻接矩阵创建不同通信模式：

线性编队:
A:
  - [0, 1, 0, 0]
  - [1, 0, 1, 0]
  - [0, 1, 0, 1]
  - [0, 0, 1, 0]


星型拓扑:
A:
  - [0, 1, 1, 1]
  - [1, 0, 0, 0]
  - [1, 0, 0, 0]
  - [1, 0, 0, 0]


控制参数

在 ConsensusController 类中调整控制增益：
self.g_pin = 0.10        # 领航者跟踪增益
self.k_consensus = 0.20  # 一致性增益


🐛 故障排除

常见问题:
• 配置文件未找到：在同一目录创建 swarm.yaml

• ROS2节点未找到：正确source ROS2工作空间

• PX4无响应：检查microROS代理和PX4-SITL连接

调试模式:
在控制器类中设置调试标志以启用详细日志。

🤝 贡献指南

我们欢迎贡献！请参阅CONTRIBUTING.md了解详情。

📄 许可证

本项目采用MIT许可证 - 详见LICENSE。

📚 参考文献

• 基于一致性的编队控制算法

• PX4 Offboard模式文档

• ROS2多节点架构最佳实践

项目名称建议: PX4-Swarm-Consensus-Controller (英文), PX4集群一致性控制器 (中文)

这个README提供了完整的中英双语介绍，包含了项目概述、特性、安装使用说明、配置示例等内容，格式规范，适合直接用于GitHub项目页面。
