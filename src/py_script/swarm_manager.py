import yaml
import rclpy
from rclpy.executors import MultiThreadedExecutor
from src.consensus_controller import ConsensusController
from src.px4_agent_node import PX4Agent
import time


def main():
    rclpy.init()

    # 1️⃣ 读取编队配置
    with open('config/swarm.yaml', 'r') as f:
        cfg = yaml.safe_load(f)

    # 2️⃣ 初始化一致性控制器
    controller = ConsensusController(cfg['A'], cfg['b'], cfg['formation'])

    # 3️⃣ 创建多机节点实例
    nodes = []
    for i in range(cfg['n']):
        node = PX4Agent(i, cfg, controller)
        nodes.append(node)
        node.get_logger().info(f"✅ PX4Agent {i} initialized.")

    # 4️⃣ 使用 ROS2 自带多线程执行器，统一 spin
    executor = MultiThreadedExecutor(num_threads=len(nodes))
    for node in nodes:
        executor.add_node(node)

    print("🚀 Swarm Manager running... Press Ctrl+C to exit.")
    try:
        executor.spin()  # 阻塞执行
    except KeyboardInterrupt:
        print("\n🛑 Shutting down swarm...")
    finally:
        for node in nodes:
            executor.remove_node(node)
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
