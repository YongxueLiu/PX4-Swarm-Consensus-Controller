#!/usr/bin/env python3

import subprocess
import os
import time
import yaml

# ------------------- 配置部分 -------------------
YAML_FILE = os.path.join(os.path.dirname(__file__), "startup.yaml")

PX4_PATH = "~/PX4-Autopilot"
SIMULATION_CMD = "./build/px4_sitl_default/bin/px4"
MODEL_NAME = "gz_x500"

GAZEBO_SCRIPT_DIR = "~/.simulation-gazebo/worlds"
GAZEBO_SCRIPT_NAME = "simulation-gazebo"


def load_config_from_yaml(yaml_path):
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)
    formation = data.get('formation', [])
    if not formation:
        raise ValueError("YAML 文件中未找到 'formation' 字段或为空")
    return [(float(p[0]), float(p[1])) for p in formation]


def main():
    # 1. 加载配置
    positions = load_config_from_yaml(YAML_FILE)
    NUM_DRONES = len(positions)
    print(f"从 {YAML_FILE} 加载 {NUM_DRONES} 架无人机的 formation 配置")

    full_px4_path = os.path.expanduser(PX4_PATH)
    px4_exec = os.path.join(full_px4_path, SIMULATION_CMD)

    # 2. 启动 Gazebo（独立终端或后台）
    print("启动Gazebo仿真环境...")
    gazebo_script_dir = os.path.expanduser(GAZEBO_SCRIPT_DIR)
    gazebo_script_path = os.path.join(gazebo_script_dir, GAZEBO_SCRIPT_NAME)

    if not os.path.isfile(gazebo_script_path):
        raise FileNotFoundError(f"Gazebo 启动脚本未找到: {gazebo_script_path}")

    subprocess.Popen(
        ['gnome-terminal', '--', 'bash', '-c',
         f'cd "{gazebo_script_dir}" && python3 {GAZEBO_SCRIPT_NAME}; exec bash']
    )

    # 3. 等待 Gazebo
    print("等待Gazebo启动...")
    time.sleep(5)

    # 4. 构建所有 tab 命令
    print(f"准备在同一个终端窗口中启动 {NUM_DRONES} 个标签页...")
    cmd_list = ['gnome-terminal']

    for i, pos in enumerate(positions):
        x, y = pos
        # 每个 tab 的 bash 命令
        tab_cmd = (
            f'export PX4_GZ_STANDALONE=1 && '
            f'export PX4_SYS_AUTOSTART=4001 && '
            f'export PX4_GZ_MODEL_POSE="{x},{y},0" && '
            f'export PX4_SIM_MODEL={MODEL_NAME} && '
            f'cd "{full_px4_path}" && '
            f'./build/px4_sitl_default/bin/px4 -i {i}; exec bash'
        )
        # 关键修改：移除多余的 '--' 分隔符
        cmd_list.extend(['--tab', '--title', f'Drone-{i}', '-e', f'bash -c "{tab_cmd}"'])

    # 5. 一次性启动多 tab 终端
    subprocess.Popen(cmd_list)

    print(f"已在单个终端窗口中打开 {NUM_DRONES} 个标签页，分别运行无人机 0 到 {NUM_DRONES - 1}")
    print("注意：PX4 实例可能需要额外时间完成初始化。")


if __name__ == "__main__":
    main()
