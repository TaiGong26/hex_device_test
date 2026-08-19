#!/usr/bin/env python3
# -*- coding:utf-8 -*-

from hex_util_runtime import HexRate

from hex_driver_robot import (
    HexRobotArcherY6, 
    HexRobotArcherY6Params,
    HexRobotFireflyY6, 
    HexRobotFireflyY6Params,
)
from robomeshcat import Scene, Robot

# 关节索引 -> URDF 关节名。Archer Y6 上报的 arm_state.jnt.position
# 顺序为关节 1..6，对应 URDF 的 joint_1..joint_6（见 empty.urdf）。
# 若真机上某个关节方向相反，可在此调整或单独处理。
ARM_JOINT_NAMES = [
    "joint_1", "joint_2", "joint_3",
    "joint_4", "joint_5", "joint_6",
]

VIZ_RATE = 100  # Hz — 向可视化推送机械臂状态的频率

URDF_PATH = '/home/hexfellow/ttg/docker/hex_ros2_dev/hex_ros2_ws/src/hex_ros_urdf_archer_y6/urdf/empty.urdf'
MESH_DIR = '/home/hexfellow/ttg/docker/hex_ros2_dev/hex_ros2_ws/src'

def main() -> None:
    params = HexRobotFireflyY6Params(
        host="172.18.0.50",
        port=8439,
        ctrl_rate=500,
        state_buffer_size=200,
        sens_ts=False,
        grip_type="empty",
    )

    # --- robomeshcat scene ------------------------------------------------
    scene = Scene()


    vis_robot = Robot(urdf_path=URDF_PATH, mesh_folder_path=MESH_DIR)
    scene.add_robot(vis_robot)

    try:
        robot = HexRobotFireflyY6(params)  
    except (ConnectionError, TimeoutError) as e:
        print(f"\033[31m[Error] 机械臂连接失败: {e}\033[0m")
        return

    rate = HexRate(VIZ_RATE)
    try:
        robot.start()
        print("[Viz] Archer Y6 connected — streaming joint angles to robomeshcat...")
        while robot.is_working():
            rate.sleep()

            # 6 轴关节角度 [rad]
            state = robot.get_arm_state()
            if state is not None:
                pos = state.arm_state.jnt.position
                for i, name in enumerate(ARM_JOINT_NAMES):
                    vis_robot[name] = pos[i]
    except KeyboardInterrupt:
        print("\n[Viz] Stopped by user.")
    finally:
        robot.stop()


if __name__ == "__main__":
    main()
