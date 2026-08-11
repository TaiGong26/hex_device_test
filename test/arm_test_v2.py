"""
arm_test_v2 — 新版入口

与 arm_test.py 完全相同的结构和 CLI 参数，仅 import 路径改为 ArmCoordinatorV2。
"""
import argparse
import traceback
import threading
import signal
import time
import sys
from pathlib import Path


from hex_device_test.managers.ArmCoordinatorV2 import ArmCoordinatorV2 as ArmCoordinator
from hex_device_test.controllers.TrajectoryController import DEFAULT_SEGMENT_DURATION
from hex_device_test.tools.PointLoader import TaskConfigLoader


DEFAULT_ARM_POSITION = [
    [-0.75, -0.25, 3.0, -1.5, 0, 0],
    [-0.75, 1.2, 0, 1.5, 0, 2.5],
    [-0.75, -0.25, 3.0, -1.5, 0, 0],
    [-2.00, -0.25, 3.0, -1.5, -1.5, 0],
    [2.00, -0.25, 3.0, -1.5, 1.5, 0],
    [0.75, -0.25, 3.0, -1.5, 0, 0],
    [0.75, -1.2, 1.5, 1.5, 0, -2.5],
    [0.75, -0.25, 3.0, -1.5, 0, 0],
]

def main():
    # 标准库中获取命令行参数：数组
    parser = argparse.ArgumentParser(
        description='Hexapod robotic arm trajectory planning and execution test',
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument(
        '--url',
        metavar='URL',
        nargs="+",
        required=True,
        default=["ws://0.0.0.0:8439"],
        help='WebSocket URL for HEX device connection, example: ws://0.0.0.0:8439 or ws://[::1%%eth0]:8439'
    )

    parser.add_argument(
        '--KCP',
        action='store_true',
        default=False,
        help='Enable KCP protocol for HEX device connection'
    )

    parser.add_argument(
        '--view',
        action='store_true',
        default=False,
        help='Enable real-time visualization of the robotic arm trajectory'
    )

    parser.add_argument(
        '--traj-json',
        type=Path,
        default=None,
        metavar='JSON',
        help='arm_record 录制的轨迹 JSON（TaskConfigLoader 格式），单文件；'
             '缺省用内置 DEFAULT_ARM_POSITION'
    )

    parser.add_argument(
        '--interp',
        type=str,
        choices=['s_curve', 'linear'],
        default='linear',
        metavar='MODE',
        help='轨迹插值方式: s_curve=S曲线平滑, linear=线性（默认 linear）'
    )

    parser.add_argument(
        '--temp-csv-dir',
        type=str,
        default=None,
        metavar='DIR',
        help='Directory to write per-device motor temperature CSV files (e.g. /tmp/temp_logs)'
    )

    parser.add_argument(
        '--robot-type',
        type=str,
        choices=["Archer_y6", "Firefly_y6"],
        default="Archer_y6",
        metavar='NAME',
        help='Robot model for all arms, one of: Archer_y6, Firefly_y6 (same for every device; default: Archer_y6)'
    )

    # =============== parse args ===============
    args = parser.parse_args()

    dev_ip_list = args.url
    if dev_ip_list is None or len(dev_ip_list) == 0:
        print("Error: No device IPs provided. Please specify at least one device IP using the --url argument.")
        return
    print(f"Device IP list: {dev_ip_list}")
    enable_kcp = args.KCP
    enable_view = args.view
    temp_csv_dir = args.temp_csv_dir
    interpolate = args.interp
    timestamps = None

    if args.traj_json is not None:
        if not args.traj_json.is_file():
            print(f"Error: trajectory JSON not found: {args.traj_json}")
            return
        loader = TaskConfigLoader(config_path=str(args.traj_json))
        arm_position = loader.get_waypoints()
        timestamps = loader.get_timestamps()
        duration_s = loader.raw_points[-1]['ts_ns'] * 1e-9
        print(
            f"Loaded {len(arm_position)} waypoints from {args.traj_json} "
            f"(duration={duration_s:.3f}s, interpolate={interpolate})"
        )
        # ###TODO: grip_waypoints = loader.get_grip_position() —— 夹爪回放第 3 轮范围外
    else:
        arm_position = DEFAULT_ARM_POSITION
        segment_duration = DEFAULT_SEGMENT_DURATION
        print(
            f"Using default trajectory ({len(arm_position)} waypoints, "
            f"segment_duration={segment_duration}s, interpolate={interpolate})"
        )

    coordinator = None

    try:
        coordinator = ArmCoordinator(
            dev_ip_list,
            enable_kcp,
            robot_type=args.robot_type,
            waypoints=arm_position,
            timestamps=timestamps,
            segment_duration=segment_duration,
            interpolate=interpolate,
            enable_view=enable_view,
            temp_csv_dir=temp_csv_dir,
        )

        while True:
            k = input()
            if k == 'q' or k=='Q':
                break
            
    except KeyboardInterrupt:
        print("keyboard interrupt")
    except Exception as e:
        print(f"main error: {e}")
        traceback.print_exc()
    finally:

        coordinator.shutdown()
        sys.exit(0)
        
        

if __name__ == "__main__":
    main()
