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
from hex_device_test.tools.trajectory_loader import (
    DEFAULT_SEGMENT_DURATION,
    estimate_waypoints_memory_mb,
    get_replay_segment_duration,
    load_waypoints_with_segment_boundaries,
)

# clean up
def cleanup(coordinator):
    print("cleanup")
    # cleanup 期间屏蔽中断信号，防止机械臂返回过程中被强制终止
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    try:
        coordinator.shutdown()
        print("coordinator shutdown")
        coordinator._stop_event.wait()
    finally:
        # 恢复默认处理（程序即将退出，主要是给子线程清理兜底）
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

# 信号处理回调
def signal_handler(sig, frame, stop_event: threading.Event):
    print(f"[Signal] {sig} received, exit")
    stop_event.set()
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    signal.signal(signal.SIGTERM, signal.SIG_IGN)
    print("---------------------------------- signal handler ----------------------------------")
    return

DEFAULT_ARM_POSITION = [
    [-0.75, -0.25, 3.0, -1.5, 0, 0],
    [-0.75, 1.35, 0, 1.5, 0, 2.5],
    [-0.75, -0.25, 3.0, -1.5, 0, 0],
    [-2.00, -0.25, 3.0, -1.5, -1.5, 0],
    [2.00, -0.25, 3.0, -1.5, 1.5, 0],
    [0.75, -0.25, 3.0, -1.5, 0, 0],
    [0.75, -1.5, 1.5, 1.5, 0, -2.5],
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
        '--timeout',
        action='store_true',
        default=False,
        help='Enable API timeout check.'
    )

    parser.add_argument(
        '--points-json',
        type=Path,
        nargs='+',
        default=None,
        metavar='JSON',
        help='One or more recorded trajectory JSON files; waypoints are concatenated in order'
    )

    parser.add_argument(
        '--stride',
        type=int,
        default=1,
        help='Sample every N frames when loading --points-json (default: 1)'
    )

    parser.add_argument(
        '--time-sleep',
        type=float,
        default=0.0,
        metavar='SECONDS',
        help='Seconds to hold init_pos between trajectory segments when using --points-json (default: 0, 0 to skip return and hold)'
    )

    parser.add_argument(
        '--raw',
        action='store_true',
        default=False,
        help='Replay waypoints as-is without S-curve interpolation between points'
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
    check_timeout = args.timeout
    temp_csv_dir = args.temp_csv_dir
    time_sleep = args.time_sleep
    interpolate = not args.raw
    segment_ends = None

    config_dict = {
        'name': 'Archer_d6y',
        'dof_num': 'six_axis',
        'motor_model': [0x80] * 6,
        'joints': [{
            'joint_name': 'joint_1',
            'joint_limit': [-2.7, 2.7, -1.0, 1.0, 0.0, 0.0]
        }, {
            'joint_name': 'joint_2',
            'joint_limit': [-1.57, 2.094, -1.0, 1.0, 0.0, 0.0]
        }, {
            'joint_name': 'joint_3',
            'joint_limit': [0.0, 3.14159265359, -1.0, 1.0, 0.0, 0.0]
        }, {
            'joint_name': 'joint_4',
            'joint_limit': [-1.5, 1.5, -1.0, 1.0, 0.0, 0.0]
        }, {
            'joint_name': 'joint_5',
            'joint_limit': [-1.56, 1.56, -1.5, 1.5, 0.0, 0.0]
        }, {
            'joint_name': 'joint_6',
            'joint_limit': [-1.57, 1.57, -1.5, 1.5, 0.0, 0.0]
        }]
    }

    if args.points_json is not None:
        for json_path in args.points_json:
            if not json_path.is_file():
                print(f"Error: points JSON not found: {json_path}")
                return
        arm_position, segment_ends = load_waypoints_with_segment_boundaries(
            args.points_json, stride=args.stride
        )
        segment_duration = get_replay_segment_duration(args.points_json, stride=args.stride)
        mem_mb = estimate_waypoints_memory_mb(len(arm_position))
        print(f"Loaded {len(arm_position)} waypoints from {len(args.points_json)} file(s)")
        for idx, json_path in enumerate(args.points_json):
            end_idx = segment_ends[idx] - 1
            print(f"  - {json_path} (segment {idx + 1} end waypoint index: {end_idx})")
        print(
            f"stride={args.stride}, segment_duration={segment_duration}s, "
            f"time_sleep={time_sleep}s, interpolate={interpolate}, "
            f"estimated waypoints memory ~{mem_mb:.2f} MB"
        )
    else:
        arm_position = DEFAULT_ARM_POSITION
        segment_duration = DEFAULT_SEGMENT_DURATION
        print(
            f"Using default trajectory ({len(arm_position)} waypoints, "
            f"segment_duration={segment_duration}s, interpolate={interpolate})"
        )

    stop_event = threading.Event()
    coordinator = None
    try:
        coordinator = ArmCoordinator(
            dev_ip_list,
            enable_kcp,
            arm_config=config_dict,
            robot_type=args.robot_type,
            waypoints=arm_position,
            segment_duration=segment_duration,
            segment_ends=segment_ends,
            time_sleep=time_sleep,
            interpolate=interpolate,
            enable_view=enable_view,
            check_timeout=check_timeout,
            temp_csv_dir=temp_csv_dir,
        )

        # 信号处理
        signal.signal(signal.SIGINT, lambda sig, frame: signal_handler(sig, frame, stop_event))
        signal.signal(signal.SIGTERM, lambda sig, frame: signal_handler(sig, frame, stop_event))

        stop_event.wait()

    except KeyboardInterrupt:
        print("keyboard interrupt")

    except Exception as e:
        print(f"main error: {e}")
        traceback.print_exc()
    finally:
        # 恢复默认信号处理
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)

        if coordinator is not None:
            cleanup(coordinator)
        print("[finally] you can try a gain ctrl c to exit the terminal")
        sys.exit(0)

if __name__ == "__main__":
    main()
