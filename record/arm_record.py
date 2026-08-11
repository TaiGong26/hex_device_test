"""
arm_record — 机械臂轨迹录制工具

录制机械臂关节轨迹，输出 JSON 文件，供 arm_test_v2 回放（耐久性测试）。
本工具只负责"录"，回放由 test/arm_test_v2.py 的 --traj-json 完成。

────────────────────────────────────────────────────────────────
 一、两种模式
────────────────────────────────────────────────────────────────

  mode=record  手动逐点录制（默认）
    运行后进入控制循环，机械臂保持零位。用键盘控制录点：
        r  → 记录当前机械臂关节位置，立即写盘
        c  → 清空已记录的所有点
    适合"摆好姿势 → 按 r 录一个点"的手动轨迹采集。
    注意：按键监听走 stdin（终端），必须在真实终端运行，不能后台/重定向。

  mode=stream  定时流式录制
    按 --samp-rate 定时采样机械臂状态，自动连续录点。
    适合直接跟随手柄/示教器的连续运动，无需手动按键。

────────────────────────────────────────────────────────────────
 二、命令行参数
────────────────────────────────────────────────────────────────

  --mode        record | stream   录制模式（默认 record）
  --ctrl-rate   控制循环频率 Hz（默认 1000，传给 HexRate）
  --samp-rate   采样频率 Hz（默认 100，stream 模式生效）
  --ip          机械臂 IP（必填）
  --port        机械臂端口（必填）
  --output      输出轨迹 JSON 路径（必填）
  --robot-type  archer_y6 | firefly_y6（默认 archer_y6）
  --grip-type   empty | gp80 | gr100 | gp100（默认 empty）

────────────────────────────────────────────────────────────────
 三、用法示例
────────────────────────────────────────────────────────────────

  # 手动逐点录制
  python arm_record.py --mode record --ip 192.168.1.100 --port 8080 \
      --output /tmp/traj.json

  # 流式录制（100Hz 采样）
  python arm_record.py --mode stream --ip 192.168.1.100 --port 8080 \
      --samp-rate 100 --output /tmp/traj_stream.json

  Ctrl+C 正常结束：record 模式会闭合 JSON 并修正 info 头部元数据。

────────────────────────────────────────────────────────────────
 四、输出格式
────────────────────────────────────────────────────────────────

  {
    "info": {
      "start_time_ns": ..., "end_time_ns": ..., "total_points": N,
      "dof": 6, "robot_type": "archer_y6", "gripper_type": "empty"
    },
    "point": {
      "1": { "ts_ns": <相对 ns, 首点=0>, "arm": [6 个关节弧度], "grip": [...] },
      ...
    }
  }

  由 src/hex_device_test/tools/PointLoader.py 的 TaskConfigLoader 读取。

────────────────────────────────────────────────────────────────
 五、与回放对接
────────────────────────────────────────────────────────────────

  # linear 插值（默认）
  python test/arm_test_v2.py --url ws://<IP>:8439 --traj-json /tmp/traj.json

  # S 曲线插值
  python test/arm_test_v2.py --url ws://<IP>:8439 \
      --traj-json /tmp/traj.json --interp s_curve

  回放说明：arm_test_v2 按录制时间戳驱动循环回放，循环边界在末点→首点
  之间补一段插值过渡（时长=平均段时长），避免硬跳导致机械臂过冲。
  夹爪轨迹已录入 JSON，但回放侧（第 3 轮）暂不驱动，见代码内 ###TODO。

────────────────────────────────────────────────────────────────
 六、注意事项
────────────────────────────────────────────────────────────────

  1. 本工具 import 使用相对路径（from traj_recorder import ...），
     必须在 record/ 目录下运行。
  2. record 模式的键盘按键需要真实终端（termios/tty 设置）。
  3. 录制时机械臂处于零位命令（set_arm_mit_cmd 全零），
     需先把手柄/示教器操纵机械臂到目标位姿再按 r 录点。
"""
import argparse
import time
import traceback
import numpy as np
from hex_util_runtime import ns_now, HexRate
import sys

from hex_driver_robot import (
    HexRobotArcherY6, 
    HexRobotArcherY6Params,
    HexRobotFireflyY6,
    HexRobotFireflyY6Params,

)
from hex_util_msg.dataclass import HexDcBaseVector3

from traj_recorder import TrajRecorder
from traj_stream import TrajStream

def _stamp_to_ns(stamp) -> int:
    """Convert HexDcBaseTime stamp to nanoseconds."""
    return int(stamp.secs * 1_000_000_000 + stamp.nsecs)

def main() -> None:
    parser = argparse.ArgumentParser(description="Arm trajectory recorder / streamer")
    parser.add_argument(
        "--mode",
        type=str,
        choices=["record", "stream"],
        default="record",
        help="'record': full trajectory recording (default), 'stream': light-weight streaming only",
    )
    parser.add_argument(
        "--ctrl-rate",
        type=int,
        default=1000,
        help="Control loop rate in Hz (default: 1000, passed to HexRate)",
    )
    parser.add_argument(
        "--samp-rate",
        type=int,
        default=100,
        help="Sampling rate in Hz (default: 100, controls how often data is recorded)",
    )
    parser.add_argument(
        "--ip",
        type=str,
        required=True,
        help="Robot IP address",
    )
    parser.add_argument(
        "--port",
        type=int,
        required=True,
        help="Robot port",
    )
    parser.add_argument(
        "--output",
        type=str,
        required=True,
        help="Output file path for trajectory data",
    )
    parser.add_argument(
        "--robot-type",
        type=str,
        default="archer_y6",
        choices=["archer_y6", "firefly_y6"],
        help="Robot type name (e.g. archer_y6, firefly_y6)",
    )
    
    parser.add_argument(
        "--grip-type",
        type=str,
        default="empty",
        choices=["empty", "gp80", "gr100", "gp100"],
        help="Grip type (empty or gp100)",
    )
    
    args = parser.parse_args()
    mode = args.mode
    ctrl_rate = args.ctrl_rate
    samp_rate = args.samp_rate

    # Compute sampling decimation: record once every N loop iterations
    sample_interval = max(1, round(ctrl_rate / samp_rate))

    _ROBOT_MAP = {
        "archer_y6": (HexRobotArcherY6Params, HexRobotArcherY6),
        "firefly_y6": (HexRobotFireflyY6Params, HexRobotFireflyY6),
    }
    _params_cls, _robot_cls = _ROBOT_MAP[args.robot_type]
    params = _params_cls(
        host=args.ip,
        port=args.port,
        ctrl_rate=500,
        state_buffer_size=200,
        sens_ts=False,
        grip_type=args.grip_type,
    )
    robot = None
    recorder = None
    stream = None
    try:
        robot = _robot_cls(params)
        robot.start()
        print(f"dofs: {robot.get_dofs()}")

        rate = HexRate(ctrl_rate)

        if mode == "record":
            recorder = TrajRecorder(
                args.output,
                robot_type=args.robot_type,
                gripper_type=params.grip_type,
            )
            recorder.start()
            print("[Mode] full trajectory recording")

        if mode == "stream":
            stream = TrajStream(
                dec=3,
                robot_type=args.robot_type,
                gripper_type=params.grip_type,
            )
            stream.start(output_path=args.output, samp_hz=samp_rate)
            print("[Mode] light-weight streaming")

        cnt = 0

        while robot.is_working():
            rate.sleep()
            
            robot.set_arm_mit_cmd({
                "ts_ns": ns_now(),
                "jnt_pos": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "jnt_vel": np.zeros(6),
                "mit_tau": np.zeros(6),
                "mit_kp": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "mit_kd": np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                "grav": HexDcBaseVector3(0.0, 0.0, -9.8),
            })

            robot.set_grip_mit_cmd({
                "ts_ns": ns_now(),
                "jnt_pos": np.array([0.0]),
                "jnt_vel": np.zeros(1),
                "mit_tau": np.zeros(1),
                "mit_kp": np.array([0.0]),
                "mit_kd": np.array([0.0]),
            })

            cnt += 1
            if cnt >= sample_interval:
                cnt = 0
                if mode == "stream":
                    assert stream is not None
                    try:
                        stream.record(robot)
                    except Exception as e:
                        print(f"\033[33m[TrajStream] record error: {e}\033[0m")
            
                if mode == "record":
                    assert recorder is not None
                    try:
                        recorder.check_and_record(robot)
                    except Exception as e:
                        print(f"\033[33m[Recorder] check_and_record error: {e}\033[0m")
            

    except KeyboardInterrupt:
        pass
    except (ConnectionError, ConnectionRefusedError, TimeoutError) as e:
        print(f"\033[31mConnection failed: {e}\033[0m")
    except Exception:
        traceback.print_exc()
    finally:
        if robot is not None:
            robot.stop()
            print("robot stopped cleanly")
        if mode == "record":
            if recorder is not None:
                try:
                    recorder.stop()
                except Exception as e:
                    print(f"\033[33m[Recorder] stop error: {e}\033[0m")
        if mode == "stream":
            try:
                if stream is not None:
                    stream.stop()
            except Exception as e:
                print(f"\033[33m[TrajStream] stop error: {e}\033[0m")


if __name__ == "__main__":
    main()
