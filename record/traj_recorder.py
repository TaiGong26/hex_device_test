
import json
import threading
import sys
import os
import termios
import tty

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def _ts_to_ns(stamp) -> int:
    """将 HexDcBaseTime 转换为纳秒"""
    return int(stamp.secs * 1_000_000_000 + stamp.nsecs)


class TrajRecorder:
    """机械臂轨迹点记录器（流式写盘）

    通过后台键盘监听，在控制循环中记录机械臂的关节状态和末端位姿。
    每次按 r 即时写入磁盘（流式），断点不丢失。
    输出格式与 TrajStream 统一（含 info 元数据头）。

    使用方式:
        recorder = TrajRecorder("trajectory.json")
        recorder.start()

        while robot.is_working():
            rate.sleep()
            recorder.check_and_record(robot)

        recorder.stop()

    键盘指令:
        r  → 记录当前点并立即写盘
        c  → 清空所有记录点
    """

    def __init__(self, output_path=os.path.join(SCRIPT_DIR, "../jsons/trajectory.json"), dec=2,
                 robot_type="", gripper_type=""):
        self._output_path = output_path
        self._dec = dec
        self._robot_type = robot_type
        self._gripper_type = gripper_type
        self._f = None
        self._seq = 0
        self._last_abs_ns = None
        self._rel_ns = 0
        self._record_flag = False
        self._clear_flag = False
        self._running = False
        self._thread = None
        self._lock = threading.Lock()

    # ------------------------------------------------------------------
    # tools
    # ------------------------------------------------------------------

    @staticmethod
    def _ffmt(val: int, width: int) -> bytes:
        """固定宽度格式化，用于原地更新 info 头部"""
        return str(val).rjust(width).encode()

    def _write_header(self, samp_hz=None):
        """写入 JSON 头部 + 占位 metadata"""
        self._f.write(b'{\n')
        self._f.write(b'  "info": {\n')
        self._f.write(b'    "start_time_ns": ')
        self._info_anchor_start = self._f.tell()
        self._f.write(self._ffmt(0, 22))
        self._f.write(b',\n')
        self._f.write(b'    "end_time_ns":   ')
        self._info_anchor_end = self._f.tell()
        self._f.write(self._ffmt(0, 22))
        self._f.write(b',\n')
        self._f.write(b'    "total_points":  ')
        self._info_anchor_total = self._f.tell()
        self._f.write(self._ffmt(0, 12))
        self._f.write(b',\n')
        if samp_hz is not None:
            self._f.write(f'    "samp_hz": {samp_hz},\n'.encode())
        self._f.write(b'    "dof": 6,\n')
        self._f.write(f'    "robot_type": "{self._robot_type}",\n'.encode())
        self._f.write(f'    "gripper_type": "{self._gripper_type}"\n'.encode())
        self._f.write(b'  },\n')
        self._f.write(b'  "point": {\n')

    def _finalize(self):
        """闭合 JSON：截断末尾逗号、闭合括号、修正 info 头部"""
        if self._f is None:
            return
        fp = self._f

        # 去除最后一个逗号，闭合 JSON
        try:
            fp.seek(-2, os.SEEK_END)
            fp.truncate()
            fp.write(b'\n  }\n}\n')
        except OSError as e:
            print(f"\033[33m[Recorder] Finalize truncate/write error: {e}\033[0m")
        try:
            fp.close()
        except OSError as e:
            print(f"\033[33m[Recorder] Finalize close error: {e}\033[0m")
        self._f = None

        # 原地修正 info 头部中的元数据（固定宽度覆盖）
        try:
            with open(self._output_path, "r+b") as f:
                f.seek(self._info_anchor_start)
                f.write(self._ffmt(0, 22))
                f.seek(self._info_anchor_end)
                f.write(self._ffmt(self._rel_ns if self._rel_ns else 0, 22))
                f.seek(self._info_anchor_total)
                f.write(self._ffmt(self._seq if self._seq else 0, 12))
        except OSError as e:
            print(f"\033[33m[Recorder] Finalize metadata error: {e}\033[0m")

        print(f"[Recorder] Done: {self._seq} points -> {self._output_path}")

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def start(self, output_path=os.path.join(SCRIPT_DIR, "trajectory.json"), samp_hz=None):
        """开始记录：打开文件 + 写 JSON 头部 + 启动键盘线程

        Args:
            output_path: 输出文件路径，None 使用初始化时的路径
            samp_hz: 采样频率 (Hz)，写入 JSON metadata
        """
        if self._f is not None:
            return

        if output_path is not None:
            self._output_path = output_path

        self._seq = 0
        self._last_abs_ns = None
        self._rel_ns = 0

        self._f = open(self._output_path, "wb")
        self._write_header(samp_hz)
        self._f.flush()
        print(f"[Recorder] Recording to {self._output_path}")

        # 启动后台键盘监听线程
        self._running = True
        self._thread = threading.Thread(target=self._input_loop, daemon=True)
        self._thread.start()
        print("[Recorder] Press 'r' to record, 'c' to clear.")

    def stop(self):
        """停止键盘监听线程 + 闭合 JSON 文件"""
        self._running = False
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        self._thread = None
        self._finalize()

    def check_and_record(self, robot):
        """在控制循环中调用，检查是否有按键记录或清空请求"""
        if self._get_record_flag():
            self._set_record_flag(False)
            try:
                self._do_record(robot)
            except Exception as e:
                print(f"\033[33m[Recorder] record error: {e}\033[0m")
        if self._get_clear_flag():
            self._set_clear_flag(False)
            try:
                self._clear()
            except Exception as e:
                print(f"\033[33m[Recorder] clear error: {e}\033[0m")

    def record(self, robot):
        """立即记录当前机械臂状态"""
        self._do_record(robot)

    # ------------------------------------------------------------------
    # Internal
    # ------------------------------------------------------------------

    def _input_loop(self):
        """Thread: 键盘输入监听"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)
            while self._running:
                try:
                    ch = sys.stdin.read(1)
                except (EOFError, OSError):
                    break
                if not self._running or not ch:
                    break
                ch = ch.lower()
                if ch == "r":
                    self._set_record_flag(True)
                    print("[Recorder] Record triggered!")
                elif ch == "c":
                    self._set_clear_flag(True)
                    print("[Recorder] Clear triggered!")
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def _do_record(self, robot):
        """记录当前帧的机械臂状态，追加到文件"""
        state = robot.get_arm_state()
        if state is None:
            print("[Recorder] No arm state available, skipping.")
            return

        ts_ns = _ts_to_ns(state.header.stamp)
        dec = self._dec

        # 纯计算，不碰状态
        if self._last_abs_ns is None:
            new_rel_ns = 0  # 首点 = 0
        else:
            new_rel_ns = self._rel_ns + (ts_ns - self._last_abs_ns)  # 累加时间差
        idx = self._seq + 1  # 1-based 序号

        # 获取夹爪状态（兼容无夹爪的 robot 类型）
        grip_pos = []
        get_grip = getattr(robot, 'get_grip_state', None)
        if get_grip is not None:
            grip_state = get_grip()
            if grip_state is not None:
                grip_pos = [round(float(v), dec)
                            for v in grip_state.grip_state.jnt.position]

        point = {
            "ts_ns": new_rel_ns,
            "arm": [round(float(v), dec) for v in state.arm_state.jnt.position],
            "grip": grip_pos,
        }

        # I/O —— 失败则 return
        try:
            line = json.dumps(point, ensure_ascii=False)
            self._f.write(f'    "{idx}": {line},\n'.encode())
            self._f.flush()
        except OSError as e:
            print(f"\033[33m[Recorder] Write error: {e}\033[0m")
            return

        # I/O 成功后才 commit 状态
        self._last_abs_ns = ts_ns
        self._rel_ns = new_rel_ns
        self._seq += 1
        print(f"[Recorder] Recorded point {idx}")

    def _clear(self):
        """清空：finalize 当前文件，重新打开写新头"""
        # 关闭当前文件（用 _rel_ns / _seq 写入 metadata）
        try:
            self._finalize()
        except Exception as e:
            print(f"\033[33m[Recorder] Clear finalize error: {e}\033[0m")

        # 重置
        self._seq = 0
        self._last_abs_ns = None
        self._rel_ns = 0

        # 新建文件
        try:
            self._f = open(self._output_path, "wb")
            self._write_header()
            self._f.flush()
        except OSError as e:
            print(f"\033[33m[Recorder] Clear open error: {e}\033[0m")
            self._f = None
            return
        print("[Recorder] Cleared all points.")

    def _get_record_flag(self):
        r = None
        with self._lock:
            r = self._record_flag
        return r

    def _get_clear_flag(self):
        r = None
        with self._lock:
            r = self._clear_flag
        return r
    
    def _set_record_flag(self, flag):
        with self._lock: 
            self._record_flag = flag
        
    def _set_clear_flag(self, flag):
        with self._lock:
            self._clear_flag = flag