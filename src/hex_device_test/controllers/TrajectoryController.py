import time
import numpy as np

DEFAULT_SEGMENT_DURATION = 2.5   # 迁自 tools/trajectory_loader.py（旧格式加载器已删除）


class TrajectoryPlanner:
    """Trajectory planner that supports smooth acceleration and deceleration planning"""
    
    def __init__(self, waypoints, segment_duration=3.0, interpolate='linear'):
        """
        Initialize trajectory planner
        waypoints: List of waypoints
        segment_duration: Duration of each trajectory segment (seconds)
        interpolate: Interpolation mode — 's_curve' | 'linear'（默认 linear）
        """
        self.waypoints = waypoints
        self.segment_duration = segment_duration
        self.interpolate = interpolate
        
        self.current_waypoint_index = 0
        self.trajectory_started = False
        self.start_time = None
        self.last_target_position = None  # Store last commanded position
        
    def start_trajectory(self):
        """Start trajectory execution"""
        if not self.waypoints:
            return False
        
        self.trajectory_started = True
        self.start_time = time.time()
        self.current_waypoint_index = 0
        return True
        
    def get_current_target(self):
        """Get the target position at the current moment"""
        if not self.trajectory_started or not self.waypoints:
            return None
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        total_segments = len(self.waypoints)
        segment_index = int(elapsed_time / self.segment_duration) % total_segments
        
        segment_elapsed = elapsed_time % self.segment_duration
        normalized_time = segment_elapsed / self.segment_duration
        
        start_waypoint = self.waypoints[segment_index]
        end_waypoint = self.waypoints[(segment_index + 1) % total_segments]

        start_pos = np.array(start_waypoint)
        end_pos = np.array(end_waypoint)

        if self.interpolate == 'linear':
            # 线性插值——匀速（ROS replay 包 _get_linear_position 公式）
            target_position = start_pos + (end_pos - start_pos) * normalized_time
        else:  # 's_curve'
            # S 曲线插值——smoothstep，端点速度/加速度为零
            s = self._smooth_step(normalized_time)
            target_position = start_pos + s * (end_pos - start_pos)

        self.current_waypoint_index = segment_index
        self.last_target_position = target_position  # Store for potential return home

        return target_position
    
    def get_last_position(self):
        """Get the last commanded position"""
        return self.last_target_position
        
    def _smooth_step(self, t):
        """S-curve interpolation function that provides smooth acceleration and deceleration"""
        # Limit t to [0,1] range
        t = max(0.0, min(1.0, t))
        
        # Use 5th degree polynomial for smoother interpolation: 6t⁵ - 15t⁴ + 10t³
        return 6 * t**5 - 15 * t**4 + 10 * t**3
        
    def get_current_segment_info(self):
        """Get information about the current segment"""
        if not self.trajectory_started:
            return None
            
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        segment_index = int(elapsed_time / self.segment_duration) % len(self.waypoints)
        segment_elapsed = elapsed_time % self.segment_duration
        segment_progress = segment_elapsed / self.segment_duration
        
        return {
            'segment_index': segment_index,
            'segment_progress': segment_progress,
            'total_elapsed': elapsed_time
        }

class ReturnHomeController:
    """Controller for smooth return to home position"""
    
    def __init__(self, start_position, home_position, duration):
        """
        Initialize return home controller
        start_position: Starting position (current position when Ctrl+C is pressed)
        home_position: Target home position
        duration: Duration to reach home position (seconds)
        """
        self.start_position = np.array(start_position)
        self.home_position = np.array(home_position)
        self.duration = duration
        self.start_time = time.time()
        self.done = False
        
    def get_target_position(self):
        """Get the current target position during return home"""
        current_time = time.time()
        elapsed_time = current_time - self.start_time
        
        if np.allclose(self.start_position,self.home_position):
            self.done = True
        
        if elapsed_time >= self.duration or self.done:
            return self.home_position, True  # Reached home
        
        # if self.done:
        #     return self.home_position, True  # Reached home
        
        # Calculate normalized time [0, 1]
        t = elapsed_time / self.duration
        
        # Use S-curve interpolation for smooth motion
        s = self._smooth_step(t)
        
        # Interpolate between start and home position
        target_position = self.start_position + s * (self.home_position - self.start_position)
        
        return target_position, False  # Not yet reached home
    
    def _smooth_step(self, t):
        """S-curve interpolation function"""
        t = max(0.0, min(1.0, t))
        return 6 * t**5 - 15 * t**4 + 10 * t**3


class TimestampsTrajectoryPlanner:
    """录制轨迹回放规划器——按录制时间戳驱动（参考 ROS hex_ros_demo_arm_replay）。

    waypoints:   List[List[float]] 6DOF 关节位置
    timestamps:  List[float] 相对秒, ts[0]=0.0（来自 PointLoader.get_timestamps）
    interpolate: 's_curve' | 'linear'（默认 'linear'）
    loop:        True=循环回放（耐久性）。循环边界在末点→首点之间补一段
                 插值过渡（时长=平均段时长），避免硬跳导致机械臂过冲。
    """

    def __init__(self, waypoints, timestamps, interpolate='linear', loop=True):
        n = len(waypoints)
        assert n >= 2 and len(timestamps) == n, \
            "waypoints and timestamps must have same length >= 2"
        self._waypoints = [np.array(w, dtype=float) for w in waypoints]
        self._timestamps = [float(t) for t in timestamps]
        self._interpolate = interpolate
        self._loop = loop

        if loop:
            # 循环 wrap 段：末点 → 首点，时长 = 平均段时长（可构造参数覆盖）
            wrap_dur = float(np.mean(np.diff(self._timestamps)))
            self._cycle_wps = self._waypoints + [self._waypoints[0]]
            self._cycle_ts = self._timestamps + [self._timestamps[-1] + wrap_dur]
        else:
            self._cycle_wps = self._waypoints
            self._cycle_ts = self._timestamps
        self._cycle_len = self._cycle_ts[-1]

        self._trajectory_started = False
        self._start_time = None
        self._current_waypoint_index = 0
        self._last_target_position = None

    def start_trajectory(self):
        if not self._cycle_wps:
            return False
        self._trajectory_started = True
        self._start_time = time.time()
        self._current_waypoint_index = 0
        return True

    def get_current_target(self):
        if not self._trajectory_started:
            return None

        elapsed = time.time() - self._start_time
        if self._loop:
            t = elapsed % self._cycle_len  # 循环回绕
        else:
            if elapsed >= self._cycle_len:
                return self._waypoints[-1]  # 单次播放：保持末点
            t = elapsed

        idx = int(np.searchsorted(self._cycle_ts, t, side='right') - 1)
        idx = max(0, min(idx, len(self._cycle_wps) - 2))

        s_elapsed = t - self._cycle_ts[idx]
        s_dur = self._cycle_ts[idx + 1] - self._cycle_ts[idx]
        start_pos, end_pos = self._cycle_wps[idx], self._cycle_wps[idx + 1]

        if s_dur <= 0:
            target = start_pos  # 相邻时间戳相同：保持起点
        elif self._interpolate == 'linear':
            target = start_pos + (end_pos - start_pos) * (s_elapsed / s_dur)
        else:  # 's_curve'
            s = self._smooth_step(s_elapsed / s_dur)
            target = start_pos + s * (end_pos - start_pos)

        self._current_waypoint_index = idx % len(self._waypoints)  # 回绕给 loop_count
        self._last_target_position = target
        return target

    def get_last_position(self):
        return self._last_target_position

    def get_current_segment_info(self):
        if not self._trajectory_started:
            return None
        elapsed = time.time() - self._start_time
        if self._loop:
            total_elapsed = elapsed % self._cycle_len
        else:
            total_elapsed = min(elapsed, self._cycle_len)
        idx = int(np.searchsorted(self._cycle_ts, total_elapsed, side='right') - 1)
        idx = max(0, min(idx, len(self._cycle_wps) - 2))
        s_elapsed = total_elapsed - self._cycle_ts[idx]
        s_dur = self._cycle_ts[idx + 1] - self._cycle_ts[idx]
        return {
            'segment_index': idx % len(self._waypoints),
            'segment_progress': s_elapsed / s_dur if s_dur > 0 else 0.0,
            'total_elapsed': elapsed,
        }

    def _smooth_step(self, t):
        t = max(0.0, min(1.0, t))
        return 6 * t**5 - 15 * t**4 + 10 * t**3

    # ###TODO: 夹爪（grip_waypoints）插值回放——第 3 轮范围外，后续接入
