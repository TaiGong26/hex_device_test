import time
from typing import Any, Dict, List, Optional


def _now_str() -> str:
    t = time.time()
    return f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t))}.{int((t % 1) * 1_000_000):06d}"

class StateChecker:
    """状态检查器

    生命周期：
      checker = StateChecker()           # __init__ 记录 task_start_time
      checker.update(data_dict)          # 每 tick 调用（温度 min/max）
      checker.update_error(...)          # 维护错误历史列表（永久去重）
      checker.is_error()                 # 判断错误历史队列是否有错误
      checker.get_summary()              # 累计汇总
      checker.mark_task_end()            # 任务结束时标记
    """

    def __init__(self, max_error_history: int = 100):
        self._task_start_time: str = _now_str()
        self._task_start_ts: float = time.time()   # 任务启动时刻（float 秒，供 run_time 计算）
        self._max_error_history = max_error_history

        # 错误历史：元素 {"source", "motor", "error", "time"}；_error_seen 做永久去重
        self._error_history: List[Dict] = []
        self._error_seen: set = set()
        self._summary: Dict[str, Any] = self._empty_summary()

    # ── 公共接口 ──

    def update(self, data: Dict[str, Any]) -> None:
        """处理一次新的设备状态数据

        Args:
            data: 原始数据字典，支持以下字段（均带默认值）：
                motor_positions     : Optional[list | np.ndarray]
                motor_temps         : Optional[list | np.ndarray]
                driver_temps        : Optional[list | np.ndarray]
                motor_error_codes   : Optional[list | np.ndarray]
                robot_mode          : str       # 默认 ""
                overtoken_mode      : str       # 默认 ""
                overtoken_reason    : str       # 默认 ""
                conn_lost           : bool      # 默认 False
                api_exit            : bool      # 默认 False
        """
        
        now_str = _now_str()
        # 更新温度 min/max
        self._update_temp_summary(data.get("motor_temps"), now_str,
                                   "motor_temp_min", "motor_temp_max",
                                   "motor_temp_min_time", "motor_temp_max_time")
        self._update_temp_summary(data.get("driver_temps"), now_str,
                                   "driver_temp_min", "driver_temp_max",
                                   "driver_temp_min_time", "driver_temp_max_time")

    def is_error(self) -> bool:
        """直接判断错误历史队列中是否有错误"""
        return len(self._error_history) > 0

    def update_error(
        self,
        arm_err: Optional[List[List[str]]] = None,
        grip_err: Optional[List[List[str]]] = None,
        robot_mode: Optional[str] = None,
    ) -> None:
        """维护错误历史列表（永久去重，每电机每错误仅记一次）

        Args:
            arm_err     : Optional[List[List[str]]] — arm 各电机错误枚举名称列表（None 不处理）
            grip_err    : Optional[List[List[str]]] — grip 各电机错误枚举名称列表（None 不处理）
            robot_mode  : Optional[str]             — "RmXxx" 字符串；只记录
                          "RmFatalError"/"RmOvertaken"，其余忽略
        """
        if arm_err is not None:
            for i, errs in enumerate(arm_err):
                for name in errs:
                    self._append_error("arm", i, name)

        if grip_err is not None:
            for i, errs in enumerate(grip_err):
                for name in errs:
                    self._append_error("grip", i, name)

        if robot_mode in ("RmFatalError", "RmOvertaken"):
            self._append_error("robot", -1, robot_mode)

    def _append_error(self, source: str, motor: int, error: str) -> None:
        """追加一条错误到历史（永久去重：同一 (source, motor, error) 仅记一次）"""
        key = (source, motor, error)
        if key in self._error_seen:
            return
        self._error_seen.add(key)
        self._error_history.append({
            "source": source,
            "motor": motor,
            "error": error,
            "time": _now_str(),
        })


    def get_summary(self) -> Dict[str, Any]:
        """返回累计汇总统计（浅拷贝）

        键名即 controller 上报给 CSV 的基准：温度沿用内部
        motor_temp_min/max、driver_temp_min/max，另增 run_time（HH:MM:SS）。
        """
        summary = dict(self._summary).copy()
        # 任务运行时长（HH:MM:SS），供 CsvLogger 的 data["run_time"]
        summary["run_time"] = self._fmt_run_time(time.time() - self._task_start_ts)
        # errors（供 CsvLogger 的 " | ".join(list(data["errors"]))）
        summary["errors"] = [f"{e['error']}@{e['time']}" for e in self._error_history]
        return summary

    @staticmethod
    def _fmt_run_time(seconds: float) -> str:
        """将运行秒数格式化为 HH:MM:SS 字符串"""
        total = max(0, int(seconds))
        h, rem = divmod(total, 3600)
        m, s = divmod(rem, 60)
        return f"{h:02d}:{m:02d}:{s:02d}"

    def mark_task_end(self) -> None:
        """标记任务结束时间"""
        now = _now_str()
        self._summary["task_end_time"] = now

    # ── 私有 ──

    def _update_temp_summary(self, temps, now_str,
                              min_key, max_key, min_time_key, max_time_key):
        """更新单组温度（motor 或 driver）的 min/max

        内部用 list 存储，兼容 list 和 np.ndarray 两种输入。
        TODO(arm_wrapper np 改造后)：使用 np.minimum / np.maximum 替代逐元素循环。
        """
        if temps is None or not hasattr(temps, '__iter__'):
            return
        # 统一转为 float list（兼容 list 和 np.ndarray）
        vals = [float(v) for v in temps if v is not None]
        if not vals:
            return

        cur_min = self._summary.get(min_key)
        if cur_min is None:
            # 首次更新：全量初始化
            self._summary[min_key] = list(vals)
            self._summary[max_key] = list(vals)
            self._summary[min_time_key] = now_str
            self._summary[max_time_key] = now_str
        else:
            # 逐电机比较
            changed_min = changed_max = False
            cur_max = self._summary[max_key]
            for i, v in enumerate(vals):
                if i < len(cur_min):
                    if v < cur_min[i]:
                        cur_min[i] = v
                        changed_min = True
                    if v > cur_max[i]:
                        cur_max[i] = v
                        changed_max = True
            if changed_min:
                self._summary[min_time_key] = now_str
            if changed_max:
                self._summary[max_time_key] = now_str

    @staticmethod
    def _format_error(error_type, message: str) -> str:
        """格式化错误信息（占位）"""
        return f"[{error_type}] {message}"

    # ── 初始值工厂 ──

    def _empty_summary(self) -> Dict[str, Any]:
        return {
            "task_start_time": self._task_start_time,
            "task_end_time": None,
            "motor_temp_min": None,
            "motor_temp_max": None,
            "motor_temp_min_time": None,
            "motor_temp_max_time": None,
            "driver_temp_min": None,
            "driver_temp_max": None,
            "driver_temp_min_time": None,
            "driver_temp_max_time": None,
            "error_history": [],
            "errors": [],
        }
