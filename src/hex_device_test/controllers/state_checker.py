"""StateChecker — 设备状态聚合与检查

设计意图：
  维护 current_state（实时快照）、summary（累计统计：温度 min/max + 时间戳，
  错误历史 + 时间戳）。update() 接收原始 data dict，不绑定数据源。

TODO(arm_wrapper np 改造后)：arm_wrapper getter 改为返回 np.ndarray 后，
  state_checker 内部 vals 转换需使用 np.asarray 而非 list comprehension，
  减少逐元素迭代开销。
"""

import time
from typing import Any, Dict, List, Optional


def _now_str() -> str:
    """带微秒的系统时间字符串"""
    t = time.time()
    return f"{time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(t))}.{int((t % 1) * 1_000_000):06d}"


def _to_list(val):
    """将 np.ndarray 或其他可迭代对象转为 list，None 保持不变"""
    if val is None:
        return None
    if hasattr(val, 'tolist'):
        return val.tolist()
    if isinstance(val, (list, tuple)):
        return list(val)
    return val


class StateChecker:
    """状态检查器

    生命周期：
      checker = StateChecker()           # __init__ 记录 task_start_time
      checker.update(data_dict)          # 每 tick 调用
      checker.is_error()                 # 检查当前是否有错误
      checker.get_current_info()         # 实时状态快照
      checker.get_summary()              # 累计汇总
      checker.mark_task_end()            # 任务结束时标记
    """

    def __init__(self, max_error_history: int = 100):
        self._task_start_time: str = _now_str()
        self._max_error_history = max_error_history

        self._current_state: Dict[str, Any] = self._empty_current_state()
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

        # 刷新 current_state（统一转为 list，兼容 np.ndarray 输入）
        self._current_state["system_time"] = now_str
        self._current_state["motor_positions"] = _to_list(data.get("motor_positions"))
        self._current_state["motor_temps"] = _to_list(data.get("motor_temps"))
        self._current_state["driver_temps"] = _to_list(data.get("driver_temps"))
        self._current_state["motor_error_codes"] = _to_list(data.get("motor_error_codes"))
        self._current_state["robot_mode"] = data.get("robot_mode", "")
        self._current_state["overtoken_mode"] = data.get("overtoken_mode", "")
        self._current_state["overtoken_reason"] = data.get("overtoken_reason", "")
        self._current_state["conn_lost"] = data.get("conn_lost", False)
        self._current_state["api_exit"] = data.get("api_exit", False)
        # TODO: 待用户评估 ArmErrorStatus 枚举后，补充具体错误检测逻辑
        self._current_state["has_error"] = False
        self._current_state["errors"] = []

        # 更新温度 min/max
        self._update_temp_summary(data.get("motor_temps"), now_str,
                                   "motor_temp_min", "motor_temp_max",
                                   "motor_temp_min_time", "motor_temp_max_time")
        self._update_temp_summary(data.get("driver_temps"), now_str,
                                   "driver_temp_min", "driver_temp_max",
                                   "driver_temp_min_time", "driver_temp_max_time")

    def is_error(self) -> bool:
        """当前是否有活动错误（占位，当前始终返回 False）"""
        return self._current_state.get("has_error", False)

    def get_current_info(self) -> Dict[str, Any]:
        """返回当前状态快照（浅拷贝）"""
        return dict(self._current_state)

    def get_summary(self) -> Dict[str, Any]:
        """返回累计汇总统计（浅拷贝）"""
        return dict(self._summary)

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

    @staticmethod
    def _empty_current_state() -> Dict[str, Any]:
        return {
            "system_time": "",
            "motor_positions": None,
            "motor_temps": None,
            "driver_temps": None,
            "motor_error_codes": None,
            "robot_mode": "",
            "overtoken_mode": "",
            "overtoken_reason": "",
            "conn_lost": False,
            "api_exit": False,
            "has_error": False,
            "errors": [],
        }

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
        }
