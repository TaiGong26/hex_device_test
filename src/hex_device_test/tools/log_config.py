"""hex_device_test 日志配置 —— 根 StreamHandler + 彩色 Formatter（标准库 logging，无第三方依赖）。

入口（test/arm_test_v2.py）在构造 ArmCoordinator 前调用 setup_logging()；
子进程（ArmControllerMpV2.run）fork 继承 handler，无需重复配置（setup_logging 幂等兜底）。
"""
import logging
import sys

_ANSI = {
    "RED": "\033[91m",
    "YELLOW": "\033[93m",
    "RESET": "\033[0m",
}
_FORMAT = "%(asctime)s [%(name)s] %(levelname)s %(message)s"
# 不传 datefmt —— 默认 default_msec_format 生成 "%Y-%m-%d %H:%M:%S,mmm"（含毫秒）


class ColorFormatter(logging.Formatter):
    """ERROR 整行红、WARNING 整行黄，其余默认色；TTY 才着色（管道/文件输出纯净文本）。"""

    def __init__(self, fmt: str = _FORMAT):
        super().__init__(fmt=fmt)
        self._use_color = False
        try:
            # 构造时缓存一次，避免每记录一次 syscall
            self._use_color = sys.stdout.isatty()
        except Exception:
            self._use_color = False

    def format(self, record: logging.LogRecord) -> str:
        if not self._use_color:
            return super().format(record)
        if record.levelno >= logging.ERROR:
            return f"{_ANSI['RED']}{super().format(record)}{_ANSI['RESET']}"
        if record.levelno >= logging.WARNING:
            return f"{_ANSI['YELLOW']}{super().format(record)}{_ANSI['RESET']}"
        return super().format(record)


def setup_logging(level: int = logging.INFO) -> None:
    """配置根 logger：一个写 sys.stdout 的彩色 StreamHandler。

    幂等：以 marker 判断本模块是否已安装过 handler，避免重复；
    fork 子进程调用时（run() 入口）因 handler 已继承而直接返回。
    """
    root = logging.getLogger()
    for h in root.handlers:
        if getattr(h, "_hexdev_formatter", None) is not None:
            return

    handler = logging.StreamHandler(sys.stdout)
    handler.setFormatter(ColorFormatter())
    handler.setLevel(level)
    handler._hexdev_formatter = True          # 幂等 marker（随 fork 继承）
    root.addHandler(handler)
    root.setLevel(level)                       # 关键：root 默认 WARNING，不设置则 INFO 全被丢弃
