import argparse
import socket
import json
from ipaddress import ip_address
import threading
import time
# from queue import Queue
from collections import deque

class PlotjuggleDraw:
    def __init__(self, address: str = "127.0.0.1", port: int = 9870):
        self.address = address
        self.port = port
        self._running = False
        self._thread = None
        self._socket = None
        
        self._data_queue: deque = deque(maxlen=100)
        
        # lock
        self._data_lock = threading.Lock()
        
        
        # 解析 IP 地址以确定使用 IPv4 还是 IPv6
        try:
            addr_obj = ip_address(self.address)
            self.family = socket.AF_INET6 if addr_obj.version == 6 else socket.AF_INET
            print(f"[UDPSender] 初始化: IPv{addr_obj.version} {self.address}:{self.port}")
            self._socket = socket.socket(self.family, socket.SOCK_DGRAM)
        except ValueError:
            print(f"[UDPSender] 警告: 无效的 IP 地址 '{self.address}'，默认使用 IPv4")
            self.family = socket.AF_INET

    def start(self):
        """启动发送线程"""
        if self._running:
            print("[Plotjuggle] threading is running")
            return

        self._running = True
        self._thread = threading.Thread(target=self._run_task_loop, daemon=True)
        self._thread.start()
        print("[Plotjuggle] start threading")

    def stop(self):
        """停止发送线程"""
        self._running = False
        if self._socket:
            self._socket.close() # 关闭 socket 以打断阻塞
        if self._thread and self._thread.is_alive():
            self._thread.join(timeout=1.0)
        print("[Plotjuggle] end threading")

    def send_data(self, data):
        self._socket.sendto(
                    data, 
                    (self.address, self.port)
                )
    def send_json(self, data_dict):
        with self._data_lock:
            self._socket.sendto(
                        json.dumps(data_dict).encode(), 
                        (self.address, self.port)
                    )

    def add_data(self, data):
        """向发送队列添加数据"""
        with self._data_lock:
            self._data_queue.append(data)
        
    def _run_task_loop(self):
        """内部发送循环逻辑"""
        try:
            # 在线程内部创建 socket
            while self._running:
                # 判断队列是否有数据
                with self._data_lock:
                    if self._data_queue:
                        data_to_send = self._data_queue.popleft()
                    else:
                        data_to_send = None
                        
                # 发送数据
                if data_to_send is not None:
                    self.send_json(data_to_send)
                else:
                    time.sleep(0.05)
                
        except Exception as e:
            if self._running: # 如果是运行中报错才打印
                print(f"[UDPSender] 发送异常: {e}")
        finally:
            if self._socket:
                self._socket.close()


senders = PlotjuggleDraw()

# --- 使用示例 ---
if __name__ == "__main__":
    # 解析命令行参数
    parser = argparse.ArgumentParser(description="Send UDP test data.")
    parser.add_argument("--address", default="127.0.0.1", help="UDP address")
    parser.add_argument("--port", default=9870, type=int, help="UDP port")
    args = parser.parse_args()

    # 实例化并启动
    sender = PlotjuggleDraw()
    
    try:
        sender.start()
        # 主线程等待，防止程序退出
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        print("\n检测到中断信号...")
    finally:
        sender.stop()