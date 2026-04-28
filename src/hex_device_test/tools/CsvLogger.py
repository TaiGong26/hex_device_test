from queue import Queue
import os
import csv
import time
from typing import List, Dict

from ..statuses.ArmStatus import ArmControllerStatus, ArmErrorStatus

"""
表格对其查看
column -s, -t ~/hex_device_log/arm_test_xxx.csv | less

"""
def write_csv(mp_queue:Queue, file_path):
    file_path = os.path.expanduser(file_path)
    os.makedirs(os.path.dirname(file_path), exist_ok=True)
    
    with open(file_path, 'w', newline='', encoding='utf-8') as f:
        writer = None

        while not mp_queue.empty():
            info = mp_queue.get()

            for device_id, data in info.items():
                row = {
                    "run_time": data["run_time"],
                    "device_id": device_id,
                    "state": data["state"],
                    "loop_counter": data["loop_counter"],
                }

                # 展开电机温度
                motors = data["motor_max_temperature"] or []
                for i, v in enumerate(motors):
                    row[f"motor_{i}"] = v

                # 展开驱动温度
                drivers = data["motor_driver_max_temperature"] or []
                for i, v in enumerate(drivers):
                    row[f"driver_{i}"] = v

                # errors（转字符串）
                row["errors"] = " | ".join(list(data["errors"]))

                # 初始化 writer
                if writer is None:
                    writer = csv.DictWriter(f, fieldnames=row.keys())
                    writer.writeheader()

                writer.writerow(row)
