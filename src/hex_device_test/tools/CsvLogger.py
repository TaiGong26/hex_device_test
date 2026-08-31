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
                    "start_time": data["start_time"],
                    "run_time": data["run_time"],
                    "device_id": device_id,
                    "device": data["device"],
                    # "state": data["state"],
                    "loop_counter": data["loop_counter"],
                }

                # 展开电机最高温（controller 键 motor_temp_max）
                motors = data["motor_temp_max"] or []
                for i, v in enumerate(motors):
                    row[f"MAX_motor_{i}_temp"] = v

                # 展开电机最低温（controller 键 motor_temp_min）
                m_mins = data["motor_temp_min"] or []
                for i, v in enumerate(m_mins):
                    row[f"MIN_motor_{i}_temp"] = v

                # 展开驱动最高温（controller 键 driver_temp_max）
                drivers = data["driver_temp_max"] or []
                for i, v in enumerate(drivers):
                    row[f"MAX_driver_{i}_temp"] = v

                # 展开驱动最低温（controller 键 driver_temp_min）
                d_mins = data["driver_temp_min"] or []
                for i, v in enumerate(d_mins):
                    row[f"MIN_driver_{i}_temp"] = v

                # errors（转字符串）
                row["errors"] = " | ".join(list(data["errors"]))

                # 初始化 writer
                if writer is None:
                    writer = csv.DictWriter(f, fieldnames=row.keys())
                    writer.writeheader()

                writer.writerow(row)
