from queue import Empty, Queue
import os
import csv

from ..statuses.ArmStatus import ArmControllerStatus, ArmErrorStatus

"""
表格对其查看
column -s, -t ~/hex_device_log/arm_test_xxx.csv | less

"""
def write_csv(mp_queue: Queue, file_path):
    """Drain controller summaries and write a CSV with a stable union schema.

    Different devices may expose different columns (for example, a device that
    failed during initialization has no temperature values).  Build every row
    before creating DictWriter so the first queue item cannot accidentally
    define an incomplete header.
    """
    file_path = os.path.expanduser(file_path)
    directory = os.path.dirname(file_path)
    if directory:
        os.makedirs(directory, exist_ok=True)

    rows = []
    while True:
        try:
            # multiprocessing.Queue.empty() is not reliable because its feeder
            # thread may still be transferring an already-put report.
            info = mp_queue.get(timeout=0.2)
        except Empty:
            break

        for device_id, data in info.items():
            row = {
                "start_time": data.get("start_time", ""),
                "run_time": data.get("run_time", ""),
                "device_id": device_id,
                "device": data.get("device", ""),
                "loop_counter": data.get("loop_counter", 0),
            }

            for i, value in enumerate(data.get("motor_temp_max") or []):
                row[f"MAX_motor_{i}_temp"] = value
            for i, value in enumerate(data.get("motor_temp_min") or []):
                row[f"MIN_motor_{i}_temp"] = value
            for i, value in enumerate(data.get("driver_temp_max") or []):
                row[f"MAX_driver_{i}_temp"] = value
            for i, value in enumerate(data.get("driver_temp_min") or []):
                row[f"MIN_driver_{i}_temp"] = value

            row["errors"] = " | ".join(data.get("errors") or [])
            rows.append(row)

    if not rows:
        return

    base_fields = [
        "start_time",
        "run_time",
        "device_id",
        "device",
        "loop_counter",
    ]
    dynamic_fields = sorted({
        key
        for row in rows
        for key in row
        if key not in base_fields and key != "errors"
    })
    fieldnames = base_fields + dynamic_fields + ["errors"]

    with open(file_path, "w", newline="", encoding="utf-8") as csv_file:
        writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
        writer.writeheader()
        writer.writerows(rows)
