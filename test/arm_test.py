import argparse
import traceback
import threading
import signal
import time
import sys


from dataclasses import dataclass

# from hex_device_test.managers.Coordinator import ArmCoordinator
from hex_device_test.managers.CoordinatorProcess import ArmCoordinator

# clean up
def cleanup(coordinator):
    print(f"cleanup")
    coordinator.shutdown()
    print(f"coordinator shutdown")
    coordinator._stop_event.wait()

# 信号处理回调
def signal_handler(signal, frame, stop_event: threading.Event):
    print(f"[Signal] {signal} received, exit")
    stop_event.set()
    print("----------------------------------------signal handler set stop event")
    return

def main():
    # 标准库中获取命令行参数：数组
    parser = argparse.ArgumentParser(
        description='Hexapod robotic arm trajectory planning and execution test',
        formatter_class=argparse.RawTextHelpFormatter
    )

    parser.add_argument(
        '--url', 
        metavar='URL',
        nargs="+",
        # default="ws://0.0.0.0:8439",
        default=["ws://0.0.0.0:8439"],
        help='WebSocket URL for HEX device connection, example: ws://0.0.0.0:8439 or ws://[::1%%eth0]:8439'
    )
    
    parser.add_argument(
        '--KCP', 
        action='store_true',
        default=False,
        help='Enable KCP protocol for HEX device connection'
    )
    
    parser.add_argument(
        '--view',
        action='store_true',
        default=False,
        help='Enable real-time visualization of the robotic arm trajectory'
    )
    
    parser.add_argument(
        '--timeout',
        action='store_true',
        default=False,
        help='Enable real-time visualization of the robotic arm trajectory'
    )
    
    # =============== parse args ===============
    args = parser.parse_args()
    
    dev_ip_list = args.url
    if dev_ip_list is None or len(dev_ip_list) == 0:
        print("Error: No device IPs provided. Please specify at least one device IP using the --url argument.")
        # sys.exit(1)
        return
    print(f"Device IP list: {dev_ip_list}")
    enable_kcp = args.KCP
    enable_view = args.view
    check_timeout = args.timeout
    # config
    config_dict = {
        'name':'Archer_d6y',
        'dof_num': 'six_axis',
        'motor_model': [0x80] * 6,
        'joints': [{
            'joint_name': 'joint_1',
            'joint_limit': [-2.7, 2.7, -0.1, 0.1, 0.0, 0.0]
        }, {
            'joint_name': 'joint_2',
            'joint_limit': [-1.57, 2.094, -0.5, 0.5, 0.0, 0.0]
        }, {
            'joint_name': 'joint_3',
            'joint_limit': [0.0, 3.14159265359, -0.5, 0.5, 0.0, 0.0]
        }, {
            'joint_name': 'joint_4',
            'joint_limit': [-1.5, 1.5, -0.5, 0.5, 0.0, 0.0]
        }, {
            'joint_name': 'joint_5',
            'joint_limit': [-1.56, 1.56, -0.5, 0.5, 0.0, 0.0]
        }, {
            'joint_name': 'joint_6',
            'joint_limit': [-1.57, 1.57, -0.5, 0.5, 0.0, 0.0]
        }]
    }
    
    arm_position = [
        [0.0, 0.223598775598, 0.0, 0.0, 0.0, 0.0],
        [0.5, 0.623598775598, 1.59439265359, 1.57, -1.0472, 0.0],
        [0.0, 0.223598775598, 0.0, 0.0, 0.0, 0.0],
        [-0.5, 0.623598775598, 1.59439265359, -1.57, 1.0472, 0.0]
    ]
        
    stop_event = threading.Event()
    coordinator = None
    try:
        coordinator = ArmCoordinator(
            dev_ip_list, 
            enable_kcp, 
            arm_config=config_dict,
            waypoints=arm_position,
            enable_view=enable_view,
            check_timeout=check_timeout
        )
        
        # 信号处理
        signal.signal(signal.SIGINT, lambda signal, frame: signal_handler(signal, frame, stop_event))
        signal.signal(signal.SIGTERM, lambda signal, frame: signal_handler(signal, frame, stop_event))
        
        stop_event.wait()   

    except KeyboardInterrupt:
        print("keyboard interrupt")
        
    except Exception as e:
        print(f"main error: {e}")
        traceback.print_exc()
    finally:
        # 恢复默认信号处理
        signal.signal(signal.SIGINT, signal.SIG_DFL)
        signal.signal(signal.SIGTERM, signal.SIG_DFL)
        
        if coordinator is not None:
            cleanup(coordinator)
        print("[finally] you can try a gain ctrl c to exit the terminal")
        sys.exit(0)

def t_thread(NUM):
    print(f"thread{NUM} start")
    time.sleep(1)
    print(f"thread{NUM} end")
    
if __name__ == "__main__":
    main()
    # test()