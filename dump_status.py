import time
from cyber.python.cyber_py3 import cyber
from modules.common_msgs.monitor_msgs.system_status_pb2 import SystemStatus

def callback(data):
    print("Received SystemStatus!")
    for name, status in data.components.items():
        print(f"Component: {name}")
        print(f"  Summary: {status.summary.status} ({status.summary.message})")
        print(f"  Process: {status.process_status.status} ({status.process_status.message})")
        print(f"  Module: {status.module_status.status} ({status.module_status.message})")
        print(f"  Channel: {status.channel_status.status} ({status.channel_status.message})")
        print(f"  Other: {status.other_status.status} ({status.other_status.message})")
    cyber.shutdown()
    exit(0)

if __name__ == "__main__":
    cyber.init()
    node = cyber.Node("status_dumper")
    node.create_reader("/apollo/monitor/system_status", SystemStatus, callback)
    print("Waiting for status...")
    while not cyber.is_shutdown():
        time.sleep(0.1)
