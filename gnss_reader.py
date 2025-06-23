import queue
import threading
import time
from datetime import datetime
from dronekit import connect, VehicleMode, LocationGlobalRelative

class GNSSReader:
    def __init__(self, connection_string):
        self.connection_string = connection_string
        self.data_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = None
        self._vehicle = None

    def start(self):
        if self._thread and self._thread.is_alive():
            return

        self._thread = threading.Thread(target=self._read_loop)
        self._thread.daemon = True
        self._thread.start()

    def stop(self):
        self._stop_event.set()
        if self._thread:
            self._thread.join(timeout=1.0)

        if self._vehicle:
            self._vehicle.close()
            self._vehicle = None

    def _read_loop(self):
        try:
            self._vehicle = connect(self.connection_string, wait_ready=True, timeout=60)
            print(f"GNSS已启动 - {self.connection_string}")

            while not self._stop_event.is_set():
                try:
                    gps = self._vehicle.gps_0

                    # 尝试获取GPS时间
                    gps_time = datetime.now()  # 默认使用系统时间

                    gnss_data = {
                        'timestamp': gps_time,
                        'fix_type': gps.fix_type,
                        'satellites': gps.satellites_visible,
                        'location': self._vehicle.location.global_frame,
                        'altitude': self._vehicle.location.global_relative_frame.alt,
                        'velocity': self._vehicle.velocity
                    }

                    self.data_queue.put(gnss_data)
                    time.sleep(0.1)  # 10Hz采样率

                except Exception as e:
                    print(f"读取GNSS数据失败: {e}")
                    time.sleep(1)

        except Exception as e:
            print(f"GNSS连接错误: {e}")

    def get_data(self, block=True, timeout=None):
        """从队列获取数据"""
        try:
            return self.data_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None

if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='单独测试GNSS读取')
    parser.add_argument('--connection_string', required=True, help='GNSS串口号')
    args = argparse.Namespace(connection_string='COM8')
    # args = parser.parse_args()

    gnss = GNSSReader(args.connection_string)
    gnss.start()

    print(f"开始测试GNSS读取 - {args.connection_string}")
    print("按 Ctrl+C 停止")

    try:
        while True:
            data = gnss.get_data(block=False)
            if data:
                timestamp = data['timestamp'].strftime("%H:%M:%S.%f")[:-3]
                print(f"[{timestamp}] 位置: {data['location'].lat:.6f}, {data['location'].lon:.6f}, {data['altitude']:.2f}m | 定位: {data['fix_type']} | 卫星: {data['satellites']}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n测试已停止")
    finally:
        gnss.stop()