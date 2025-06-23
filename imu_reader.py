import serial
import queue
import threading
from datetime import datetime

FDLINK_HEADER = b'\xFC'
FDLINK_FOOTER = b'\xFD'

class IMUReader:
    def __init__(self, port, baudrate=115200):
        self.port = port
        self.baudrate = baudrate
        self.data_queue = queue.Queue()
        self._stop_event = threading.Event()
        self._thread = None

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

    def _read_loop(self):
        try:
            with serial.Serial(self.port, self.baudrate, timeout=1) as ser:
                print(f"IMU已启动 - {self.port} [{self.baudrate}]")
                buffer = b''

                while not self._stop_event.is_set():
                    data = ser.read(ser.in_waiting or 1)
                    if not data:
                        continue

                    buffer += data

                    # 解析FDLink帧
                    while len(buffer) >= 3:
                        if buffer[0] != FDLINK_HEADER[0]:
                            buffer = buffer[1:]
                            continue

                        frame_length = buffer[2]
                        total_frame_size = frame_length + 4

                        if len(buffer) >= total_frame_size:
                            if buffer[total_frame_size - 1] == FDLINK_FOOTER[0]:
                                frame = buffer[:total_frame_size]
                                imu_data = self._parse_frame(frame)
                                if imu_data:
                                    self.data_queue.put(imu_data)
                                buffer = buffer[total_frame_size:]
                            else:
                                buffer = buffer[1:]
                        else:
                            break
        except Exception as e:
            print(f"IMU读取错误: {e}")

    def _parse_frame(self, frame):
        """解析FDLink格式的IMU数据"""
        try:
            data_start = 2
            data_end = len(frame) - 1
            imu_data = frame[data_start:data_end]

            if len(imu_data) >= 18:
                ax = int.from_bytes(imu_data[0:2], 'little', signed=True) / 16384.0
                ay = int.from_bytes(imu_data[2:4], 'little', signed=True) / 16384.0
                az = int.from_bytes(imu_data[4:6], 'little', signed=True) / 16384.0

                gx = int.from_bytes(imu_data[6:8], 'little', signed=True) / 131.0
                gy = int.from_bytes(imu_data[8:10], 'little', signed=True) / 131.0
                gz = int.from_bytes(imu_data[10:12], 'little', signed=True) / 131.0

                return {
                    'timestamp': datetime.now(),
                    'acceleration': (ax, ay, az),
                    'gyro': (gx, gy, gz),
                    'raw_data': frame
                }
        except Exception as e:
            print(f"解析IMU帧失败: {e}")
        return None

    def get_data(self, block=True, timeout=None):
        """从队列获取数据"""
        try:
            return self.data_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None