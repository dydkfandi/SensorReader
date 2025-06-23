import serial
import queue
import threading
import time
import struct
from datetime import datetime

FDLINK_HEADER = b'\xFC'
FDLINK_FOOTER = b'\xFD'

class IMUReader:
    def __init__(self, port, baudrate=921600):
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
                    while len(buffer) >= 8:  # 最小帧长度为8字节（帧头 + 指令类别 + 数据长度 + 流水序号 + 帧头CRC8 + 数据CRC16 + 帧尾）
                        if buffer[0] != FDLINK_HEADER[0]:
                            buffer = buffer[1:]
                            continue

                        # 提取数据长度
                        frame_length = buffer[2]  # 数据长度字段在第三个字节
                        total_frame_size = frame_length + 8  # 总帧长度 = 数据长度 + 8字节的固定部分

                        if len(buffer) >= total_frame_size:
                            if buffer[total_frame_size - 1] == FDLINK_FOOTER[0]:  # 检查帧尾
                                if buffer[1] == 0xF0:
                                    buffer = buffer[total_frame_size:]
                                    continue
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
            # 提取各字段
            header = frame[0]  # 帧头
            command = frame[1]  # 指令类别
            data_length = frame[2]  # 数据长度
            sequence = frame[3]  # 流水序号
            crc8 = frame[4]  # 帧头CRC8
            crc16 = frame[5:7]  # 数据CRC16
            imu_data = frame[7:-1]  # 数据部分
            footer = frame[-1]  # 帧尾

            # 解析数据部分
            if len(imu_data) >= 56:
                float_part = imu_data[:48]
                int_part = imu_data[-8:]
                float_values =  struct.unpack('<12f', float_part)
                int_values = struct.unpack('<q', int_part)[0]

                gx = float_values[0]
                gy = float_values[1]
                gz = float_values[2]

                ax = float_values[3]
                ay = float_values[4]
                az = float_values[5]

                mx = float_values[6]
                my = float_values[7]
                mz = float_values[8]

                imu_temperature = float_values[9]
                pressure = float_values[10]
                pressure_temperature = float_values[11]

                imu_time = int_values/1000/1000

                return {
                    'timestamp': datetime.now(),
                    'acc': (ax, ay, az),
                    'gyro': (gx, gy, gz),
                    'mag':(mx, my, mz),
                    'imu_temperature': imu_temperature,
                    'pressure': pressure,
                    'pressure_temperature': pressure_temperature,
                    'imu_time':imu_time,
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


if __name__ == "__main__":
    import argparse

    parser = argparse.ArgumentParser(description='单独测试IMU读取')
    parser.add_argument('--port', required=True, help='IMU串口号')
    parser.add_argument('--baudrate', type=int, default=115200, help='IMU波特率')
    # args = parser.parse_args()
    args = argparse.Namespace(port='COM5', baudrate=921600)

    imu = IMUReader(args.port, args.baudrate)
    imu.start()

    print(f"开始测试IMU读取 - {args.port} [{args.baudrate}]")
    print("按 Ctrl+C 停止")

    try:
        while True:
            data = imu.get_data(block=False)
            if data:
                timestamp = data['timestamp'].strftime("%H:%M:%S.%f")[:-3]
                acc = data['acceleration']
                gyro = data['gyro']
                mag = data['magnetometer']
                imu_temp = data['imu_temperature']
                press = data['pressure']
                press_tem = data['pressure_temperature']
                imu_time = data['imu_time']
                frame = data['raw_data']
                print(f"[{timestamp}]"
                      f"加速度: {acc[0]:f}, {acc[1]:f}, {acc[2]:f} | "
                      f"角速度: {gyro[0]:f}, {gyro[1]:f}, {gyro[2]:f} | "
                      f"磁感应强度: {mag[0]:f}, {mag[1]:f}, {mag[2]:f} |"
                      f"IMU温度：{imu_temp:f} | 上电时间： {imu_time:f}")
                # print(frame.hex())
            time.sleep(0.01)
    except KeyboardInterrupt:
        print("\n测试已停止")
    finally:
        imu.stop()