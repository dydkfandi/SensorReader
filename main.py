import argparse
import time
import os
from datetime import datetime
from sensor_factory import SensorFactory
import serial
import serial.tools.list_ports


def list_serial_ports():
    """列出所有可用串口并返回设备列表"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        print("未找到可用串口设备")
        return []

    print("\n可用串口列表:")
    port_list = []
    for i, port in enumerate(ports):
        print(f"{i + 1}. {port.device} - {port.description}")
        port_list.append(port.device)
    return port_list


def get_available_port(description="请选择串口"):
    """获取用户选择的可用串口"""
    available_ports = list_serial_ports()
    if not available_ports:
        return None

    while True:
        try:
            choice = int(input(f"\n{description} (输入序号): ")) - 1
            if 0 <= choice < len(available_ports):
                return available_ports[choice]
            print("无效选择，请重新输入")
        except (ValueError, IndexError):
            print("请输入有效的序号")


def create_output_file(sensor_type):
    """创建带时间戳的输出文件"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{sensor_type}_{timestamp}.txt"
    file = open(filename, 'w')
    file.write(f"{'=' * 60}\n")
    file.write(f"{sensor_type}数据记录\n")
    file.write(f"开始时间: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
    file.write(f"{'=' * 60}\n\n")
    return file, filename


def main():
    parser = argparse.ArgumentParser(description='多传感器数据采集系统')
    args = parser.parse_args()

    # 创建传感器列表
    sensors = []
    file_handles = {}  # 存储文件句柄

    # 添加IMU传感器（带串口选择）
    imu_port = get_available_port("请选择IMU串口")
    # 支持的常用波特率
    baudrates = [9600, 19200, 38400, 57600, 115200, 230400, 460800, 921600]
    # 确定使用的波特率
    while imu_port:
        try:
            # 获取用户输入并转换为整数
            imu_boud = int(input("请输入IMU波特率："))

            # 验证波特率是否在可用列表中
            if imu_boud in baudrates:
                imu = SensorFactory.create_sensor("imu", imu_port, imu_boud)
                sensors.append(imu)
                file_handles["imu"], imu_filename = create_output_file("IMU")
                print(f"IMU数据将保存至: {imu_filename}")
                break  # 输入正确，退出循环
            else:
                # 提示错误信息并继续循环
                print(f"无效的波特率！可用选项：{', '.join(map(str, baudrates))}")
        except ValueError:
            # 处理非整数输入的情况
            print("请输入有效的整数波特率！")


    # 添加GNSS传感器
    gnss_connection = get_available_port("请选择GNSS串口")

    if gnss_connection:
        gnss = SensorFactory.create_sensor("gnss", gnss_connection)
        sensors.append(gnss)
        file_handles["gnss"], gnss_filename = create_output_file("GNSS")
        print(f"GNSS数据将保存至: {gnss_filename}")

    if not sensors:
        print("未选择任何传感器，程序退出")
        return

    # 启动所有传感器
    for sensor in sensors:
        sensor.start()

    print("\n传感器数据采集已启动...")
    print("按 Ctrl+C 停止")

    try:
        # 数据采集循环
        while True:
            for sensor in sensors:
                data = sensor.get_data(block=False)
                if data:
                    sensor_type = type(sensor).__name__.lower()
                    if sensor_type not in file_handles:
                        continue

                    file = file_handles[sensor_type]
                    timestamp = data['timestamp'].strftime("%H:%M:%S.%f")[:-3]

                    if sensor_type == "imu":
                        acc = data['acceleration']
                        gyro = data['gyro']
                        file.write(
                            f"[{timestamp}] 加速度: {acc[0]:.3f}, {acc[1]:.3f}, {acc[2]:.3f} | 角速度: {gyro[0]:.3f}, {gyro[1]:.3f}, {gyro[2]:.3f}\n")

                    elif sensor_type == "gnss":
                        file.write(
                            f"[{timestamp}] 位置: {data['location'].lat:.6f}, {data['location'].lon:.6f}, {data['altitude']:.2f}m | 定位: {data['fix_type']} | 卫星: {data['satellites']}\n")

                    file.flush()  # 确保数据写入磁盘

            time.sleep(0.01)  # 减少CPU占用

    except KeyboardInterrupt:
        print("\n程序已停止")
    finally:
        # 停止所有传感器
        for sensor in sensors:
            sensor.stop()

        # 关闭所有文件
        for file in file_handles.values():
            file.close()
        print("数据文件已关闭")


if __name__ == "__main__":
    main()