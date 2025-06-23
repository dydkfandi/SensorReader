import argparse
import time
import os
from datetime import datetime
from sensor_factory import SensorFactory
import serial
import serial.tools.list_ports
import logging
import yaml

from imu_reader import IMUReader
from gnss_reader import GNSSReader

# 配置日志
logging.basicConfig(level=logging.INFO, format='%(asctime)s - %(levelname)s - %(message)s')

def list_serial_ports():
    """列出所有可用串口并返回设备列表"""
    ports = serial.tools.list_ports.comports()
    if not ports:
        logging.warning("未找到可用串口设备")
        return []

    logging.info("\n可用串口列表:")
    port_list = []
    for i, port in enumerate(ports):
        logging.info(f"{i + 1}. {port.device} - {port.description}")
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
            logging.warning("无效选择，请重新输入")
        except (ValueError, IndexError):
            logging.warning("请输入有效的序号")

def create_output_file(sensor_type):
    """创建带时间戳的输出文件"""
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    filename = f"{sensor_type}_{timestamp}.txt"
    file = open(filename, 'w')

    if sensor_type == 'IMU':
        file.write("接收时间 x轴角速度(rad/s) y轴角速度(rad/s) z轴角速度(rad/s) x轴加速度(m/s^2) y轴加速度(m/s^2) z轴加速度(m/s^2) IMU温度(°C) IMU时间(s)\n")
    elif sensor_type == 'GNSS':
        file.write("接收时间 纬度(deg) 经度(deg) 高度(m) GNSS状态 卫星数量\n")
    return file

def load_config(config_file):
    """加载配置文件"""
    if not os.path.exists(config_file):
        logging.error(f"配置文件 {config_file} 不存在")
        return None

    with open(config_file, 'r') as file:
        return yaml.safe_load(file)

def main():
    parser = argparse.ArgumentParser(description='多传感器数据采集系统')
    parser.add_argument('--config', help='配置文件路径', default='config.yaml')
    args = parser.parse_args()

    config = load_config(args.config)
    if not config:
        return

    sensors = []
    file_handles = {}

    for sensor_config in config['sensors']:
        sensor_type = sensor_config['type'].lower()
        if sensor_type == "imu":
            port = get_available_port("请选择IMU串口")
            baudrate = sensor_config.get('baudrate')
            imu = SensorFactory.create_sensor("imu", port, baudrate)
            sensors.append(imu)
            file_handles[imu] = create_output_file("IMU")
            logging.info(f"IMU数据将保存至: {file_handles[imu].name}")
        elif sensor_type == "gnss":
            connection_string = get_available_port("请选择GNSS串口")
            gnss = SensorFactory.create_sensor("gnss", connection_string)
            sensors.append(gnss)
            file_handles[gnss] = create_output_file("GNSS")
            logging.info(f"GNSS数据将保存至: {file_handles[gnss].name}")

    if not sensors:
        logging.warning("未配置任何传感器，程序退出")
        return

    # 启动所有传感器
    for sensor in sensors:
        sensor.start()

    logging.info("传感器数据采集已启动...")
    logging.info("按 Ctrl+C 停止")

    try:
        # 数据采集循环
        while True:
            for sensor, file in file_handles.items():
                data = sensor.get_data(block=False)
                if data:
                    timestamp = data['timestamp'].strftime("%H:%M:%S.%f")[:-3]

                    if isinstance(sensor, IMUReader):
                        acc = data['acc']
                        gyro = data['gyro']
                        imu_temp = data['imu_temperature']
                        imu_time = data['imu_time']
                        file.write(f"{timestamp}    {gyro[0]:f}    {gyro[1]:f}    {gyro[2]:f}    {acc[0]:f}    {acc[1]:f}    {acc[2]:f}    {imu_temp:f}    {imu_time:f}\n")

                    elif isinstance(sensor, GNSSReader):
                        file.write(f"{timestamp}  {data['location'].lat:f}    {data['location'].lon:f}    {data['altitude']:f}    {data['fix_type']}    {data['satellites']}\n")
                        print(f"[{timestamp}] 位置: {data['location'].lat:.6f}, {data['location'].lon:.6f}, {data['altitude']:.2f}m | 状态: {data['fix_type']} | 卫星数量: {data['satellites']}")
                    file.flush()  # 确保数据写入磁盘

            time.sleep(0.01)  # 减少CPU占用

    except KeyboardInterrupt:
        logging.info("\n程序已停止")
    finally:
        # 停止所有传感器
        for sensor in sensors:
            sensor.stop()

        # 关闭所有文件
        for file in file_handles.values():
            file.close()
        logging.info("数据文件已关闭")

if __name__ == "__main__":
    main()