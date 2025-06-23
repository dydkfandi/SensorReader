from abc import ABC, abstractmethod
from imu_reader import IMUReader
from gnss_reader import GNSSReader


class Sensor(ABC):
    """传感器抽象基类"""

    @abstractmethod
    def start(self):
        """启动传感器数据采集"""
        pass

    @abstractmethod
    def stop(self):
        """停止传感器数据采集"""
        pass

    @abstractmethod
    def get_data(self):
        """获取传感器数据"""
        pass


class SensorFactory:
    """传感器工厂类，用于创建不同类型的传感器"""

    @staticmethod
    def create_sensor(sensor_type, *args, **kwargs):
        """创建传感器实例"""
        if sensor_type.lower() == "imu":
            return IMUReader(*args, **kwargs)
        elif sensor_type.lower() == "gnss":
            return GNSSReader(*args, **kwargs)
        else:
            raise ValueError(f"未知的传感器类型: {sensor_type}")