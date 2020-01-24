#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import smbus
import time
import numpy as np

ADDR_ACCEL_CONFIG = 0x1C
ADDR_GYRO_CONFIG = 0x1B

ACCEL_FS_0 = 0x00 << 3
ACCEL_FS_1 = 0x01 << 3
ACCEL_FS_2 = 0x02 << 3
ACCEL_FS_3 = 0x03 << 3

GYRO_FS_0 = 0x00 << 3
GYRO_FS_1 = 0x01 << 3
GYRO_FS_2 = 0x02 << 3
GYRO_FS_3 = 0x03 << 3

ADDR_ACCEL_XOUT_H = 0x3B
ADDR_ACCEL_XOUT_L = 0x3C
ADDR_ACCEL_YOUT_H = 0x3D
ADDR_ACCEL_YOUT_L = 0x3E
ADDR_ACCEL_ZOUT_H = 0x3F
ADDR_ACCEL_ZOUT_L = 0x40

ADDR_GYRO_XOUT_H = 0x43
ADDR_GYRO_XOUT_L = 0x44
ADDR_GYRO_YOUT_H = 0x45
ADDR_GYRO_YOUT_L = 0x46
ADDR_GYRO_ZOUT_H = 0x47
ADDR_GYRO_ZOUT_L = 0x48

ADDR_PWR_MGMT_1 = 0x6B

LOW_PASS = False

# Address of DLPF port
MPU6050_DLPF_BW_5 = 0x1A


class Node:

    def __init__(self):
        rospy.init_node("imu_node")
        self.rate = rospy.Rate(100)  # 100
        self.bus = smbus.SMBus(1)
        # address is either 0x68 or 0x69. see MPU-6050 data sheet for details
        self.address = 0x68

        if LOW_PASS == True:
            self.bus.write_byte_data(self.address, MPU6050_DLPF_BW_5, 6)

        # set full scale for gyroscope and accelerometer
        self.accel_fs = ACCEL_FS_0
        self.gyro_fs = GYRO_FS_0

        self.init_sensor()
        self.publisher = rospy.Publisher("~imu",
                                         Imu,
                                         queue_size=1)

    def run(self):
        while not rospy.is_shutdown():
            imu_message = Imu()
            accel, gyro = self.get_all()

            imu_message.angular_velocity.x = gyro[0]
            imu_message.angular_velocity.y = gyro[1]
            imu_message.angular_velocity.z = gyro[2]

            # TODO genaues mapping auf tatsaechliche Beschl
            imu_message.linear_acceleration.x = accel[0]
            imu_message.linear_acceleration.y = accel[1]
            imu_message.linear_acceleration.z = accel[2]

            imu_message.header.stamp = rospy.Time.now()

            self.publisher.publish(imu_message)

            self.rate.sleep()

    def read_byte(self, register):
        return self.bus.read_byte_data(self.address, register)

    def read_word(self, register):
        low = self.bus.read_byte_data(self.address, register)
        high = self.bus.read_byte_data(self.address, register + 1)
        return (high << 8) + low

    def write_byte(self, register, data):
        self.bus.write_byte_data(self.address, register, data)

    def init_sensor(self):
        self.write_byte(ADDR_PWR_MGMT_1, 0)
        time.sleep(0.2)
        self.set_accel_full_scale(self.accel_fs)
        val = self.get_accel_full_scale()
        if not (val == self.accel_fs):
            raise Exception("Could not set accel_full_scale to '{}'. "
                            "Actual value: '{}'".format(self.accel_fs, val))

        self.set_gyro_full_scale(self.gyro_fs)
        val = self.get_accel_full_scale()
        if not (val == self.gyro_fs):
            raise Exception("Could not set gyro_full_scale to '{}'. "
                            "Actual value: '{}'".format(self.gyro_fs, val))

    def set_accel_full_scale(self, full_scale):
        self.write_byte(ADDR_ACCEL_CONFIG, full_scale)

    def get_accel_full_scale(self):
        return self.read_byte(ADDR_ACCEL_CONFIG)

    def set_gyro_full_scale(self, full_scale):
        self.write_byte(ADDR_GYRO_CONFIG, full_scale)

    def get_gyro_full_scale(self):
        return self.read_byte(ADDR_GYRO_CONFIG)

    def get_all_raw(self):
        return self.bus.read_i2c_block_data(self.address, ADDR_ACCEL_XOUT_H, 14)

    def get_all(self):
        data = self.get_all_raw()
        accel_x = np.int16((data[0] << 8) | data[1])
        accel_y = np.int16((data[2] << 8) | data[3])
        accel_z = np.int16((data[4] << 8) | data[5])
        gyro_x = np.int16((data[8] << 8) | data[9])
        gyro_y = np.int16((data[10] << 8) | data[11])
        gyro_z = np.int16((data[12] << 8) | data[13])

        accel = np.array([accel_x, accel_y, accel_z], dtype=np.float64)
        gyro = np.array([gyro_x, gyro_y, gyro_z], dtype=np.float64)
        accel = accel / self.get_accel_scale_factor()
        gyro = gyro / self.get_gyro_scale_factor()
        return accel, gyro

    def get_accel_scale_factor(self):
        if self.accel_fs == ACCEL_FS_0:
            return 16384.0
        elif self.accel_fs == ACCEL_FS_1:
            return 8192.0
        elif self.accel_fs == ACCEL_FS_2:
            return 4096.0
        elif self.accel_fs == ACCEL_FS_3:
            return 2048.0
        else:
            raise Exception("No valid accel_fs: {}".format(self.accel_fs))

    def get_gyro_scale_factor(self):
        if self.gyro_fs == GYRO_FS_0:
            return 131.0
        elif self.gyro_fs == GYRO_FS_1:
            return 65.5
        elif self.gyro_fs == GYRO_FS_2:
            return 32.8
        elif self.gyro_fs == GYRO_FS_3:
            return 16.4
        else:
            raise Exception("No valid gyro_fs: {}".format(self.gyro_fs))


if __name__ == "__main__":
    node = Node()
    node.run()
