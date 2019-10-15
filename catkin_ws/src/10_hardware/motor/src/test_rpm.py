import Adafruit_GPIO.I2C as i2c
import time
import struct
import numpy as np

ticks_per_sec = 16000000/64.0
pulses_per_rev = 10


class Node(object):
    def __init__(self):
        self.dev = i2c.get_i2c_device(0x54)


if __name__ == "__main__":
    n = Node()
    speeds = [0, 0, 0]
    index = 0
    while True:
        time.sleep(0.1)
        data = n.dev.readList(0, 6)
        data = struct.unpack('>HH', data)
        speed = ticks_per_sec/data[1]/pulses_per_rev if data[1] > 0 else 0.0
        speeds[index] = speed
        index = index+1 if index < 2 else 0
        speed = np.median(speeds)
        print(data)
        print(speed)
