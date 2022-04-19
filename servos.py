from smbus import SMBus
import time
import numpy as np
import math

class PCA9685:

    def __init__(self,i2c=SMBus(1), address=0x40):
      """
      i2c is the I2C channel which is being used
      address is the hex address from the PCA9685 in the I2C channel bus
      """
        self.i2c = i2c
        self.i2c_addr = address

        self.i2c.write_byte_data(self.i2c_addr, 0x00, 0x31)
        self.i2c.write_byte_data(self.i2c_addr, 0xfe, 0x79)
        time.sleep(600 * 10 ** -6)
        self.i2c.write_byte_data(self.i2c_addr, 0x00, 0x21)

        servo1 = []
        servo2 = []
        servo3 = []
        servo4 = []
        servo5 = []
        servo6 = []
        servo7=[]
        servo8=[]
        """
        For calibration purposes
        """
        for i in list(np.linspace(81, 482, 181*2 - 1)):
            servo1.append(list(map(hex, divmod(int(round(i)), 1 << 8))))
        for i in list(np.linspace(140, 455, 151*2 - 1)):
            servo2.append(list(map(hex, divmod(int(round(i)), 1 << 8))))
        for i in list(np.linspace(130, 500, 181*2 - 1)):
            servo3.append(list(map(hex, divmod(int(round(i)), 1 << 8))))
        for i in list(np.linspace(121, 490, 181*2 - 1)):
            servo4.append(list(map(hex, divmod(int(round(i)), 1 << 8))))
        for i in list(np.linspace(95, 511, 181*2 - 1)):
            servo5.append(list(map(hex, divmod(int(round(i)), 1 << 8))))
        for i in list(np.linspace(138, 498, 181*2 - 1)):
            servo7.append(list(map(hex, divmod(int(round(i)), 1 << 8))))
        for i in list(np.linspace(266-180, 266+180, 181*2 - 1)):
            servo8.append(list(map(hex, divmod(int(round(i)), 1 << 8))))
        

        self.servos = list((servo1, servo2, servo3, servo4, servo5,servo6,servo7,servo8))

    def Servo(self, servo=1, grados=0,sleep=False):
      """
      servo is the PCA9685 pwm channel which is going to be used
      grados are the degrees you want your servo to move
      """
        grados*=2
        self.i2c.write_byte_data(self.i2c_addr, servo * 4 + 2, 0)
        self.i2c.write_byte_data(self.i2c_addr, servo * 4 + 3, 0)
        self.i2c.write_byte_data(self.i2c_addr, servo * 4 + 4, int(self.servos[servo - 1][int(grados)][1], 16))
        self.i2c.write_byte_data(self.i2c_addr, servo * 4 + 5, int(self.servos[servo - 1][int(grados)][0], 16))
        if sleep:
            time.sleep(.5)
        
    def getAngle(self,servo):
      
        lst=[]            
        msb=self.i2c.read_byte_data(self.i2c_addr,(servo) * 4 + 4)
        lsb=self.i2c.read_byte_data(self.i2c_addr,(servo) * 4 + 5)
        ret = [pos for pos in range(len(self.servos[servo-1])) if self.servos[servo-1][pos] == [hex(lsb), hex(msb)]]
        lst.append(ret[0]/2)
        return lst
