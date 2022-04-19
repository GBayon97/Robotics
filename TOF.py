from smbus import SMBus
import time
import numpy as np
import math

class TOF:
    def __init__(self, i2c = SMBus(4), address = 0x52):
      """
      i2c is the I2C channel to be used
      address is the I2C address from the I2C channel bus
      """
        self.i2c = i2c
        self.i2c_addr = address
        
    def value(self):
      """
      Returns the distance from an object
      """
        self.i2c.write_byte(self.i2c_addr,0x00)
        time.sleep(0.01)
        tmp=self.read_i2c(0x00)
        time.sleep(0.01)
        return tmp

    def read_i2c(self,register):  
        msb=self.i2c.read_byte_data(self.i2c_addr,register)
        lsb=self.i2c.read_byte_data(self.i2c_addr,register +1)
        msb = msb - 256 if msb > 127 else msb
        return (msb << 8) + lsb
