from smbus import SMBus
import time
import numpy as np
import math


class IMU:

    def __init__(self):

        self.i2c = SMBus(3)
        self.i2c_addr = 0x68
        self.period = 0.05
        self.i2c.write_byte_data(self.i2c_addr,0x6b,0x00)
        self.i2c.write_byte_data(self.i2c_addr,0x1b,0x08)
        self.i2c.write_byte_data(self.i2c_addr,0x1c,0x10)
        self.i2c.write_byte_data(self.i2c_addr,0x1a,0x03)
    
    def Calibrate(self):

        print("Calib")
        gyro_X_calibracion = 0
        gyro_Y_calibracion = 0
        gyro_Z_calibracion = 0
        angle_pitch_acc_calibracion = 0
        angle_roll_acc_calibracion = 0
        angle_yaw_acc_calibracion = 0
        self.angulo_pitch = 0
        self.angulo_roll = 0
        el=time.time()
        for i in range(1000):
            ax=self.read_i2c(0x3b)
            ay=self.read_i2c(0x3d)
            az=self.read_i2c(0x3f)
            gx=self.read_i2c(0x43)
            gy=self.read_i2c(0x45)
            gz=self.read_i2c(0x47)
            gyro_X_calibracion += gx
            gyro_Y_calibracion += gy
            gyro_Z_calibracion += gz
            angle_pitch_acc_calibracion += ax
            angle_roll_acc_calibracion += ay
            angle_yaw_acc_calibracion += az
        self.gyro_X_cal = gyro_X_calibracion / 1000
        self.gyro_Y_cal = gyro_Y_calibracion / 1000
        self.gyro_Z_cal = gyro_Z_calibracion / 1000
        self.angle_pitch_acc_cal = angle_pitch_acc_calibracion / 1000
        self.angle_roll_acc_cal = angle_roll_acc_calibracion / 1000
        self.angle_yaw_acc_cal = angle_yaw_acc_calibracion / 1000
        print(time.time()-el)

        self.loop_timer = time.time()
        print("Calib1")

    def read_i2c(self,register):
        
        msb=self.i2c.read_byte_data(self.i2c_addr,register)
        lsb=self.i2c.read_byte_data(self.i2c_addr,register +1)
        msb = msb - 256 if msb > 127 else msb
        return (msb << 8) + lsb

    def processMPU(self):
        ax=self.read_i2c(0x3b)
        ay=self.read_i2c(0x3d)
        az=self.read_i2c(0x3f)
        gx=self.read_i2c(0x43)
        gy=self.read_i2c(0x45)
        gz=self.read_i2c(0x47)
        ax -= self.angle_pitch_acc_cal
        ay -=self.angle_roll_acc_cal
        az -=self.angle_yaw_acc_cal
        az += 4096
        pitchGyro = (gx - self.gyro_X_cal) / 65.5
        rollGyro = (gy - self.gyro_Y_cal) / 65.5
        tiempo_ejecucion = time.time() - self.loop_timer
        self.angulo_pitch += pitchGyro * tiempo_ejecucion 
        self.angulo_roll += rollGyro * tiempo_ejecucion 
        self.angulo_pitch += self.angulo_roll * np.sin(
            (gz - self.gyro_Z_cal) * (tiempo_ejecucion * np.pi) / (65.5 * 180))
        self.angulo_roll -= self.angulo_pitch * np.sin(
            (gz - self.gyro_Z_cal) * ((tiempo_ejecucion) * np.pi) / (65.5 * 180))

        acc_total_vector = np.sqrt(ay ** 2 + ax ** 2 + az ** 2)
        angle_roll_acc = np.arcsin(ax / acc_total_vector) * (-180/np.pi)

        return self.angulo_roll * 0.99 + angle_roll_acc * 0.01
        
    def setLooptimer(self):
        self.loop_timer = time.time()


if __name__ == '__main__':
    
    MPU = IMU()
    MPU.Calibrate()
    while (1):
        angulo=MPU.processMPU()
        print(angulo)
        MPU.setLooptimer()
        time.sleep(0.025)
