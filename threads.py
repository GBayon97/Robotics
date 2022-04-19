from threading import Thread,Semaphore,Event
import time
from ADC import LDR, IR
import kinematics
import servos
import IMU
import cv2
import numpy as np
import camera


class Thread_IMU(Thread):
    def __init__(self,semIMU,semServos,evento):
        Thread.__init__(self)
        self.imu=IMU.IMU()
        self.semIMU=semIMU
        self.semServos=semServos
        self.stop = evento
        self.servoimu=servos.PCA9685()
        
    def run(self):
        self.servoimu.Servo(servo=5,grados=0)
        self.semIMU.acquire()
        time.sleep(2)
        self.imu.Calibrate()
        self.semServos.release()
        while not self.stop.wait(0.025):
            ang=self.imu.processMPU()
            self.imu.setLooptimer()
            if ang<0:
             ang=0
            if ang>180:
             ang=180
            self.servoimu.Servo(servo=5,grados=ang)


class Thread_robot(Thread):
    def __init__(self,semIMU,semServos,semCamara):
        self.robot=servos.PCA9685()
        self.LDR=LDR()
        self.semIMU=semIMU
        self.semServos=semServos
        self.semCamara=semCamara
        Thread.__init__(self)
        
    def calibrar_imu(self):
        """
        Calibrating the IMU
        """
        self.robot.Servo(servo=1,grados=90,sleep=True)
        self.robot.Servo(servo=2,grados=90-15,sleep=True)
        self.robot.Servo(servo=3,grados=90,sleep=True)
        self.robot.Servo(servo=4,grados=90,sleep=True)
        
    def smooth(self,pos1,pos2,pos3,pos4,posf1,posf2,posf3,posf4, jump = 2.5):
        """
        pos1,pos2,pos3,pos4 are the current servos angles
        posf1,posf2,posf3,posf4 are the target servos angles
        Making the Servos to move smoothly
        """
        salto=jump
        lst1=np.arange(pos1,posf1+salto if posf1>pos1 else posf1-salto,salto if posf1>pos1 else -salto)
        lst1=[x if x>=0 else 0 for x in lst1]
        lst1=[x if x<=180 else 180 for x in lst1]
        lst2=np.arange(pos2,posf2+salto if posf2>pos2 else posf2-salto,salto if posf2>pos2 else -salto)
        lst2=[x if x>=0 else 0 for x in lst2]
        lst2=[x if x<=150 else 150 for x in lst2]
        lst3=np.arange(pos3,posf3+salto if posf3>pos3 else posf3-salto,salto if posf3>pos3 else -salto)
        lst3=[x if x>=0 else 0 for x in lst3]
        lst3=[x if x<=180 else 180 for x in lst3]
        lst4=np.arange(pos4,posf4+salto if posf4>pos4 else posf4-salto,salto if posf4>pos4 else -salto)
        lst4=[x if x>=0 else 0 for x in lst4]
        lst4=[x if x<=180 else 180 for x in lst4]
        
        lst5=[salto if posf1>pos1 else -salto,salto if posf2>pos2 else -salto,salto if posf3>pos3 else -salto,salto if posf4>pos4 else -salto]
        
        for i in np.arange(0,181,0.5):
            j=180-i
            if lst5[0]>0:
                if i in lst1:
                    self.robot.Servo(servo=1,grados=i)
            else:
                if j in lst1:
                    self.robot.Servo(servo=1,grados=j)
            if lst5[1]>0:
                if i in lst2:
                    self.robot.Servo(servo=2,grados=i)
            else:
                if j in lst2:
                    self.robot.Servo(servo=2,grados=j)
            if lst5[2]>0:
                if i in lst3:
                    self.robot.Servo(servo=3,grados=i)
            else:
                if j in lst3:
                    self.robot.Servo(servo=3,grados=j)
            if lst5[3]>0:
                if i in lst4:
                    self.robot.Servo(servo=4,grados=i)
            else:
                if j in lst4:
                    self.robot.Servo(servo=4,grados=j)
            time.sleep(0.005)
        
    def casa(self,a,b,c,d):
        """
        Home position
        """
        self.smooth(a,b-15,c,d,90,45-15,0,0)    

    def comer(self,angulo):
        self.smooth(90,45-15,0,0,angulo[0],angulo[1]-15,angulo[2],angulo[3])
        
    def run(self):
        self.calibrar_imu()
        time.sleep(1)
        self.semIMU.release()
        self.semServos.acquire()
        self.casa(90,90,90,90)
        time.sleep(1)
        self.semCamara.release()
        while True:
            self.semServos.acquire()
            self.comer(angulos)
            while(self.LDR.value()<2.5):
                time.sleep(1)
                
            while(self.LDR.value()>2):
                time.sleep(1)
                
            time.sleep(2)
            self.casa(angulos[0],angulos[1],angulos[2],angulos[3])
            time.sleep(5)
            self.semCamara.release()
                                  
class Thread_Camara(Thread):
    def __init__(self,cam,brazo):
        self.semCam,self.semBrazo=cam,brazo
        self.camara=camera.Camara()
        self.robot=kinematics.Kinematics()
        Thread.__init__(self)
        
    def run(self):
        while True:
            self.semCam.acquire()
            global angulos
            val_x,val_y,val_z=self.camara.encontrar_cara()
            angulos=self.robot.inversa(val_x,val_y,val_z)
            print('cinematica calcualda')
            print(angulos)
            self.semBrazo.release()
