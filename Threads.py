from threading import Thread,Semaphore,Event
import time
from ADC import LDR, IR
import Kinematics
import Servos
import IMU_bueno 
from picamera import PiCamera 
from picamera.array import PiRGBArray
import cv2
import numpy as np
import intento_camaraIR


class Thread_IMU(Thread):
    def __init__(self,semIMU,semServos,evento):
        Thread.__init__(self)
        self.imu=IMU_bueno.IMU()
        self.semIMU=semIMU
        self.semServos=semServos
        self.stop = evento
        self.servoimu=Robot_def.Servo_PCA9685()
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
        self.robot=Robot_def.Servo_PCA9685()
        self.LDR=LDR()
        self.semIMU=semIMU
        self.semServos=semServos
        self.semCamara=semCamara
        Thread.__init__(self)
    def calibrar_imu(self):
        
        self.robot.Servo(servo=1,grados=90,sleep=True)
        self.robot.Servo(servo=2,grados=90-15,sleep=True)
        self.robot.Servo(servo=3,grados=90,sleep=True)
        self.robot.Servo(servo=4,grados=90,sleep=True)
    def casa(self,a,b,c,d):
        #PRIMERO COGERIA LA COMIDA Y LUEGO MOVERIA A CASA
        self.smooth(a,b-15,c,d,90,45-15,0,0)
        
    def smooth(self,pos1,pos2,pos3,pos4,posf1,posf2,posf3,posf4):
        salto=2.5
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
        
        

    def comer(self,angulo):
        self.smooth(90,45-15,0,0,angulo[0],angulo[1]-15,angulo[2],angulo[3])
        
    def run(self):
        #calibrar imu
        self.calibrar_imu()
        time.sleep(1)
        self.semIMU.release()
        self.semServos.acquire()
        self.casa(90,90,90,90)
        time.sleep(1)
        self.semCamara.release()
        while True:
            #ESPERO A QUE LA CAMARA ACTIVE PARA DAR DE COMER
            self.semServos.acquire()
            self.comer(angulos)
#             self.comer(45,90,125,35)
            #bucle while LDR() UNA VEZ QUE ENTRE Y SALGA DE LA BOCA
            while(self.LDR.value()<2.5):
                print("esperando para comer")
                time.sleep(1)
                
            while(self.LDR.value()>2):
                print("COMIENDO!!!")
                time.sleep(1)
                
            print("Ha dejado de comer")
            time.sleep(2)
            self.casa(angulos[0],angulos[1],angulos[2],angulos[3])
#             self.casa(45,90,125,35)
            print('Casita')
            time.sleep(5)
            self.semCamara.release()
            
            
        

            

            
class Thread_Camara(Thread):
    def __init__(self,cam,brazo):
        self.semCam,self.semBrazo=cam,brazo
        self.camara=intento_camaraIR.Camara()
#         self.sensor_ir=IR()
        self.robot=Robot_def.Robot()
#         self.pwm=Robot_def.Servo_PCA9685()
#         self.camara=PiCamera()        
#         res=(1280,720)
#         self.camara.resolution=res
#         self.camara.framerate=32
#         self.rawCapture=PiRGBArray(self.camara,size=res)
        Thread.__init__(self)
    def run(self):
        while True:
            self.semCam.acquire()
#             print("Camara grabando")
#             circles = np.loadtxt("/home/pi/Desktop/definitivo/circulos.txt")
#             boca=[]
#             start=time.time()
#             for frame in self.camara.capture_continuous(self.rawCapture,format="bgr",use_video_port=True):
#                 img=frame.array
#                 small_frame = cv2.resize(img, (0, 0), fx=0.2, fy=0.2)
#                 rgb_small_frame = small_frame[:, :, ::-1]
#                 face_locations = face_recognition.face_locations(rgb_small_frame)
#                 for (top, right, bottom, left) in face_locations:
#                         top *= 5
#                         right *= 5
#                         bottom *= 5
#                         left *= 5
#                         cv2.rectangle(img, (left, top), (right, bottom), (0, 0, 255), 2)
#                         cv2.rectangle(img, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
#                         font = cv2.FONT_HERSHEY_DUPLEX
#                         cv2.putText(img, 'PUTO AMO', (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
#                 face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
#                 try:
#                     mouthT=[]
#                     mouthL=[]
#                     for x in face_landmarks_list[0]['top_lip']:
#                         mouthT.append(tuple(5*i for i in x))
#                     for x in face_landmarks_list[0]['bottom_lip']:
#                         mouthL.append(tuple(5*i for i in x))
#                     for i,x in enumerate(zip(mouthT,mouthL)):
#                         if i == 0:
#                             minTx=x[0][0]
#                             minLx=x[1][0]
#                             maxTx=x[0][0]
#                             maxLx=x[1][0]
#                             minTy=x[0][1]
#                             minLy=x[1][1]
#                             maxTy=x[0][1]
#                             maxLy=x[1][1]
#                         else:
#                             minTx=x[0][0] if x[0][0]<minTx else minTx
#                             minLx=x[1][0] if x[1][0]<minLx else minLx
#                             maxTx=x[0][0] if x[0][0]>maxTx else maxTx
#                             maxLx=x[1][0] if x[1][0]>maxLx else maxLx
#                             minTy=x[0][1] if x[0][1]<minTy else minTy
#                             minLy=x[1][1] if x[1][1]<minLy else minLy
#                             maxTy=x[0][1] if x[0][1]>maxTy else maxTy
#                             maxLy=x[1][1] if x[1][1]>maxLy else maxLy
# 
#                     minX=min(minTx,minLx)
#                     minY=min(minTy,minLy)
#                     maxX=max(maxTx,maxLx)
#                     maxY=max(maxTy,maxLy)
#                     middle=(int((minX+maxX)/2),int((minY+maxY)/2))
#                     cv2.rectangle(img, (minX, minY), (maxX, maxY), (0, 255, 0), 1)
#                     cv2.circle(img, middle, 1, (255,255,0),1)
#                     boca.append(middle[0])
#                     boca.append(middle[1])
#                     print("Boca detectada {}".format(middle))
#                 except IndexError:
#                     pass
#                 self.rawCapture.truncate(0)
#                 if len(boca)==2:
#                     print("Todo detectado")
#                     break
#                 time.sleep(0.5)
#             elapsed=time.time()-start
#             print("Tiempo de deteccion",elapsed)
#             
#             cv2.circle(img,(int(circles[0]),int(circles[1])),1,(0,0,0),5)
#             cv2.circle(img,(int(circles[2]),int(circles[3])),1,(255,255,0),5)
#             cv2.circle(img,(int(circles[4]),int(circles[5])),1,(0,0,255),5)
#             cv2.circle(img,(int(circles[6]),int(circles[7])),1,(0,255,0),5)
#             cv2.imshow('img',img)
#             cv2.waitKey(0)
#             cv2.destroyAllWindows()
#             cv2.waitKey(1)
#             file=np.loadtxt("/home/pi/Desktop/anterior/calibrar/calibrar.txt")
#             ret=int(file[0])
#             mtx=np.zeros((3,3))
#             dist=np.zeros((1,5))
#             j=k=0
#             for i in range(1,10):
#                 mtx[j][k]=float(file[i])
#                 k+=1
#                 if k%3==0:
#                     j+=1
#                     k=0
#             for i in range(10,15):
#                 dist[0][i-10]=float(file[i])
#             boca=np.resize(np.asarray(boca,np.float64),(3,1))
#             boca[2]=1
#             print('boca {}'.format(boca))
#             orden=[[0,0,0],[12,0,23],[39,0,23],[55,0,0]]
#             orden1=np.asarray(orden,np.float64)
#             circles=np.asarray(circles,np.float64)
#             print('circulos {}'.format(circles))
#             circles=np.resize(circles,(4,2))
#             _,rvec,T=cv2.solvePnP(orden1,circles,mtx,dist)
#             R,_=cv2.Rodrigues(rvec)
#             leftmat=np.dot(np.dot(np.linalg.inv(R),np.linalg.inv(mtx)),boca)
#             rightmat=np.dot(np.linalg.inv(R),T)
#             suma=0
#             cont=0
#             dormir = 0.05
#             for i in range(0,10):
#                 try:
#                     suma+=self.sensor_ir.value()
#                     cont+=1
#                 except ZeroDivisionError:
#                     print("El puto error del IR")
#                 time.sleep(dormir)
#             suma/=cont
#             print(suma,cont)
#             lejos = 29 - suma
#             print(lejos)
#             s=(lejos +rightmat[2])/(leftmat[2])
#             tres_D=(s*leftmat)-rightmat
#             print(tres_D)
#             val_x=35-tres_D[2]
# #             val_x=10
#             val_y=58-tres_D[0]
#             val_z=tres_D[1]
            global angulos
            val_x,val_y,val_z=self.camara.encontrar_cara()
            print("PUNTO")
            print(val_x,val_y,val_z)
#             pos=intento_camaraIR.Pos_3D.Rotaciones()
#             val_x,val_y,val_z=pos.Traslacion(p=(val_x,val_y,val_z),d=10)
#             print("PUNTO Trasladado")
#             print(val_x,val_y,val_z)
            angulos=self.robot.inversa(val_x,val_y,val_z)
            print('cinematica calcualda')
            print(angulos)
            self.semBrazo.release()
