from picamera import PiCamera
import ADC
from picamera.array import PiRGBArray
import face_recognition
import numpy as np
import math
import time
import cv2
import servos
import pos_3D
import TOF

class Camara:
    def __init__(self):
        res=(1280,720)
        self.cam=PiCamera()
        self.cam.resolution=res
        self.cam.framerate=32
        self.rawCapture=PiRGBArray(self.cam,size=res)
    
    def detectarboca(self,method=0):
      """
      Returns the center pixel from where the mouth was detected
      """
        middle=0  
        for frame1 in self.cam.capture_continuous(self.rawCapture,format="bgr",use_video_port=True):
            img=frame1.array
            frame=img
            small_frame = cv2.resize(frame, (0, 0), fx=0.2, fy=0.2)

            # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
            rgb_small_frame = small_frame[:, :, ::-1]

            # Only process every other frame of video to save time
            # Find all the faces and face encodings in the current frame of video
            face_locations = face_recognition.face_locations(rgb_small_frame)
            for (top, right, bottom, left) in face_locations:
                    # Scale back up face locations since the frame we detected in was scaled to 1/4 size
                    top *= 5
                    right *= 5
                    bottom *= 5
                    left *= 5

                    # Draw a box around the face
                    cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

                    # Draw a label with a name below the face
                    cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
                    font = cv2.FONT_HERSHEY_DUPLEX
                    cv2.putText(frame, 'PUTO AMO', (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)
            face_landmarks_list = face_recognition.face_landmarks(rgb_small_frame)
            try:
                mouthT=[]
                mouthL=[]
                for x in face_landmarks_list[0]['top_lip']:
                    mouthT.append(tuple(5*i for i in x))
                for x in face_landmarks_list[0]['bottom_lip']:
                    mouthL.append(tuple(5*i for i in x))
                for i,x in enumerate(zip(mouthT,mouthL)):
                    if i == 0:
                        minTx=x[0][0]
                        minLx=x[1][0]
                        maxTx=x[0][0]
                        maxLx=x[1][0]
                        minTy=x[0][1]
                        minLy=x[1][1]
                        maxTy=x[0][1]
                        maxLy=x[1][1]
                    else:
                        minTx=x[0][0] if x[0][0]<minTx else minTx
                        minLx=x[1][0] if x[1][0]<minLx else minLx
                        maxTx=x[0][0] if x[0][0]>maxTx else maxTx
                        maxLx=x[1][0] if x[1][0]>maxLx else maxLx
                        minTy=x[0][1] if x[0][1]<minTy else minTy
                        minLy=x[1][1] if x[1][1]<minLy else minLy
                        maxTy=x[0][1] if x[0][1]>maxTy else maxTy
                        maxLy=x[1][1] if x[1][1]>maxLy else maxLy

                minX=min(minTx,minLx)
                minY=min(minTy,minLy)
                maxX=max(maxTx,maxLx)
                maxY=max(maxTy,maxLy)
                middle=(int((minX+maxX)/2),int((minY+maxY)/2))
                cv2.rectangle(frame, (minX, minY), (maxX, maxY), (0, 255, 0), 1)
                cv2.circle(frame, middle, 1, (255,255,0),1)
            except IndexError:
                pass
            self.rawCapture.truncate(0) 
            if middle!=0:
                break
            elif middle==0 and method==0:
                return 0,0
        return middle,frame
    
    def encontrar_cara(self):
      """
      Returns the 3D point from the mouth on the Arduino Braccio working space
      """
        pwm = servos.PCA9685()
        pwm.Servo(servo=7,grados=90,sleep=True)
        pwm.Servo(servo=8,grados=90,sleep=True)
        sensor=TOF.tof()
        while(self.detectarboca()[0]==0):
            pwm.Servo(servo=8,grados=pwm.getAngle(8)[0]-3)
            pwm.Servo(servo=7,grados=pwm.getAngle(7)[0]-3)
            print(pwm.getAngle(7))
            print(pwm.getAngle(8))
            time.sleep(0.1)
        while(True):
            boca,fr=self.detectarboca(1)
            print(boca)
            if self.cam.resolution[0]/2 - 20 <boca[0]<self.cam.resolution[0]/2 + 20 and self.cam.resolution[1]/2 - 20 <boca[1]< self.cam.resolution[1]/2 + 20:
                break
            if boca[0]<self.cam.resolution[0]/2 - 20:
                e=self.cam.resolution[0]/2-boca[0]
                e=np.floor(e/10)
                pwm.Servo(servo=7,grados=pwm.getAngle(7)[0]+3)
                time.sleep(0.5)
            elif boca[0]>self.cam.resolution[0]/2 + 20:
                e=self.cam.resolution[0]/2-boca[0]
                e=np.floor(e/10)
                pwm.Servo(servo=7,grados=pwm.getAngle(7)[0]-3)
                time.sleep(0.5)
            if boca[1]<self.cam.resolution[1]/2 - 20 :
                pwm.Servo(servo=8,grados=pwm.getAngle(8)[0]-0.5)
                time.sleep(0.5)
            elif boca[1]>self.cam.resolution[1]/2 + 20:
                pwm.Servo(servo=8,grados=pwm.getAngle(8)[0]+0.5)
                time.sleep(0.5)
        cv2.imshow('img',fr)
        key=cv2.waitKey(0)
        if key == ord("q"):
            cv2.destroyAllWindows()
        cv2.waitKey(1)  
        suma=0
        cont=0
        dormir = 0.05
        lista=[]
        for i in range(0,10):
            try:
                valor=sensor.value()
                suma+=valor
                lista.append(valor)
                cont+=1
            except ZeroDivisionError:
            time.sleep(dormir)
        suma/=cont
        suma=suma/10
        print(suma,lista)
        r=pos_3D.Rotaciones()
        r1=r.puntoFinal(angY=90-pwm.getAngle(8)[0],angZ=90-pwm.getAngle(7)[0],distancia=suma)
        return (r1[0][0]-20,30+r1[1][0],r1[2][0]+3.5)
