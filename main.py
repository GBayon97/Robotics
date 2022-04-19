import threads
import time
from threading import Semaphore

if __name__=='__main__':
  stop=threads.Event()
  semIMU=Semaphore(0)
  semServos=Semaphore(0)
  semCamara=Semaphore(0)

  IMU=threads.Thread_IMU(semIMU,semServos,stop)
  Servos=threads.Thread_robot(semIMU,semServos,semCamara)
  Camara=threads.Thread_Camara(semCamara,semServos) 
  
  Servos.start()
  IMU.start()
  Camara.start()
