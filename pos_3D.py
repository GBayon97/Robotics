import numpy as np
import math

class Rotaciones:
  """
  Rotation matrixes
  """

    def __init__(self):
        self.punto=np.array([[0],
            [0],
            [0]])

    def X(self,ang1):
        ang = math.radians(ang1)
        return np.array([[1, 0, 0],
                         [0, np.cos(ang), np.sin(ang)],
                         [0, -np.sin(ang), np.cos(ang)]],dtype=np.float16)

    def Z(self,ang1):
        ang = math.radians(ang1)
        return np.array([
            [np.cos(ang), np.sin(ang), 0],
            [-np.sin(ang), np.cos(ang), 0],
            [0, 0, 1]], dtype=np.float16)

    def Y(self,ang1):
        ang = math.radians(ang1)
        return np.array([
            [np.cos(ang), 0, -np.sin(ang)],
            [0, 1, 0],
            [np.sin(ang), 0, np.cos(ang)]], dtype=np.float16)

    def puntoFinal(self,angY=0,angZ=0,distancia=0):
      """
      angY,angZ are the angles each servo has move in each axis
      distancia is the linear distance computed via the TOF sensor
      Returns the 3D point from another point which distance is distancia
      """
        self.punto[0]=distancia
        return np.round(np.dot(np.dot(self.Y(angY),self.Z(angZ)),self.punto),4)
    
    def Traslacion(self,p=(0,0,0),d=7,ang=10):
        x=d*np.sin(math.radians(ang))
        y=d*np.cos(math.radians(ang))
        return p[0]-7,p[1]-7,p[2]
