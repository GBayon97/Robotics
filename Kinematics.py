class Kinematics:

    def __init__(self, l1=12.5, l2=12.5, l3=6, items = 10, puntos=none):
    """
    l1, l2 and l3 are the lenghts from the different segments the Arduino Braccio has
    items is the number of iterations you want the inverse kinematic method to perform
    puntos are some targets the arm has with its respectives servo angles, for making the inverse kinematic function faster 
    """
        self.items=items
        self.l1=l1
        self.l2=l2
        self.l3=l3
        if puntos == none:
          self.puntos = {
              '10,20,0' : [117, 125, 9, 68],
              '10,20,-6': [117, 142, 7, 83],
              '10,20,15': [117, 120, 62, 34],
              '10,20,20': [117, 126, 78, 61],
              '0,20,0'  : [90, 133, 28, 9],
              '0,20,-6' : [90, 140, 10, 9],
              '0,20,15' : [90, 109, 49, 28],
              '0,20,23' : [90, 122, 81, 69],
              '20,20,0' : [135, 152, 36, 111],
              '20,20,-2': [135, 157, 37, 116],
              '20,20,10': [135, 143, 60, 89],
              '20,10,5' : [153, 114, 2, 94],
              '20,10,-6': [153, 143, 0, 109],
              '20,10,0' : [153, 143, 0, 92],
              '20,10,21': [153, 130, 84, 72],
              '20,0,21' : [180, 108, 46, 96],
              '20,0,15' : [180, 98, 14, 97],
              '20,0,10' : [180, 98, 0, 93],
              '0,25,0'  : [90, 163, 84, 0],
              '0,25,10' : [90, 134, 66, 30],
              '0,25,15' : [90, 134, 77, 48],
              '25,0,0'  : [180, 139, 9, 129],
              '25,0,-6' : [180, 167, 22, 162],
              '25,0,10' : [180, 124, 24, 118],
              '25,0,15' : [180, 128, 46, 116]}
          else:
            self.puntos = puntos

    def directa(self,q1, q2, q3, q4):
    """
    q1, q2, q3, q4 are the Denavit-Hartenberg parameters
   Returns the point the Arduino Braccio will reach when adopting the input angles
    """
        mat = np.array([
            [(self.l1 * np.cos(math.radians(q2)) - self.l2 * np.sin(math.radians(q2 - q3)) - self.l3 * np.cos(math.radians(q3 + q4 - q2))) * np.cos(math.radians(q1))],
            [(-self.l1 * np.cos(math.radians(q2)) + self.l2 * np.sin(math.radians(q2 - q3)) + self.l3 * np.cos(math.radians(q3 + q4 - q2))) * np.sin(math.radians(q1))],
            [self.l1 * np.sin(math.radians(q2)) + self.l2 * np.cos(math.radians(q2 - q3)) + self.l3 * np.sin(math.radians(q3 + q4 - q2))]],dtype=np.float64)
        return mat

    def jacobiano(self,q1, q2, q3, q4):
    """
    Returns the Jacobian matrix for the Arduino Braccio
    """
        mat = np.array([
            [-(self.l1 * np.cos(math.radians(q2)) - self.l2 * np.sin(math.radians(q2 - q3)) - self.l3 * np.cos(math.radians(q3 + q4 - q2))) * np.sin(math.radians(q1)), (-self.l1 * np.sin(math.radians(q2)) - self.l2 * np.cos(math.radians(q2 - q3)) - self.l3 * np.sin(math.radians(q3 + q4 - q2))) * np.cos(math.radians(q1)),(self.l2 * np.cos(math.radians(q2 - q3)) + self.l3 * np.sin(math.radians(q3 + q4 - q2))) * np.cos(math.radians(q1)), self.l3 * np.sin(math.radians(q3 + q4 - q2)) * np.cos(math.radians(q1))],
            [(-self.l1 * np.cos(math.radians(q2)) + self.l2 * np.sin(math.radians(q2 - q3)) + self.l3 * np.cos(math.radians(q3 + q4 - q2))) * np.cos(math.radians(q1)), (self.l1 * np.sin(math.radians(q2)) + self.l2 * np.cos(math.radians(q2 - q3)) + self.l3 * np.sin(math.radians(q3 + q4 - q2))) * np.sin(math.radians(q1)),(-self.l2 * np.cos(math.radians(q2 - q3)) - self.l3 * np.sin(math.radians(q3 + q4 - q2))) * np.sin(math.radians(q1)), -self.l3 * np.sin(math.radians(q3 + q4 - q2)) * np.sin(math.radians(q1))],
            [0, self.l1 * np.cos(math.radians(q2)) - self.l2 * np.sin(math.radians(q2 - q3)) - self.l3 * np.cos(math.radians(q3 + q4 - q2)),self.l2 * np.sin(math.radians(q2 - q3)) + self.l3 * np.cos(math.radians(q3 + q4 - q2)),self.l3 * np.cos(math.radians(q3 + q4 - q2))],
            [-np.sin(math.radians(q1)), np.cos(math.radians(q2)), np.sin(math.radians(q3)),np.sin(math.radians(q4))],
            [-np.cos(math.radians(q1)), np.sin(math.radians(q2)), np.cos(math.radians(q3)),np.cos(math.radians(q4))],
            [0, 0, 0, 0]],dtype=np.float64)
        return mat

    def punto_final(self,x, y, z):
    """
    Returns the closest points from your list of points to your target point
    """
        punto = [x, y, z]
        lst=[]
        for j, i in enumerate(self.puntos.keys()):
            aux = np.sqrt(float((punto[0] - int(i.split(',')[0])) ** 2) + float((punto[1] - int(i.split(',')[1])) ** 2)
                        + float((punto[2] - int(i.split(',')[2])) ** 2))
            lst.append([aux,j])
        lst.sort()
        return lst[:self.items]
    
    def inversa(self,x,y,z):
    """
    Returns the angles required for reaching the desired point with coordinates (x,y,z) with the Arduino Braccio
    """
        lstt = self.punto_final(x, y, z)
        for k in range(self.items +1):
            if k==0:
                angulos_init=[90,45,0,0]
                empezar=angulos_init
            else:
                angulos_init=lstt
                empezar=list(self.puntos.items())[angulos_init[k-1][1]][1]
            ang = empezar[0]
            ang1 = empezar[1]
            ang2 = empezar[2]
            ang3 = empezar[3]
            actual = self.directa(ang, ang1, ang2, ang3)
            actual = np.concatenate((actual, [[0], [0], [0]]))
            punto = np.array([[x], [y], [z], [0], [0], [0]])
            tiempo = time.time()
            while (1):
                dx = punto - actual
                j0 = self.jacobiano(ang, ang1, ang2, ang3)
                j_inv = np.linalg.pinv(j0)
                dq = np.dot(j_inv, (dx * .001))
                ang = ang + math.degrees(dq[0])
                ang1 = ang1 + math.degrees(dq[1])
                ang2 = ang2 + math.degrees(dq[2])
                ang3 = ang3 + math.degrees(dq[3])
                actual = self.directa(ang, ang1, ang2, ang3)
                actual = np.concatenate((actual, [[0], [0], [0]]))
                if np.sqrt(float(dx[0] ** 2) + float(dx[1] ** 2) + float(dx[2] ** 2)) < .01 or (time.time() - tiempo) > 10:
                    break
            cont=0
            for i in np.array([[ang], [ang1], [ang2], [ang3]]):
                if i[0]<0 or i[0]>180:
                    cont+=1
            if ang1>165 or ang1 <15:
                cont+=1
            if cont==0:
                ret=np.zeros(4)
                for j,i in enumerate(np.round(([[ang], [ang1], [ang2], [ang3]]), decimals=1)):
                    if 0.25<i%int(i)<0.75:
                        ret[j]=float(str(i[0]).split('.')[0]+'.5')
                    elif i%int(i)>0.75:
                        ret[j]=int(np.ceil(i))
                    else:
                        ret[j]=int(i)
                break
        return ret
