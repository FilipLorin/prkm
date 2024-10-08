from dataclasses import dataclass
import json
import numpy as np
import math



@dataclass
class Robot:
    bZO:float
    wB:float
    L:float
    l:float
    wP:float
    pZO: float

    def __post_init__(self):
        self.toolVec = np.array([0,0,0])
        self.baseVec = np.array([0,0,0])

    def attach_tool(self, tool:np.array):
        self.toolVec = tool

    def attach_base(self, base:np.array):
        self.baseVec = base

    def _minarg(self, a:float, b:float):
        if abs(a) < abs(b):
            return a
        else:
            return b

    def _vec_dist(self, a, b):
        return math.sqrt((a[0]-b[0])**2+(a[1]-b[1])**2+(a[2]-b[2])**2)

    def _internal_inverse_knee_hyperextention(self, x:float, y:float, z:float, tol:float=0.95) -> bool:
        I = np.array([x,y,z])
        P1 = np.array([0, self.wB, 0])
        P2 = np.array([self.wB*math.cos(2*math.pi/3), self.wB*math.sin(2*math.pi/3), 0])
        P3 = np.array([self.wB*math.cos(4*math.pi/3), self.wB*math.sin(4*math.pi/3), 0])
        
        if self._vec_dist(I, P1) > (self.l+self.L)*tol or self._vec_dist(I, P1) < (self.l+self.L)*(1-tol): 
            return True
        if self._vec_dist(I, P2) > (self.l+self.L)*tol or self._vec_dist(I, P2) < (self.l+self.L)*(1-tol): 
            return True
        if self._vec_dist(I, P3) > (self.l+self.L)*tol or self._vec_dist(I, P3) < (self.l+self.L)*(1-tol):
            return True
        return False

    def _internal_inverse_kinematics(self, x:float, y:float, z:float):
        if self._internal_inverse_knee_hyperextention(x,y,z):
            raise ValueError("Inverse kinematics failed, knee hyperextention")

        a = self.wB - 2*self.wP
        b = (3/math.sqrt(3))*self.wP - (math.sqrt(3)/2)*self.wB
        c = self.wP - 0.5*self.wB

        E1 = 2*self.L*(y+a)
        F = 2*z*self.L
        G1 = x**2+y**2+z**2+a**2+self.L**2+2*y*a-self.l**2

        E2 = -self.L*(math.sqrt(3)*(x+b)+y+c)
        G2 = x**2+y**2+z**2+b**2+c**2+self.L**2+2*(x*b+y*c)-self.l**2

        E3 = self.L*(math.sqrt(3)*(x-b)-y-c)
        G3 = x**2+y**2+z**2+b**2+c**2+self.L**2+2*(-x*b+y*c)-self.l**2

        try:
            t11 = (F-math.sqrt(E1**2+F**2-G1**2))/(G1-E1)
            t21 = (F-math.sqrt(E2**2+F**2-G2**2))/(G2-E2)
            t31 = (F-math.sqrt(E3**2+F**2-G3**2))/(G3-E3)
            
            t12 = (F+math.sqrt(E1**2+F**2-G1**2))/(G1-E1)
            t22 = (F+math.sqrt(E2**2+F**2-G2**2))/(G2-E2)
            t32 = (F+math.sqrt(E3**2+F**2-G3**2))/(G3-E3)
        except ValueError:
            raise ValueError("Inverse kinematics failed, target point out of range.")


        # forcing "knee outwards" position
        th1 = self._minarg(2*math.atan(t11), 2*math.atan(t12))
        th2 = self._minarg(2*math.atan(t21), 2*math.atan(t22))
        th3 = self._minarg(2*math.atan(t31), 2*math.atan(t32))

        #TODO: check for invalid data

        return [th1, th2, th3]

    def inverse_kinematics(self, x, y, z, w, precision=6):
        ExtP = np.array([x, y, z])
        BOVec = np.array([0, 0, -self.bZO])
        ToolZRot = np.array([[math.cos(w), -math.sin(w), 0], [math.sin(w), math.cos(w), 0], [0, 0, 1]])
        I = self.baseVec + BOVec - ExtP + ToolZRot.dot(self.toolVec)
        return list(map(lambda x: round(x,precision), [*self._internal_inverse_kinematics(I[0], I[1], I[2]), w]))

    def knee_joint_coordinates(self, th1, th2, th3):
        # knee joints' coordinates
        A1 = np.array([0, 
                       -self.wB-self.L*math.cos(th1)+2*self.wP, 
                       -self.L*math.sin(th1)])
        A2 = np.array([0.5*math.sqrt(3)*(self.wB+self.L*math.cos(th2))-(3/math.sqrt(3))*self.wP, 
                       0.5*(self.wB+self.L*math.cos(th2))-self.wP, 
                       -self.L*math.sin(th2)])
        A3 = np.array([-0.5*math.sqrt(3)*(self.wB+self.L*math.cos(th3))+(3/math.sqrt(3))*self.wP,
                       0.5*(self.wB+self.L*math.cos(th3))-self.wP,
                       -self.L*math.sin(th3)])
        return [A1, A2, A3]

    def _internal_forward_kinematics(self, th1, th2, th3):
        # three spheres intersection algorythm
        
        #  same z height guard clause
        if math.isclose(th1, th2) and math.isclose(th2, th3):
            A1, A2, A3 = self.knee_joint_coordinates(th1, th2, th3)

            #print("Z-Heights close, switching to alternative solver")
            a = 2*(A3[0]-A1[0])
            b = 2*(A3[1]-A1[1])
            c = A3[0]**2+A3[1]**2-A1[0]**2-A1[1]**2
            d = 2*(A3[0]-A2[0])
            e = 2*(A3[1]-A2[1])
            f = A3[0]**2+A3[1]**2-A2[0]**2-A2[1]**2

            x = (c*e-b*f)/(a*e-b*d)
            y = (a*f-c*d)/(a*e-b*d)

            B = -2*A1[2]
            C = A1[2]**2-self.l**2+(x-A1[0])**2+(y-A1[1])**2

            z1 = 0.5*(-B+math.sqrt(B**2-4*C))
            z2 = 0.5*(-B-math.sqrt(B**2-4*C))

            return [x, y, min(z1, z2)]

        if math.isclose(th2, th3):
            # swap coordinates to avoid 0 division
            swap_flag = True
            A1, A2, A3 = self.knee_joint_coordinates(th2, th3, th1)
        else:
            swap_flag = False
            A1, A2, A3 = self.knee_joint_coordinates(th1, th2, th3)

        #  normal solution
        a1_ = 2*(A3-A1)
        a2_ = 2*(A3-A2)
        b1 = A3.dot(A3) - A1.dot(A1)
        b2 = A3.dot(A3) - A2.dot(A2)


        a1 = a1_[0]/a1_[2]-a2_[0]/a2_[2]
        a2 = a1_[1]/a1_[2]-a2_[1]/a2_[2]
        a3 = b2/a2_[2]-b1/a1_[2]
        a4 = -a2/a1
        a5 = -a3/a1
        a6 = (-a2_[0]*a4-a2_[1])/(a2_[2])
        a7 = (b2-a2_[0]*a5)/(a2_[2])

        a = a4**2+1+a6**2
        b = 2*a4*(a5-A1[0])-2*A1[1]+2*a6*(a7-A1[2])
        c = a5*(a5-2*A1[0])+a7*(a7-2*A1[2])+A1.dot(A1)-self.l**2

        y1 = (-b+math.sqrt(b**2-4*a*c))/(2*a)
        y2 = (-b-math.sqrt(b**2-4*a*c))/(2*a)

        x1 = a4*y1+a5
        z1 = a6*y1+a7
        x2 = a4*y2+a5
        z2 = a6*y2+a7

        # forcing downmost solution
        if z1 < z2:
            result = np.array([-x1, -y1, z1])
        else:
            result = np.array([-x2, -y2, z2])

        # reverse rotation caused by coordinate swap
        if swap_flag:
            w = 2*math.pi/3
            ZRot = np.array([[math.cos(w), -math.sin(w), 0], [math.sin(w), math.cos(w), 0], [0, 0, 1]])
            result = ZRot.dot(result)

        return result


    def forward_kinematics(self, th1, th2, th3, phi, precision=4):
        BOVec = np.array([0, 0, -self.bZO])
        ToolZRot = np.array([[math.cos(phi), -math.sin(phi), 0], [math.sin(phi), math.cos(phi), 0], [0, 0, 1]])
        I = np.array(self._internal_forward_kinematics(th1, th2, th3))
        ExtP = self.baseVec + BOVec + I + ToolZRot.dot(self.toolVec)
        return list(map(lambda x: round(x,precision), [ExtP[0], ExtP[1], ExtP[2], phi]))



class RobotFactory:
    def _deserialize_from_json(self, file_name:str) -> dict:
        with open(file_name, "r") as file:
            raw = json.loads(file.read())
        try:
            data = {k:float(raw[k]) for k in ['bZ', 'wB', 'L', 'l', 'wP', 'pZ']}
        except Error as e:
           raise ValueError("Could not convert dimention data to float", e) 
        return data

    def _get_deserializer(self, format:str):
        if format == 'JSON':
            return self._deserialize_from_json
        else:
            raise ValueError(fromat)

    def deserialize(self, format:str, file_name:str) -> list:
        deserializer = self._get_deserializer(format)
        return deserializer(file_name)

    def _str2arr(self, val:str) -> np.array:
       return np.array([float(v) for v in val.split(", ")]) 

    def create_from_file(self, format:str, file_name:str) -> Robot:
        data = self.deserialize(format, file_name)
        #TODO: Data validation
        try:
            robot = Robot(
                data['bZ'],
                data['wB'],
                data['L'],
                data['l'],
                data['wP'],
                data['pZ']
            ) #TODO: dodać multiplikatory kątów obrotu w osiach napędowych
        except KeyError as e:
            raise KeyError(f"Missing data in file! {e}")

        if 'baseVec' in data.keys():
            robot.attach_base(self._str2arr(data['baseVec']) if isinstance(str, data['baseVec']) else np.array(data['baseVec']))
        if 'toolVec' in data.keys():
            robot.attach_tool(self._str2arr(data['toolVec']) if isinstance(str, data['toolVec']) else np.array(data['toolVec']))
        return robot

