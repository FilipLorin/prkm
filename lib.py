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
    tH:float

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

    def _internal_inverse_kinematics(self, x:float, y:float, z:float, w:float):
        a = self.wB - 2*self.wP
        b = 3*math.sqrt(3)*self.wP - (math.sqrt(3)/2)*self.wB
        c = self.wP - 0.5*self.wB

        E1 = 2*self.L*(y+a)
        F = 2*z*self.L
        G1 = x**2+y**2+z**2+a**2+self.L**2+2*y*a-self.l**2

        E2 = -self.L*(math.sqrt(3)*(x+b)+y+c)
        G2 = x**2+y**2+z**2+b**2+c**2+self.L**2+2*(x*b+y*c)-self.l**2

        E3 = self.L*(math.sqrt(3)*(x-b)-y-c)
        G3 = x**2+y**2+z**2+b**2+c**2+self.L**2+2*(-x*b+y*c)-self.l**2

        # forcing "knee outwards" position
        t11 = (F-math.sqrt(E1**2+F**2-G1**2))/(G1-E1)
        t21 = (F-math.sqrt(E2**2+F**2-G2**2))/(G2-E2)
        t31 = (F-math.sqrt(E3**2+F**2-G3**2))/(G3-E3)
        
        t12 = (F+math.sqrt(E1**2+F**2-G1**2))/(G1-E1)
        t22 = (F+math.sqrt(E2**2+F**2-G2**2))/(G2-E2)
        t32 = (F+math.sqrt(E3**2+F**2-G3**2))/(G3-E3)

        th1 = self._minarg(2*math.atan(t11), 2*math.atan(t12))
        th2 = self._minarg(2*math.atan(t21), 2*math.atan(t22))
        th3 = self._minarg(2*math.atan(t31), 2*math.atan(t32))

        #TODO: check for invalid data

        return [th1, th2, th3, w]

    def inverse_kinematics(self, x, y, z, w):
        ExtP = np.array([x,y,z])
        ToolZRot = np.array([[math.cos(w), -math.sin(w), 0], [math.sin(w), math.cos(w), 0], [0, 0, 1]])
        I = self.baseVec - ExtP + ToolZRot.dot(self.toolVec)
        return self._internal_inverse_kinematics(I[0], I[1], I[2], w)


class RobotFactory:
    def _deserialise_from_json(self, file_name:str):
        with open(file_name, "r") as file:
            data = json.loads(file.read())
        #TODO: move type conversions here
        return data

    def _get_deserialiser(self, format:str):
        if format == 'JSON':
            return self._deserialise_from_json
        else:
            raise ValueError(fromat)

    def deserialise(self, format:str, file_name:str):
        deserialiser = self._get_deserialiser(format)
        return deserialiser(file_name)

    def _str2arr(self, val):
       return np.array(val.split(", ")) 

    def create_from_file(self, format:str = 'JSON', file_name:str = 'dimentions.json'):
        data = self.deserialise(format, file_name)
        #TODO: Data validation
        try:
            robot = Robot(float(data['baseZOffset']),
                      float(data['wB']),
                      float(data['L']),
                      float(data['l']),
                      float(data['wP']))
        except KeyError as e:
            raise KeyError(f"Missing data in file! {e}")

        if 'baseVec' in data.keys():
            robot.attach_base(self._str2arr(data['baseVec']))
        if 'toolVec' in data.keys():
            robot.attach_tool(self._str2arr(data['toolVec']))
        return robot

