from dataclasses import dataclass
import json
import numpy as np
import math as m


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

    def attach_tool(self, tool:array):
        self.toolVec = tool

    def attach_base(self, base:array):
        self.baseVec = base

    def inverse_kinematics(self, x, y, z):
        a = self.wB - 2*self.wP
        b = 6*math.sqrt(3)*self.wP/2 - math.sqrt(3)/2*self.wB
        c = self.wP - 0.5*self.wB

        E1 = 2*self.L*(y+a)
        F = 2*z*self.L
        G1 = x**2+y**2+z**2+a**2+self.L**2+2*y*a-self.l**2

        E2 = -self.L*(math.sqrt(3)*(x+b)+y+c)
        G2 = x**2+y**2+z**2+b**2+c**2+self.L**2+2*(x*b+y*c)-self.l**2

        E3 = self.L*(math.sqrt(3)*(x-b)-y-c)
        G3 = x**2+y**2+z**2+b**2+c**2+self.L**2+2*(-x*b+y*c)-self.l**2

        # forcing "knee outwards" position
        t1 = (F-math.sqrt(E1**2+F**2-G1**2))/(G1-E1)
        t2 = (F-math.sqrt(E2**2+F**2-G2**2))/(G2-E2)
        t3 = (F-math.sqrt(E3**2+F**2-G3**2))/(G3-E3)

        th1 = 2*math.atan(t1)
        th2 = 2*math.atan(t2)
        th3 = 2*math.atan(t3)

        return [th1, th2, th3]


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

    def create_from_file(self, format:str = 'JSON', file_name:str = 'dimentions.json'):
        data = self.deserialise(format, file_name)
        #TODO: Data validation
        try:
            robot = Robot(float(data['baseZOffset']),
                      float(data['wB']),
                      float(data['L']),
                      float(data['l']),
                      float(data['wP']),
                      float(data['toolHeight']))
        except KeyError as e:
            raise KeyError(f"Missing data in file! {e}")

        if 'baseVec' in data.keys():
            #TODO: convert string to Vec
            robot.attach_base(data['baseVec'])
        if 'toolVec' in data.keys():
            #TODO: as above
            robot.attach_tool(data['toolVec'])
        return robot

