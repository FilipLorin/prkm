from dataclasses import dataclass
import json
import numpy as np
import math as m


@dataclass
class Vec:
    x:float
    y:float
    z:float
    #TODO: Change this to numpy arrays


@dataclass
class Robot:
    bZO:float
    wB:float
    L:float
    l:float
    wP:float
    tH:float

    def __post_init__(self):
        self.toolVec = Vec(0,0,0)
        self.baseVec = Vec(0,0,0)

    def attach_tool(self, tool:Vec):
        self.toolVec = tool

    def attach_base(self, base:Vec):
        self.baseVec = base


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

factory = RobotFactory()

