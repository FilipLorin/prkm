from dataclasses import dataclass
import json


@dataclass
class Vec:
    x: float
    y: float
    z: float


@dataclass
class Robot:
    bZO:float
    bR:float
    a:float
    l:float
    fR:float
    tH:float
    baseVec:Vec
    toolVec:Vec

    def attach_tool(self, tool:Vec):
        self.toolVec = tool

    def attach_base(self, base:Vec):
        self.baseVec = base


class RobotFactory:
    def _deserialise_from_json(self, file_name:str):
        with open(file_name, "r") as file:
            data = json.loads(file.read())
        return data

    def _get_deserialiser(self, format:str):
        if format == 'JSON':
            return self._deserialise_from_json
        else:
            raise ValueError(fromat)

    def deserialise(self, format:str, file_name:str):
        deserialiser = self._get_deserialiser(format)
        #TODO: Data validation
        return deserialiser(file_name, id)

    def create_from_file(self, format:str, file_name:str):
        data = deserialise(format, file_name)
    
