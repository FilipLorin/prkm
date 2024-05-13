from dataclasses import dataclass
import json


@dataclass
class Vec:
    x: float
    y: float
    z: float


@dataclass
class Robot:
    bZO: float
    bR: float
    a: float
    l: float
    fR: float
    tH: float
    baseVec: Vec
    toolVec: Vec


def readDimentions(fileName:str="dimentions.json", id:str="robot"):
    with open(fileName, "r") as file:
        data = json.loads(file.read())
    #TODO: Data validation
    return data[id]


def setup():
    pass
