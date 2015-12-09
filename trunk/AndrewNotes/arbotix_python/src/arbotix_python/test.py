from pygame import *
from math import radians
from arbotix import *

Id = 6



a = ArbotiX()

pos = a.getPosition(Id)

print pos
