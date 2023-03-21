#gensinewave
import math
from math import sin
f = open("sinwave.txt", "w")
f.write("{")
step = 2*3.14159265358979323846 / 1024
phase = 0
for i in range(0, 1024):
    f.write(str(127*sin(phase)) + ", ")
    phase = phase + step
f.write("}")
