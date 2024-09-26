#gensinewave
import math
from math import sin
# f = open("sinwave.txt", "w")
# f.write("{")
# step = 2*3.14159265358979323846 / 1024
# phase = 0
# for i in range(0, 1024):
#     f.write(str(127*sin(phase)) + ", ")
#     phase = phase + step
# f.write("}")


f = open("lfo.txt", "w")
f.write("{")
lfoPhase = 0
for i in range(0, 4400):
    f.write(str((sin(2*3.14159265358979323846*lfoPhase/4294967295)+1)/2) + ", ")
    lfoPhase = lfoPhase + 976128.9309


# void genflo(){
#   volatile float lfoPhase = 0;
#   for(uint32_t i = 0; i< 4400; i++){
#     lfowave[i] = ((sin(2*PI*lfoPhase / (UINT_MAX))+1)/2);
#     lfoPhase += LFO_STEP_SIZE;
#   }
# }