import numpy as np
import math
import smbus
import struct
from BNO055 import *


def produceVector(bno):
    rPlace = 1000
    matrixData = np.zeros(shape=(2,3))

    #grav = bno.getVector(BNO055.VECTOR_GRAVITY)
    #euler = bno.getVector(BNO055.VECTOR_EULER)
    #gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)

    #grav_r = (math.ceil(grav[0]*rPlace)/rPlace,math.ceil(grav[1]*rPlace)/rPlace,math.ceil(grav[2]*rPlace)/rPlace)
    #gyro_r = (math.ceil(gyro[0]*rPlace)/rPlace,math.ceil(gyro[1]*rPlace)/rPlace,math.ceil(gyro[2]*rPlace)/rPlace)
    #euler_rounded = (math.ceil(euler[0]*rPlace)/rPlace,math.ceil(euler[1]*rPlace)/rPlace, math.ceil(euler[2]*rPlace)/rPlace)
 
    
    #matrixData[0,0:3] = gyro_r
    #matrixData[0,0:3] = euler_rounded
    #matrixData[1,0:3] = grav_r

    # data preparation
    grav = [1,2,3]
    euler = [-1,-2,-3]
    # round the values to the thousandth place
    rPlace = 1000
    grav_rounded  = [math.ceil(x*rPlace)/rPlace for x in grav]
    euler_rounded = [math.ceil(x*rPlace)/rPlace for x in euler]
    euler_rounded[0] = (abs(euler_rounded[0]) - 360) if euler_rounded[0] > 180 else euler_rounded[0]

    return grav_rounded, euler_rounded