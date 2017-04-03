
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

#points array of values from 0-1 (to be mapped)


def spline_gen(points_femur, points_tibia, period, cycles):

        freq=60

        y_femur=np.array(points_femur)
        y_tibia=np.array(points_tibia)
        x_femur=np.linspace(0, period, num=len(y_femur))
        x_tibia=np.linspace(0, period, num=len(y_tibia))

        y_femur_iter=y_femur
        for i in range(cycles+2):
                y_femur=np.append(y_femur,y_femur_iter)
        y_tibia_iter=y_tibia
        for i in range(cycles+2):
                y_tibia=np.append(y_tibia,y_tibia_iter)

        #time array 1-1 correspondence to joint angles with period*(cycles+2) points
        x_femur=np.linspace(0, period*(cycles+2), num=len(y_femur))
        x_tibia=np.linspace(0, period*(cycles+2), num=len(y_tibia))

        #interpolate
        f_femur = interp1d(x_femur, y_femur, kind='cubic')
        f_tibia = interp1d(x_tibia, y_tibia, kind='cubic')

        #create high res time array without first and last cycle
        #something wrong, doesnt start at start point
        x_femur_cut=np.linspace(period, period*(cycles+2)-2.5, period*freq*cycles)
        x_tibia_cut=np.linspace(period, period*(cycles+2)-2.5, period*freq*cycles)
        #returns array of sampled function
        f_femur_sample=f_femur(x_femur_cut)
        f_tibia_sample=f_tibia(x_tibia_cut)

        #Graphical output for this. Comment out for outputting to Spyndra
        #print(f_femur_sample)
        #plt.plot(x_femur, y_femur, 'o', x_femur_cut, f_femur(x_femur_cut), '--')
        #plt.show()

	#Pipe
	#Pipe f_femur_sample, f_tibia_sample
	print(f_femur_sample)
	print(f_tibia_sample)	
        

def checkVelocityArr(arr):
        if (abs(arr[4] - arr[3]) > 0.5) or (abs(arr[3] - arr[2]) > 0.5) or (abs(arr[2]-arr[1]) > 0.5) or (abs(arr[1] - arr[0]) > 0.5) or (abs(arr[0] - arr[4]) > 0.5):
                arr = np.random.rand(5)
                arr = checkVelocityArr(arr)
                return arr	
	else:
        	return arr

	

random_femur = np.random.rand(5)
random_tibia = np.random.rand(5)
random_femur = checkVelocityArr(random_femur)
random_tibia = checkVelocityArr(random_tibia)
spline_gen(random_femur, random_tibia, 5, 2) #changed 2 to 10 cycles

