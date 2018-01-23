
import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d
import json

# prints (returns) the femur and tibia value in lists
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
    return f_femur_sample, f_tibia_sample



def manualGait(manualGaitFilePath):
	manual_femur = list()
	manual_tibia = list()
	# manual.txt
	f = open(manualGaitFilePath, 'r')
	man_fem = f.readline()
	man_tib = f.readline()
	man_fem = man_fem.replace("\n", "")
	man_tib = man_tib.replace("\n", "")
	manual_femur = man_fem.split(' ')
	manual_tibia = man_tib.split(' ')
	manual_femur.remove("")
	manual_tibia.remove("")
	manual_femur = map(int, manual_femur)
	manual_tibia = map(int, manual_tibia) 
	return spline_gen(manual_femur, manual_tibia, 5, 2)

def randomGait():
	def checkVelocityArr(arr):
	        if (abs(arr[4] - arr[3]) > 0.7) or (abs(arr[3] - arr[2]) > 0.7) or (abs(arr[2]-arr[1]) > 0.7) or (abs(arr[1] - arr[0]) > 0.7) or (abs(arr[0] - arr[4]) > 0.7):
	                arr = np.random.rand(5)
	                arr = checkVelocityArr(arr)
	                return arr	
		else:
	        	return arr
	#Traditional Random Gait
	random_femur = np.random.rand(5)
	random_tibia = np.random.rand(5)
	#random_femur = np.array([0.6642565, 0.93031099, 0.98387567, 0.55137283, 0.25167978])
	#random_tibia = np.array([0.22984195, 0.63087833, 0.65343093, 0.92651416, 0.54060193])
		
	# gaitFile = open('/home/pi/Desktop/SpyndraSpy/project/Spyndra_Control/Spyndra_newCode/analog/saved_gates/most_recent.txt','w')

	# #Test gait
	# gaitFile.write(str(random_femur) + '\n')
	# gaitFile.write(str(random_tibia) + '\n')

	random_femur = checkVelocityArr(random_femur)
	random_tibia = checkVelocityArr(random_tibia)

	a = random_femur.tolist()
	b = random_tibia.tolist()

	file = open('temp.json', 'w');
	key = {"Femur Sequence": a,
	       "Tibia Sequence": b}
	json.dump(key, file, sort_keys = True, indent = 4)
	file.close()
	return spline_gen(random_femur, random_tibia, 5, 2) #changed 2 to 10 cycles

def standingGait():
	standing_femur = [0.30, 0.50, 0.30, 0.10]
	standing_tibia = [0.20, 0.40, 0.20, 0.00]
	return spline_gen(standing_femur, standing_tibia, 5, 2)


#Stands Spyndra up before spline execution
def spyndraStand():
	startFemur = 255
	startTibia = 570
	femur = []
	tibia = []
	femur.append(startFemur)
	femur.append(startFemur)
	femur.append(startFemur)
	femur.append(startFemur)
	tibia.append(startTibia)
	tibia.append(startTibia)
	tibia.append(startTibia)
	tibia.append(startTibia)
	while startTibia > 275:
		startTibia += -1
		femur.append(startFemur)
		femur.append(startFemur)
		femur.append(startFemur)
		femur.append(startFemur)
		tibia.append(startTibia)
		tibia.append(startTibia)
		tibia.append(startTibia)
		tibia.append(startTibia)
	return femur, tibia
		
#Sits Spyndra back down after spline execution
def spyndraSit():	
	endFemur = 255
	endTibia = 275
	femur = []
	tibia = []
	femur.append(startFemur)
	femur.append(startFemur)
	femur.append(startFemur)
	femur.append(startFemur)
	tibia.append(startTibia)
	tibia.append(startTibia)
	tibia.append(startTibia)
	tibia.append(startTibia)
	while endTibia < 550:
		endTibia += 1
		femur.append(startFemur)
		femur.append(startFemur)
		femur.append(startFemur)
		femur.append(startFemur)
		tibia.append(startTibia)
		tibia.append(startTibia)
		tibia.append(startTibia)
		tibia.append(startTibia)
	return femur, tibia

