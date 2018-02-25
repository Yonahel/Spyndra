import json 
import numpy as np
import time
import Adafruit_PCA9685
import math
from BNO055 import *
from subprocess import Popen, PIPE
import json



pwn = Adafruit_PCA9685.PCA9685()

#observed minimums to begin calibration 
tibia_min = 260 #325 according to json file 
tibia_max = 375 #425
chassis_min = 225 #275
chassis_max = 425 #475

saved_pwms = []
pwn.set_pwm_freq(60)	#Sets frequency to 60 Hz


#Outputs the motor signals to the motors 
def outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum)
	pwn.set_pwm(chassisNum, 0, int(chassisOutput))
	pwn.set_pwm(tibiaNum, 0, int(tibiaOutput))
 		
#starts IMU data collection
def startIMUdata():
        process = Popen(['python',IMUGenerator], stdout=PIPE, stderr=PIPE)
        return process

#produces the flat IMU vector
def flatIMUdata(bno):
        flatCounter = 0
        nDataPoints = 10
        matrixData = np.zeros(shape=(nDataPoints,2,3))
        
        #collects matrix of 10 data points for stability 
        while flatCounter <nDataPoints:
                currentVectors = produceVector(bno)
                matrixData[flatCounter,0:2,0:3] = currentVectors
                flatCounter= flatCounter+1
                
        return np.median(matrixData,axis=0)


#calibrate all four tibias 
def calibrateTibia(bno,femur_center):

    #beginning motor outputs
	startFemur = chassis_min
	startTibia = tibia_max +150 #offset 

	
    time.sleep(3)
    outputMotor(femur_center[0], startTibia, 0, 1)
	outputMotor(femur_center[1], startTibia, 2, 3)	
	outputMotor(femur_center[2], startTibia, 4, 5)
	outputMotor(femur_center[3], startTibia, 6, 7)
	time.sleep(3)


    zod_matrix = flatIMUdata(bno)   #obtain flat IMU 
    zod_gyro = zod_matrix[0]        #starting gyroscopic data
    zod_accel = zod_matrix[1]       #starting acceleration data
	print("Initial flat data (gyro and accel data):")
    print(zod_gyro)                 #the first vector value is perpendicular to table
    print('\n')
    #print(zod_accel)

    #only using roll and pitch
    flat_gyro =zod_gyro[1:3]

    maxChange= .1



    tibiaCalibrated = np.zeros(shape=(4,1))
    nTibia = 4 #we have 4 tibias
    tibiaCalValue = -1 #if any calibrated value is returned as -1, we've hit an error
    previous_diff = 0
    current_diff = 0
    delta_threshold = .05
    diff = 0
    delta =0
    change_threshold = 6

    for iTibia in range(0,nTibia):
            previous_diff = 0
            current_diff = 0
            change_counter = 0
            
            tibiaIncrementer = startTibia
            while tibiaIncrementer > tibia_min:
                    tibiaIncrementer = tibiaIncrementer-1
                    outputMotor(femur_center[iTibia],tibiaIncrementer, iTibia*2, iTibia*2+1)
            	next_pwm =0

            	while (change_counter < change_threshold) and (not next_pwm):
                            time.sleep(0.01)
                            currentV = produceVector(bno)
                            print(currentV[0][1:3])

                            previous_diff = current_diff
                            #only using rolls and pitch
                            current_diff = flat_gyro - currentV[0][1:3]

                            if tibiaIncrementer == startTibia-1:
                                    previous_diff = current_diff
                                    print("we should only see this once per cycle")

                    
                            delta = abs(current_diff-previous_diff)
                            print delta
                            #print ("\n")
                            #print(str(np.linalg.norm(delta)))
                            if (np.linalg.norm(delta) > delta_threshold):
                                    change_counter = change_counter+1
                                    #time.sleep(0.1)
                                    print(str(change_counter) + " for the boys back home")
                            else:
                                    change_counter =0
                                    next_pwm =1
                            if change_counter == change_threshold:
                                    next_pwm =1
                                    change_counter =0
            
                                    tibiaCalValue = tibiaIncrementer
                                    tibiaIncrementer = tibia_min
                                    #print("gate Delta")
                                    #femurIncrementer = chassis_max#escape the while loop
                                    #print(np.linalg.norm(delta))
                                    #print(delta)
				#print(flat_gyro)
				#print(currentVector3)
                                    #print("calibrated!")
					

            tibiaCalibrated[iTibia] = tibiaCalValue
            outputMotor(femur_center[0], startTibia, 0, 1)
        	outputMotor(femur_center[1], startTibia, 2, 3)	
        	outputMotor(femur_center[2], startTibia, 4, 5)
        	outputMotor(femur_center[3], startTibia, 6, 7)
        	time.sleep(3)

        	zod_matrix = flatIMUdata(bno)
                zod_gyro = zod_matrix[0]
                zod_accel = zod_matrix[1]
		print("next initial set of flat data")
                print(zod_gyro) #the first vector value is perpendicular to table
                print('\n')
                #print(zod_accel)
                #again, only using pitch and roll
                flat_gyro =zod_gyro[1:3]
                time.sleep(3)
                
        print(tibiaCalibrated)
        max_list = []
        min_list = []
        
        for j in range(0,4):
                currentValue = tibiaCalibrated[j]

                currentMax = currentValue-44 #based off diagram from paper
                currentMin = currentValue - 159 #based off diagram from paper
                min_list.append(currentMin)
                max_list.append(currentMax)
                

        return max_list,min_list


#calibrate each Femur                
def calibrateFemur(bno):
	startFemur = chassis_min
	startTibia = tibia_max + 175
	#motor4Femur = 370;

        time.sleep(3)
        outputMotor(startFemur, startTibia, 0, 1)
	outputMotor(startFemur, startTibia, 2, 3)	
	outputMotor(startFemur, startTibia, 4, 5)
	outputMotor(startFemur, startTibia, 6, 7)
	time.sleep(3)

        zod_matrix = flatIMUdata(bno)
        zod_gyro = zod_matrix[0]
        zod_accel = zod_matrix[1]
	print("Initial flat data (gyro and accel data):")
        print(zod_gyro) #the first vector value is perpendicular to table
        print('\n')
        #print(zod_accel)
        #only using roll and pitch
        flat_gyro =zod_gyro[1:3]

        maxChange= .1



        femurCalibrated = np.zeros(shape=(4,1))
        nFemur = 4 #we have 4 tibias
        femurCalValue = -1 #if any calibrated value is returned as -1, we've hit an error
        previous_diff = 0
        current_diff = 0
        delta_threshold = .05
        diff = 0
        delta =0
        change_threshold = 6

        for iFemur in range(0,nFemur):
                previous_diff = 0
                current_diff = 0
                change_counter = 0
                
                femurIncrementer = startFemur
                while femurIncrementer < chassis_max:
                        femurIncrementer = 1 + femurIncrementer
                        outputMotor(femurIncrementer, startTibia, iFemur*2, iFemur*2+1)
                	next_pwm =0

                	while (change_counter < change_threshold) and (not next_pwm):
                                time.sleep(0.01)
                                currentV = produceVector(bno)
                                print(currentV[0][1:3])

                                previous_diff = current_diff
                                #only using rolls and pitch
                                current_diff = flat_gyro - currentV[0][1:3]

                                if femurIncrementer == startFemur+1:
                                        previous_diff = current_diff
                                        print("we should only see this once per cycle")

                        
                                delta = abs(current_diff-previous_diff)
                                print delta
                                #print ("\n")
                                #print(str(np.linalg.norm(delta)))
                                if (np.linalg.norm(delta) > delta_threshold):
                                        change_counter = change_counter+1
                                        #time.sleep(0.1)
                                        print(str(change_counter) + " for the boys back home")
                                else:
                                        change_counter =0
                                        next_pwm =1
                                if change_counter == change_threshold:
                                        next_pwm =1
                                        change_counter =0
                
                                        femurCalValue = femurIncrementer
                                        femurIncrementer = chassis_max
                                        #print("gate Delta")
                                        femurIncrementer = chassis_max#escape the while loop
                                        #print(np.linalg.norm(delta))
                                        #print(delta)
					#print(flat_gyro)
					#print(currentVector3)
                                        #print("calibrated!")
						

                femurCalibrated[iFemur] = femurCalValue
                outputMotor(startFemur, startTibia, 0, 1)
        	outputMotor(startFemur, startTibia, 2, 3)	
        	outputMotor(startFemur, startTibia, 4, 5)
        	outputMotor(startFemur, startTibia, 6, 7)
        	time.sleep(3)

        	zod_matrix = flatIMUdata(bno)
                zod_gyro = zod_matrix[0]
                zod_accel = zod_matrix[1]
		print("next initial set of flat data")
                print(zod_gyro) #the first vector value is perpendicular to table
                print('\n')
                #print(zod_accel)
                #again, only using pitch and roll
                flat_gyro =zod_gyro[1:3]
                time.sleep(3)
                
        print(femurCalibrated)

        femurCenter = []
        femurMax = []
        femurMin = []
        for j in range(0,4):
                currentValue = femurCalibrated[j]
                currentCenter = currentValue - 48 #observationally based
                currentMax = currentCenter+70 #based off diagram from paper
                currentMin = currentCenter - 30 #based off diagram from paper

                femurCenter.append(currentCenter)
                femurMax.append(currentMax)
                femurMin.append(currentMin)

        return femurCenter,femurMax,femurMin
        
def produceVector(bno):
        rPlace = 1000
        matrixData = np.zeros(shape=(2,3))
        testCounter = 1

        grav = bno.getVector(BNO055.VECTOR_GRAVITY)
        euler = bno.getVector(BNO055.VECTOR_EULER)
        #gyro = bno.getVector(BNO055.VECTOR_GYROSCOPE)

        grav_r = (math.ceil(grav[0]*rPlace)/rPlace,math.ceil(grav[1]*rPlace)/rPlace,math.ceil(grav[2]*rPlace)/rPlace)
        #gyro_r = (math.ceil(gyro[0]*rPlace)/rPlace,math.ceil(gyro[1]*rPlace)/rPlace,math.ceil(gyro[2]*rPlace)/rPlace)
        euler_rounded = (math.ceil(euler[0]*rPlace)/rPlace,math.ceil(euler[1]*rPlace)/rPlace, math.ceil(euler[2]*rPlace)/rPlace)


     
        #print gyro_r
        #print euler_rounded
        #print grav_r
        
        #matrixData[0,0:3] = gyro_r
        matrixData[0,0:3] = euler_rounded
        matrixData[1,0:3] = grav_r

	if ((euler_rounded[0] > 370) or (euler_rounded[1]>370) or (euler_rounded[2]>370)):
		return produceVector(bno)

	if (euler_rounded[0]>180):
		matrixData[0][0] = abs(euler_rounded[0]-360)

        return matrixData


        
#This block turns on and starts the IMU
bno = BNO055()
if bno.begin() is not True:
        print "Error initializing IMU"
        exit()
time.sleep(1)
bno.setExternalCrystalUse(True)

#pullMotorVal()

totalReturn = array([]);
tCounter = 0
rPlace = 1000


#type = input("Type 1 if inputting percents, 2 if inputting motor angles: ")

gait= obtainGait()
gait = gait.replace("]","")
gait = gait.replace("\n","")
gaitArray = gait.split('[')
#print(gait)
chassis = np.fromstring(gaitArray[1],dtype=float,sep=" ")
tibia = np.fromstring(gaitArray[2],dtype=float,sep=" ")
#phase = input('Enter phase between legs (in degrees): ')

#matrix used to zero imu inputs

#while True:
        
        #currentVector = produceVector(bno)
        #print("next" + "\n")
        #print abs(currentVector[0,1]-zod_gyro[1])
        #print abs(currentVector[0,2]-zod_gyro[2])
        #print(currentVector==zod_matrix)
        #time.sleep(.5)

femur_center,femur_maxes, femur_mins=calibrateFemur(bno)
#femur_center = [260,249,249,246]
tibia_maxes, tibia_mins = calibrateTibia(bno,femur_center)

#json_data = {
#        "femur_max_values" : str(femur_maxes),
#        "femur_min_values" : str(femur_mins),
#        "tibia_max_values" : str(tibia_maxes),
#        "tibia_min_values" : str(tibia_mins)
#}
with open(json_filename,"w") as f:
        json.dump({"motor 0 min": str(int(femur_mins[0])), "motor 0 max": str(int(femur_maxes[0])),
		   "motor 1 min": str(int(tibia_mins[0])), "motor 1 max": str(int(tibia_maxes[0])),
		   "motor 2 min": str(int(femur_mins[1])), "motor 2 max": str(int(femur_maxes[1])),
		   "motor 3 min": str(int(tibia_mins[1])), "motor 3 max": str(int(tibia_maxes[1])),
		   "motor 4 min": str(int(femur_mins[2])), "motor 4 max": str(int(femur_maxes[2])),
		   "motor 5 min": str(int(tibia_mins[2])), "motor 5 max": str(int(tibia_maxes[2])),
		   "motor 6 min": str(int(femur_mins[3])), "motor 6 max": str(int(femur_maxes[3])),
		   "motor 7 min": str(int(tibia_mins[3])), "motor 7 max": str(int(tibia_maxes[3]))
		   },
		  f, sort_keys = True, indent = 4, separators=(',',':'))

#file name: ../servosettings.json


