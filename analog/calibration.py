import json 
import numpy as np
import time
import Adafruit_PCA9685
import math
import smbus
import struct
from BNO055 import *
from subprocess import Popen, PIPE

pwn = Adafruit_PCA9685.PCA9685()

tibia_min = 275 #325 according to json file 
tibia_max = 400 #425
chassis_min = 255 #275
chassis_max = 350 #475



#the file path for the gait generator file. Uncomment the one you want to use
#gaitGenerator = './splinegen.py'
#gaitGenerator = './Spyndra_newCode/standinggait.py'
gaitGenerator = './standingGait.py'

pwn.set_pwm_freq(60)	#Sets frequency to 60 Hz

#Pulls Min and Max values for femurs and tibias from the json file
def pullMotorVal():
	json_data = open('/home/pi/Desktop/SpyndraSpy/project/Spyndra_Control/Spyndra_newCode/servo_settings.json').read()
	parsed_json = json.loads(json_data)
	tibia_min = parsed_json['analog tibia min']
	tibia_max = parsed_json['analog tibia max']
	chassis_min = parsed_json['analog chassis min']
	chassis_max = parsed_json['analog chassis max']

#Outputs the motor signals to the motors from the splinegen arrays
#def outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum):
def outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum):
	pwn.set_pwm(chassisNum, 0, int(chassisOutput))
	pwn.set_pwm(tibiaNum, 0, int(tibiaOutput))
 
#Outputs the splines according to a phase
def splineRunner(chassis, tibia, phase, type, bno):

    
	for chassisCounter in range(1):
            for angleCounter in range(450,300,-1):
		    chassisOutput = angleCounter
		    #tibiaOutput = tibia[leg1_counter]*(tibia_max-tibia_min)+tibia_min
		    outputMotor(chassisOutput, tibiaOutput, 0, 1)
		    currentMatrix = produceVector(bno)
			
                    time.sleep(0.01)			

#calls gaitGenerator file and receives arrays from pipeline
def obtainGait():
	process = Popen(['python',gaitGenerator], stdout=PIPE, stderr=PIPE)
	stdout, stderr = process.communicate()
	return stdout

def startIMUdata():
        process = Popen(['python',IMUGenerator], stdout=PIPE, stderr=PIPE)
        return process

#make spyndra stand up before running
def spyndraStand(bno):
	startFemur = chassis_min
	startTibia = tibia_max
	#motor4Femur = 370;
	outputMotor(startFemur, startTibia, 0, 1)
	outputMotor(startFemur, startTibia, 2, 3)	
	outputMotor(startFemur, startTibia, 4, 5)
	outputMotor(startFemur, startTibia, 6, 7)
	while startTibia < tibia_max:
		startTibia += 1
		outputMotor(startFemur, startTibia, 0, 1)
		outputMotor(startFemur, startTibia, 2, 3)
		outputMotor(startFemur, startTibia, 4, 5)
		outputMotor(startFemur, startTibia, 6, 7)
		time.sleep(0.01)
		produceVector(bno)

def flatIMUdata(bno):
        flatCounter = 0
        nDataPoints = 10
        matrixData = np.zeros(shape=(nDataPoints,2,3))
        
        while flatCounter <nDataPoints:
                currentVectors = produceVector(bno)
                matrixData[flatCounter,0:2,0:3] = currentVectors
                flatCounter= flatCounter+1
                
        return numpy.median(matrixData,axis=0)

def calibrateTibia(bno):
	startFemur = 440
	startTibia = 225
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
        print(zod_gyro) #the first vector value is perpendicular to table
        print('\n')
        print(zod_accel)
        flat_gyro =zod_gyro

        maxChange= .1

        flat_gyro = zod_gyro

        tibiaCalibrated = np.zeros(shape=(4,1))
        nTibia = 4 #we have 4 tibias
        tibiaCalValue = -1 #if any calibrated value is returned as -1, we've hit an error

        for iTibia in range(0,nTibia):
                tibiaIncrementer = startTibia
                while tibiaIncrementer < tibia_max:
                        tibiaIncrementer += 1
                        outputMotor(startFemur, tibiaIncrementer, iTibia*2, iTibia*2+1)
                	time.sleep(0.01)
                        currentV = produceVector(bno)
                        diff = flat_gyro - currentV[0]
                        #print diff
                        #print ("\n")
                        #print(np.linalg.norm(diff))
                        if (np.linalg.norm(diff) > .05):
                                diff2 = flat_gyro-(produceVector(bno)[0])
                                #print("gate alpha")
                                if (np.linalg.norm(diff2)>.05):
                                        diff3 = flat_gyro-(produceVector(bno)[0])
                                        #print("gate beta")
                                        if (np.linalg.norm(diff3)>.05):
                                                tibiaCalValue = tibiaIncrementer
                                                #print("gate Delta")
                                                tibiaIncrementer = 1000 #escape the while loop
                                                print(np.linalg.norm(diff))
                                                print("calibrated!")

                tibiaCalibrated[iTibia] = tibiaCalValue
                outputMotor(startFemur, startTibia, 0, 1)
        	outputMotor(startFemur, startTibia, 2, 3)	
        	outputMotor(startFemur, startTibia, 4, 5)
        	outputMotor(startFemur, startTibia, 6, 7)
        	time.sleep(3)

        	zod_matrix = flatIMUdata(bno)
                zod_gyro = zod_matrix[0]
                zod_accel = zod_matrix[1]
                print(zod_gyro) #the first vector value is perpendicular to table
                print('\n')
                print(zod_accel)
                flat_gyro =zod_gyro
                time.sleep(3)
                
        print(tibiaCalibrated)

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
        flat_gyro =zod_gyro[0:3]

        maxChange= .1



        femurCalibrated = np.zeros(shape=(4,1))
        nFemur = 4 #we have 4 tibias
        femurCalValue = -1 #if any calibrated value is returned as -1, we've hit an error
        diffThreshold = 8.5

        for iFemur in range(0,nFemur):
                femurIncrementer = startFemur
                while femurIncrementer < chassis_max:
                        femurIncrementer = 1 + femurIncrementer
                        outputMotor(femurIncrementer, startTibia, iFemur*2, iFemur*2+1)
                	time.sleep(0.01)
                        currentV = produceVector(bno)
                        diff = flat_gyro - currentV[0][0:3]
                        #print diff
                        #print ("\n")
                        print(np.linalg.norm(diff))
                        if (np.linalg.norm(diff) > diffThreshold):
                                diff2 = flat_gyro-(produceVector(bno)[0][0:3])
                                #print("gate alpha")
                                if (np.linalg.norm(diff2)> diffThreshold):
					currentVector3 = produceVector(bno)[0][0:3]
                                        diff3 = flat_gyro-currentVector3
                                        #print("gate beta")
                                        if (np.linalg.norm(diff3)>diffThreshold):
                                                femurCalValue = femurIncrementer
                                                #print("gate Delta")
                                                femurIncrementer = chassis_max#escape the while loop
                                                print(np.linalg.norm(diff3))
						print(flat_gyro)
						print(currentVector3)
                                                print("calibrated!")
						

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
                flat_gyro =zod_gyro[0:3]
                time.sleep(3)
                
        print(femurCalibrated)
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

pullMotorVal()

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

calibrateFemur(bno)		

