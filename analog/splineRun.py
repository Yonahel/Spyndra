from BNO055 import *
from datetime import datetime
import json 
import numpy as np
import time
import Adafruit_PCA9685
import csv
from subprocess import Popen, PIPE

pwm = Adafruit_PCA9685.PCA9685()

#saves gyro and accel data
IMU_flatMatrix = np.zeros(shape=(2,3))

#temp IMU storage variables
IMU_allData = [IMU_flatMatrix]
IMU_alData_centered = [IMU_flatMatrix]

#output file to write IMU data to
today=datetime.now()
fileName = today.isoformat()
IMU_file = './IMU_data/' + fileName + '.csv'
#csvfile = open(IMU_file,'a')
#IMU_output = csv.writer(csvfile, delimiter = ' ')


#the file path for the gait generator file. Uncomment the one you want to use
#gaitGenerator = './splineGen.py'
#gaitGenerator = './standingGait.py'
#gaitGenerator = './manualGait.py'

pwm.set_pwm_freq(60)	#Sets frequency to 60 Hz

motor0_min = 250
motor0_max = 300
motor1_min = 250
motor1_max = 300
motor2_min = 250
motor2_max = 300
motor3_min = 250
motor3_max = 300
motor4_min = 250
motor4_max = 300
motor5_min = 250
motor5_max = 300
motor6_min = 250
motor6_max = 300
motor7_min = 250
motor7_max = 300

#Pulls Min and Max values for femurs and tibias from the json file
def pullMotorVal(motorType):
	if(motorType == 1):
		json_data = open('../servo_settings.json').read()
		parsed_json = json.loads(json_data)
		motor9_min = parsed_json['motor 0 min']
		motor0_max = parsed_json['motor 0 max']
		motor1_min = parsed_json['motor 1 min']
		motor1_max = parsed_json['motor 1 max']
		motor2_min = parsed_json['motor 2 min']
		motor2_min = parsed_json['motor 2 max']
		motor3_min = parsed_json['motor 3 min']
		motor3_max = parsed_json['motor 3 max']
		motor4_min = parsed_json['motor 4 min']
		motor4_max = parsed_json['motor 4 max']
		motor5_min = parsed_json['motor 5 min']
		motor5_max = parsed_json['motor 5 max']
		motor6_min = parsed_json['motor 6 min']
		motor6_max = parsed_json['motor 6 max']
		motor7_min = parsed_json['motor 7 min']
		motor7_max = parsed_json['motor 7 max']
	elif(motorType == 2):
		json_data = open('./servo_settings.json').read()
		parsed_json = json.loads(json_data)
		tibia_min = parsed_json['digital tibia min']
		tibia_max = parsed_json['digital tibia max']
		chassis_min = parsed_json['digital chassis min']
		chassis_max = parsed_json['digital chassis max']
   
#Outputs the motor signals to the motors from the splinegen arrays
def outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum):
	pwm.set_pwm(chassisNum, 0, int(chassisOutput))
	pwm.set_pwm(tibiaNum, 0, int(tibiaOutput))
 
#Outputs the splines according to a phase
def splineRunner(chassis, tibia, phase, type, bno, flatMatrix, dataIMU):


        if dataIMU==1:
                tick = datetime.now()

                #with open(IMU_file, 'a') as csvfile:
                #        IMU_writer = csv.writer(csvfile, delimiter = ' ')
                #        IMU_writer.writerow("The flat matrix is " + str(flatMatrix) + " \n")
                
                with open(IMU_file, 'a') as csvfile:
                        IMU_writer = csv.writer(csvfile, delimiter = ' ')
                        flatMatrixWrite = [flatMatrix[0][0], flatMatrix[0][1], flatMatrix[0][2], flatMatrix[1][0],  flatMatrix[1][1], flatMatrix[1][2]]
                        IMU_writer.writerow(flatMatrixWrite)

                # IMU_output.writerow("The flat matrix is " + str(flatMatrix) + " \n")

                #this determines the number of times the IMU reads data
                IMU_cycleThreshold = 5
        	IMU_cycleCounter = 1
        
	leg1_counter = ((4.0*phase)/360.0)*len(chassis)
	leg2_counter = ((3.0*phase)/360.0)*len(chassis)
	leg3_counter = ((2.0*phase)/360.0)*len(chassis)
	leg4_counter = ((1.0*phase)/360.0)*len(chassis)

	#In the case of a phase greater than 180, leg1 and leg2 must be corrected back to 0 and 180 degrees
	if(phase >= 180):
		leg2_counter = ((1.0*phase)/360.0)*len(chassis)
		leg1_counter = ((2.0*phase)/360.0)*len(chassis)
		

	for i in range(3):
		for i in range(len(chassis)):
			if leg1_counter >= len(chassis):
				leg1_counter -= len(chassis)
			if leg2_counter >= len(chassis):
				leg2_counter -= len(chassis)
			if leg3_counter >= len(chassis):
				leg3_counter -= len(chassis)
			if leg4_counter >= len(chassis):
				leg4_counter -= len(chassis)

                        #determines whether the IMU will read in the current cycle
			if dataIMU==1:
                                if IMU_cycleCounter == IMU_cycleThreshold:
                                        IMU_cycleCounter = 1
                                else:
                                        IMU_cycleCounter+= 1
			
			#run for percentages
			if type == 1 or type == 2:
				chassisOutput1 = chassis[leg1_counter]*(motor0_max-motor0_min) + motor0_min
				tibiaOutput1 = tibia[leg1_counter]*(motor1_max-motor1_min)+motor1_min
				
				chassisOutput2 = chassis[leg2_counter]*(motor2_max-motor2_min) + motor2_min
				tibiaOutput2 = tibia[leg2_counter]*(motor3_max-motor3_min)+motor3_min
	
				chassisOutput3 = chassis[leg3_counter]*(motor4_max-motor4_min) + motor4_min
				tibiaOutput3 = tibia[leg3_counter]*(motor5_max-motor5_min)+motor5_min
	
				chassisOutput4 = chassis[leg4_counter]*(motor6_max-motor6_min) + motor6_min
				tibiaOutput4 = tibia[leg4_counter]*(motor7_max-motor7_min)+motor7_min	

				outputMotor(chassisOutput1, tibiaOutput1, 0, 1)
				outputMotor(chassisOutput2, tibiaOutput2, 2, 3)					
				outputMotor(chassisOutput3, tibiaOutput3, 4, 5)
				outputMotor(chassisOutput4, tibiaOutput4, 6, 7)

				#read data from IMU
				if dataIMU==1:
                                        if IMU_cycleCounter == IMU_cycleThreshold:
                                                currentTimeD = tick - datetime.now()
                                                currentReading, errorCounter = produceVector(bno)
                                                currentEditedReading = currentReading - flatMatrix

                                                dataWrite = [currentTimeD.total_seconds(), currentEditedReading[0][0], currentEditedReading[0][1], currentEditedReading[0][2], currentEditedReading[1][0], currentEditedReading[1][1], currentEditedReading[1][2], chassisOutput1, tibiaOutput1, chassisOutput2, tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4] 
                                                
                                                #IMU_output.writerow(str(currentTimeD.total_seconds()) + " ")
                                                #IMU_output.write(str(currentReading[0])+ ' ' + str(currentReading[1]))
                                                #IMU_output.writerow(str(currentEditedReading[0]) + ' ' + str(currentEditedReading[1]))

                                                #the below line also outputs tibia and femur positions
                                                #IMU_output.writerow(str(chassisOutput1) + " " + str(tibiaOutput1) + " ")
                                                #IMU_output.writerow(str(chassisOutput2) + " " + str(tibiaOutput2) + " ")
                                                #IMU_output.writerow(str(chassisOutput3) + " " + str(tibiaOutput3) + " ")
                                                #IMU_output.writerow(str(chassisOutput4) + " " + str(tibiaOutput4) + " ")

                                                with open(IMU_file, 'a') as csvfile:
                                                        IMU_writer = csv.writer(csvfile, delimiter = ' ')
                                                        IMU_writer.writerow(dataWrite)
                                                #IMU_output.writerow('\n')

					
			
			#run for motor angles
			elif type == 3:
				chassisOutput1 = chassis[leg1_counter]
				tibiaOutput1 = tibia[leg1_counter]
				
				chassisOutput2 = chassis[leg2_counter]
				tibiaOutput2 = tibia[leg2_counter]

				chassisOutput3 = chassis[leg3_counter]
				tibiaOutput3 = tibia[leg3_counter]

				chassisOutput4 = chassis[leg4_counter]
				tibiaOutput4 = tibia[leg4_counter]	
				
				outputMotor(chassisOutput1, tibiaOutput1, 0, 1)
				outputMotor(chassisOutput2, tibiaOutput2, 2, 3)	
				outputMotor(chassisOutput3, tibiaOutput3, 4, 5)
				outputMotor(chassisOutput4, tibiaOutput4, 6, 7)

                                #read data from IMU
				if dataIMU==1:
                                        if IMU_cycleCounter == IMU_cycleThreshold:
                                                currentTimeD = tick - datetime.now()
                                                currentReading, errorCounter = produceVector(bno)
                                                currentEditedReading = currentReading - flatMatrix

                                                dataWrite = [currentTimeD.total_seconds(), currentEditedReading[0][0], currentEditedReading[0][1], currentEditedReading[0][2], currentEditedReading[1][0], currentEditedReading[1][1], currentEditedReading[1][2], chassisOutput1, tibiaOutput1, chassisOutput2, tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4] 
                                                
                                                #IMU_output.writerow(str(currentTimeD.total_seconds()) + " ")
                                                #IMU_output.write(str(currentReading[0])+ ' ' + str(currentReading[1]))
                                                #IMU_output.writerow(str(currentEditedReading[0]) + ' ' + str(currentEditedReading[1]))

                                                #the below line also outputs tibia and femur positions
                                                #IMU_output.writerow(str(chassisOutput1) + " " + str(tibiaOutput1) + " ")
                                                #IMU_output.writerow(str(chassisOutput2) + " " + str(tibiaOutput2) + " ")
                                                #IMU_output.writerow(str(chassisOutput3) + " " + str(tibiaOutput3) + " ")
                                                #IMU_output.writerow(str(chassisOutput4) + " " + str(tibiaOutput4) + " ")

                                                with open(IMU_file, 'a') as csvfile:
                                                        IMU_writer = csv.writer(csvfile, delimiter = ' ')
                                                        IMU_writer.writerow(dataWrite)
                                                #IMU_output.writerow('\n')

			leg1_counter+=1
			leg2_counter+=1
			leg3_counter+=1
			leg4_counter+=1

#function to move motors from standing to first. TODO : Implement
#def goToPoint(chassisOutput, tibiaOutput, startFemur, startTibia):
#	for i in range(8):
#		if i == 0 or i%2 == 0:
#			while(startFemur != int(chassisOutput[i/2])):
#				pwm.set_pwm(i, 0, int(startFemur))
#				if (startFemur < chassisOutput[i/2]):
#					startFemur+=1
#				elif (startFemur > chassisOutput[i/2]):
#					startFemur-=1					
#				time.sleep(0.005)
#		else:
#			while(startTibia != int(tibiaOutput[(i-1)/2])):
#				pwm.set_pwm(i, 0, int(startTibia))
#				if (startTibia < tibiaOutput[(i-1)/2]):
#					startTibia+=1
#				elif (startTibia > tibiaOutput[(i-1)/2]):
#					startTibia-=1
#				time.sleep(0.005)
#
#					
#		startFemur = 255
#		startTibia = 275					

	

#Stands Spyndra up before spline execution
def spyndraStand():
	startFemur = 255
	startTibia = 570
	outputMotor(startFemur, startTibia, 0, 1)
	outputMotor(startFemur, startTibia, 2, 3)	
	outputMotor(startFemur, startTibia, 4, 5)
	outputMotor(startFemur, startTibia, 6, 7)
	while startTibia > 275:
		startTibia += -1
		outputMotor(startFemur, startTibia, 0, 1)
		outputMotor(startFemur, startTibia, 2, 3)
		outputMotor(startFemur, startTibia, 4, 5)
		outputMotor(startFemur, startTibia, 6, 7)
		time.sleep(0.001) 
		
#Sits Spyndra back down after spline execution
def spyndraSit():	
	endFemur = 255
	endTibia = 275
	outputMotor(endFemur, endTibia, 0, 1)
	outputMotor(endFemur, endTibia, 2, 3)
	outputMotor(endFemur, endTibia, 4, 5)
	outputMotor(endFemur, endTibia, 6, 7)
	while endTibia < 550:
		endTibia += 1
		outputMotor(endFemur, endTibia, 0, 1)
		outputMotor(endFemur, endTibia, 2, 3)
		outputMotor(endFemur, endTibia, 4, 5)
		outputMotor(endFemur, endTibia, 6, 7)
		time.sleep(0.01)

#turns on IMU
def IMUstart():
	bno = BNO055()
	if bno.begin() is not True:
		print "Error initializing IMU"
		exit()
	time.sleep(1)
	bno.setExternalCrystalUse(True)
	return bno

def flatIMUdata(bno):
	flatCounter = 0

	#number of datapoints we get to find definition of flat
	nDataPoints = 10

	IMU_tempFlatData = np.zeros(shape=(nDataPoints,2,3))
	
	#we compute the IMU flat Data nDataPoints times and return the 
	#median of these. This is done to refer small jump errors
	while flatCounter < nDataPoints:
		currentVectors, errorCounter = produceVector(bno)
		IMU_tempFlatData[flatCounter,0:2,0:3] = currentVectors
		flatCounter = flatCounter+1
	
	return numpy.median(IMU_tempFlatData,axis=0)

#computes the current gyro and accelerometer readings from the IMU
def produceVector(bno, errorCounter = 0):

	#we round the values to the thousandth place
	rPlace = 1000
	matrixData = np.zeros(shape=(2,3))
	
	#we obtain the current vectors using the BN0O55 function getVector
	#euler = degrees of yaw, pitch, and roll
	#grav = acceleration in x, y and z direction (TODO: double check if that's true)
	grav = bno.getVector(BNO055.VECTOR_GRAVITY)
	euler = bno.getVector(BNO055.VECTOR_EULER)
	
	#we round the values to the rPlace
	grav_rounded = (math.ceil(grav[0]*rPlace)/rPlace,math.ceil(grav[1]*rPlace)/rPlace,math.ceil(grav[2]*rPlace)/rPlace)
        euler_rounded = (math.ceil(euler[0]*rPlace)/rPlace,math.ceil(euler[1]*rPlace)/rPlace, math.ceil(euler[2]*rPlace)/rPlace)

	matrixData[0,0:3] = euler_rounded
	matrixData[1,0:3] = grav_rounded
	
	#This error correction technique below has been shown to be wasteful.
	#Euler angles should be from 0 to 360. I've been getting weird errors in which they
	#jump to ~1000, so I just recompute in that case. 
	#if ((euler_rounded[0] > 360) or (euler_rounded[1]>360) or (euler_rounded[2]>360)):
        #        return produceVector(bno, errorCounter+1)

	#An euler angle of 359 is only 2 degrees from an euler angle of 1 (they wrap around 360)
	#to make the two values equivalent, I artificially make the euler angle return value
	#into the absolute difference from 180 (sadly, this does mean we lose information on 
	#direction of tilt. I only do this to the yaw euler angle because it only becomes
	#a problem with yaw when measuring balance.
        if (euler_rounded[0]>180):
                matrixData[0][0] = abs(euler_rounded[0]-360)

        return matrixData, errorCounter




##Main Program Execution##

#Queries user for y/n data collection
dataIMU = input("Type 1 if IMU data collection desired, 0 if not \n (NOTE: only works  if IMU installed): ")

if dataIMU==1:
        #with open(IMU_file,'w') as csvfile:
        #        IMU_output = csv.writer(csvfile, delimiter = ' ')
        #start by turning on IMU
        bno = IMUstart()
else:
        bno=0
        flat_matrix=0

#calls gaitGenerator file and receives arrays from pipeline
def obtainGait():
	process = Popen(['python',gaitGenerator], stdout=PIPE, stderr=PIPE)
	stdout, stderr = process.communicate()
	return stdout


#Queries user for motor type (TODO: Remove this once motors finalized)
motorType = input("Type 1 if Analog Servo, 2 if Digital Servo: ")

#Queries user for type of gait input
type = input("Type 1 for Random Gait, 2 for Standing Gait, 3 for Manual Gait: ")
if(type == 1):
	gaitGenerator = './splineGen.py'
elif(type == 2):
	gaitGenerator = './standingGait.py'
elif(type == 3):
	gaitGenerator = './manualGait.py'
	f = open('manual.txt', 'w')
	fem_num = input("Enter how many femur coordinates you want: ")
	for i in range(int(fem_num)):
		n = input("Number: ")
		f.write(str(n))
		f.write(' ')
	f.write('\n')
	tib_num = input("Enter how many tibia coordinates you want: ")
	for i in range(int(tib_num)):
		n = input("Number: ")
		f.write(str(n))
		f.write(' ')
	f.write('\n')
	f.close()	

#Pulls motor extrema from json file
pullMotorVal(motorType)

gait = obtainGait()
gaitSave = gait

#Parses gait to pure numerical array
gait = gait.replace("]","")
gait = gait.replace("\n","")
gaitArray = gait.split('[')
chassis = np.fromstring(gaitArray[1],dtype=float,sep=" ")
tibia = np.fromstring(gaitArray[2],dtype=float,sep=" ")

#Queries user for input phase
phase = input('Enter phase between legs (in degrees): ')

if dataIMU==1:
        flat_matrix = flatIMUdata(bno)

#Robot Movement Command Sequence. Also evaluates time of execution of gait
spyndraStand()
starttime = time.time()
splineRunner(chassis, tibia, phase, type, bno, flat_matrix,dataIMU)
endtime = time.time()
elapsedtime = endtime - starttime
spyndraSit()

print("Duration of gait is " + str(elapsedtime) + " seconds.\n")


#if dataIMU==1:
#        IMU_output.close()
#csvfile.close()

#Saving a gait sequence from random gait generator
save = 0
if(type == 1):
	save = input("Would you like to save that run? (1 if yes, 0 if no): ")
if(save == 1):
	name = str(input("Input name for file: "))
	filePath = "./savedGaits/%s%s"(name,".json")
	temp = open('temp.json', 'r')
	parsed_json = json.loads(temp)
	femurSequence = parsed_json['Femur Sequence']
	tibiaSequence = parsed_json['Tibia Sequence']
	temp.close()
	target = open(filePath, 'w')
	json.dump({"Title": name,
		   "Femur IDs": [1,3,5,7],
		   "Tibia IDs": [2,4,6,8],
		   "Femur Sequence": femurSequence,
		   "Tibia Sequence": tibiaSequence,
		   "Offset": phase},
		   target, indent = 4, sort_keys = True, separators={',',':'})
	target.close()
