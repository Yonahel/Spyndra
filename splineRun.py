import json 
import numpy as np
import time
import Adafruit_PCA9685
from subprocess import Popen, PIPE

pwm = Adafruit_PCA9685.PCA9685()

tibia_min = 350
tibia_max = 450
chassis_min = 325
chassis_max = 450


#the file path for the gait generator file. Uncomment the one you want to use
gaitGenerator = './splinegen.py'
#gaitGenerator = './standinggait.py'


pwm.set_pwm_freq(60)	#Sets frequency to 60 Hz

#Pulls Min and Max values for femurs and tibias from the json file
def pullMotorVal():
	json_data = open('/home/pi/Desktop/SpyndraSpy/project/Spyndra_Control/servo_settings2.json').read()
	parsed_json = json.loads(json_data)
	tibia_min = parsed_json['tibia min']
	tibia_max = parsed_json['tibia max']
	chassis_min = parsed_json['chassis min']
	chassis_max = parsed_json['chassis max']

#Outputs the motor signals to the motors from the splinegen arrays
def outputMotor(chassisOutput, tibiaOutput, chassisNum, tibiaNum):
	pwm.set_pwm(chassisNum, 0, int(chassisOutput))
	pwm.set_pwm(tibiaNum, 0, int(tibiaOutput))
 
#Outputs the splines according to a phase
def splineRunner(chassis, tibia, phase, type):
	leg1_counter = ((4.0*phase)/360.0)*len(chassis)
	leg2_counter = ((3.0*phase)/360.0)*len(chassis)
	leg3_counter = ((2.0*phase)/360.0)*len(chassis)
	leg4_counter = ((1.0*phase)/360.0)*len(chassis)
	if(phase >= 180):
		leg2_counter = ((1.0*phase)/360.0)*len(chassis)
		leg1_counter = ((2.0*phase)/360.0)*len(chassis)
	for i in range(1):
		for i in range(len(chassis)):
			if leg1_counter >= len(chassis):
				leg1_counter -= len(chassis)
			if leg2_counter >= len(chassis):
				leg2_counter -= len(chassis)
			if leg3_counter >= len(chassis):
				leg3_counter -= len(chassis)
			if leg4_counter >= len(chassis):
				leg4_counter -= len(chassis)
			
			if type == 1:
				chassisOutput = chassis[leg1_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput = tibia[leg1_counter]*(tibia_max-tibia_min)+tibia_min
				outputMotor(chassisOutput, tibiaOutput, 0, 1)
				
				chassisOutput = chassis[leg2_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput = tibia[leg2_counter]*(tibia_max-tibia_min)+tibia_min
				outputMotor(chassisOutput, tibiaOutput, 2, 3)					
	
				chassisOutput = chassis[leg3_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput = tibia[leg3_counter]*(tibia_max-tibia_min)+tibia_min
				outputMotor(chassisOutput, tibiaOutput, 4, 5)
	
				chassisOutput = chassis[leg4_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput = tibia[leg4_counter]*(tibia_max-tibia_min)+tibia_min
				outputMotor(chassisOutput, tibiaOutput, 6, 7)

			elif type == 2:
				chassisOutput = chassis[leg1_counter]
				tibiaOutput = tibia[leg1_counter]
				outputMotor(chassisOutput, tibiaOutput, 0, 1)
				
				chassisOutput = chassis[leg2_counter]
				tibiaOutput = tibia[leg2_counter]
				outputMotor(chassisOutput, tibiaOutput, 2, 3)	

				chassisOutput = chassis[leg3_counter]
				tibiaOutput = tibia[leg3_counter]
				outputMotor(chassisOutput, tibiaOutput, 4, 5)

				chassisOutput = chassis[leg4_counter]
				tibiaOutput = tibia[leg4_counter]	
				outputMotor(chassisOutput, tibiaOutput, 6, 7)
			leg1_counter+=1
			leg2_counter+=1
			leg3_counter+=1
			leg4_counter+=1
			#time.sleep(0.01)			

#makes spyndra stand up before running
def spyndraStand():
	startFemur = 400
	startTibia = 225
	#motor4Femur = 370;
	outputMotor(startFemur, startTibia, 0, 1)
	outputMotor(startFemur, startTibia, 2, 3)	
	outputMotor(startFemur, startTibia, 4, 5)
	outputMotor(startFemur, startTibia, 6, 7)
	while startTibia < 450:
		startTibia += 1
		outputMotor(startFemur, startTibia, 0, 1)
		outputMotor(startFemur, startTibia, 2, 3)
		outputMotor(startFemur, startTibia, 4, 5)
		outputMotor(startFemur, startTibia, 6, 7)
		time.sleep(0.01) 
		
def spyndraSit():	
	endFemur = 400
	endTibia = 450
	#motor4Femur = 370
	outputMotor(endFemur, endTibia, 0, 1)
	outputMotor(endFemur, endTibia, 2, 3)
	outputMotor(endFemur, endTibia, 4, 5)
	outputMotor(endFemur, endTibia, 6, 7)
	while endTibia > 225:
		endTibia += -1
		outputMotor(endFemur, endTibia, 0, 1)
		outputMotor(endFemur, endTibia, 2, 3)
		outputMotor(endFemur, endTibia, 4, 5)
		outputMotor(endFemur, endTibia, 6, 7)
		time.sleep(0.01)
	

#calls gaitGenerator file and receives arrays from pipeline
def obtainGait():
	process = Popen(['python',gaitGenerator], stdout=PIPE, stderr=PIPE)
	stdout, stderr = process.communicate()
	return stdout


type = input("Type 1 if inputting percents, 2 if inputting motor angles: ")
pullMotorVal()
gait= obtainGait()
gait = gait.replace("]","")
gait = gait.replace("\n","")
gaitArray = gait.split('[')
chassis = np.fromstring(gaitArray[1],dtype=float,sep=" ")
tibia = np.fromstring(gaitArray[2],dtype=float,sep=" ")
phase = input('Enter phase between legs (in degrees): ')
#print(chassis)
#print(tibia)
spyndraStand()
splineRunner(chassis, tibia, phase, type)
spyndraSit()
