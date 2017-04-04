
import json 
import numpy as np
import time
import Adafruit_PCA9685
from subprocess import Popen, PIPE

pwm = Adafruit_PCA9685.PCA9685()

tibia_min = 200
tibia_max = 500
chassis_min = 200
chassis_max = 500


#the file path for the gait generator file. Uncomment the one you want to use
#gaitGenerator = './splineGen.py'
#gaitGenerator = './standingGait.py'
#gaitGenerator = './manualGait.py'

pwm.set_pwm_freq(60)	#Sets frequency to 60 Hz

#Pulls Min and Max values for femurs and tibias from the json file
def pullMotorVal(motorType):
	if(motorType == 1):
		json_data = open('../servo_settings.json').read()
		parsed_json = json.loads(json_data)
		tibia_min = parsed_json['analog tibia min']
		tibia_max = parsed_json['analog tibia max']
		chassis_min = parsed_json['analog chassis min']
		chassis_max = parsed_json['analog chassis max']
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
def splineRunner(chassis, tibia, phase, type):
	leg1_counter = ((4.0*phase)/360.0)*len(chassis)
	leg2_counter = ((3.0*phase)/360.0)*len(chassis)
	leg3_counter = ((2.0*phase)/360.0)*len(chassis)
	leg4_counter = ((1.0*phase)/360.0)*len(chassis)
	if(phase >= 180):
		leg2_counter = ((1.0*phase)/360.0)*len(chassis)
		leg1_counter = ((2.0*phase)/360.0)*len(chassis)
		
	flag = 1
	startFemur = 255
	startTibia = 275 

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
			
			#run for percentages
			if type == 1:
				chassisOutput1 = chassis[leg1_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput1 = tibia[leg1_counter]*(tibia_max-tibia_min)+tibia_min
				
				chassisOutput2 = chassis[leg2_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput2 = tibia[leg2_counter]*(tibia_max-tibia_min)+tibia_min
	
				chassisOutput3 = chassis[leg3_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput3 = tibia[leg3_counter]*(tibia_max-tibia_min)+tibia_min
	
				chassisOutput4 = chassis[leg4_counter]*(chassis_max-chassis_min) + chassis_min
				tibiaOutput4 = tibia[leg4_counter]*(tibia_max-tibia_min)+tibia_min	

				outputMotor(chassisOutput1, tibiaOutput1, 0, 1)
				outputMotor(chassisOutput2, tibiaOutput2, 2, 3)					
				outputMotor(chassisOutput3, tibiaOutput3, 4, 5)
				outputMotor(chassisOutput4, tibiaOutput4, 6, 7)
			
			#run for motor angles
			elif type == 2 or type == 3:
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


			leg1_counter+=1
			leg2_counter+=1
			leg3_counter+=1
			leg4_counter+=1

def goToPoint(chassisOutput, tibiaOutput, startFemur, startTibia):
	for i in range(8):
		if i == 0 or i%2 == 0:
			while(startFemur != int(chassisOutput[i/2])):
				pwm.set_pwm(i, 0, int(startFemur))
				if (startFemur < chassisOutput[i/2]):
					startFemur+=1
				elif (startFemur > chassisOutput[i/2]):
					startFemur-=1					
				time.sleep(0.005)
		else:
			while(startTibia != int(tibiaOutput[(i-1)/2])):
				pwm.set_pwm(i, 0, int(startTibia))
				if (startTibia < tibiaOutput[(i-1)/2]):
					startTibia+=1
				elif (startTibia > tibiaOutput[(i-1)/2]):
					startTibia-=1
				time.sleep(0.005)

					
		startFemur = 255
		startTibia = 275					

	

#makes spyndra stand up before running
def spyndraStand():
	startFemur = 255
	startTibia = 570
	#motor4Femur = 370;
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
		
def spyndraSit():	
	endFemur = 255
	endTibia = 275
	#motor4Femur = 370
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
	

#calls gaitGenerator file and receives arrays from pipeline
def obtainGait():
	process = Popen(['python',gaitGenerator], stdout=PIPE, stderr=PIPE)
	stdout, stderr = process.communicate()
	return stdout

motorType = input("Type 1 if Analog Servo, 2 if Digital Servo: ")
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
pullMotorVal(motorType)
gait= obtainGait()
gaitSave = gait
gait = gait.replace("]","")
gait = gait.replace("\n","")
gaitArray = gait.split('[')
chassis = np.fromstring(gaitArray[1],dtype=float,sep=" ")
tibia = np.fromstring(gaitArray[2],dtype=float,sep=" ")
phase = input('Enter phase between legs (in degrees): ')
spyndraStand()
splineRunner(chassis, tibia, phase, type)
spyndraSit()
save = 0
if(type == 1):
	save = input("Would you like to save that run? (1 if yes, 0 if no): ")
if(save == 1):
	target = open('log.txt', 'a')
	target.write(time.strftime("%c"))
	target.write("\n")
	target.write(gaitSave)
	target.write("\n")
	target.close()
