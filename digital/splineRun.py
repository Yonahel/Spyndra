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
#gaitGenerator = './splineGen.py'
#gaitGenerator = './standingGait.py'


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
		
	flag = 1
	startFemur = 400
	startTibia = 450 

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
			elif type == 2:
				chassisOutput1 = chassis[leg1_counter]
				tibiaOutput1 = tibia[leg1_counter]
				
				chassisOutput2 = chassis[leg2_counter]
				tibiaOutput2 = tibia[leg2_counter]

				chassisOutput3 = chassis[leg3_counter]
				tibiaOutput3 = tibia[leg3_counter]

				chassisOutput4 = chassis[leg4_counter]
				tibiaOutput4 = tibia[leg4_counter]	
				
				#on first run, smooth run to first movement
				if flag == 1:
					chassisOutput = [chassisOutput1, chassisOutput2, chassisOutput3, chassisOutput4]
					tibiaOutput = [tibiaOutput1, tibiaOutput2, tibiaOutput3, tibiaOutput4]	
					goToPoint(chassisOutput, tibiaOutput, startFemur, startTibia)	
					flag = 2				


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

					
		startFemur = 400
		startTibia = 450					

	

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


type = input("Type 1 for Random Gait, 2 for Standing Gait: ")
if(type == 1):
	gaitGenerator = './splineGen.py'
elif(type == 2):
	gaitGenerator = './standingGait.py'
pullMotorVal()
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
save = input("Would you like to save that run? (1 if yes, 0 if no): ")
if(save == 1):
	target = open('log.txt', 'a')
	target.write(time.strftime("%c"))
	target.write("\n")
	target.write(gaitSave)
	target.write("\n")
	target.close()
