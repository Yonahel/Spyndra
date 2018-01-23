import math
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d

#points array of values from 0-1 (to be mapped)
#All femurs 400 except 4 which is 370
#All tibia 450

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

standing_femur = [0.30, 0.50, 0.30, 0.10]
standing_tibia = [0.20, 0.40, 0.20, 0.00]
spline_gen(standing_femur, standing_tibia, 5, 2)