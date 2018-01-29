import numpy as np

def reset():
	# return the initial state
	# here should assign some initial values, eg. standing gait data
	position = np.zeros(8)
	speed    = np.zeros(8)
	load     = np.zeros(8)
	prev_actions = np.zeros(5)
	imu = np.zeros(6)
	positions = np.random.randint(180, size=8)
	speed = np.random.randint(10, size=8)
	load = np.random.randint(10, size=8)
	imu = np.random.randint(10, size=6)
	
	return np.hstack((position, speed, load, prev_actions, imu))


def takeAction(action, s):
	# take action and update the observation(state)
	s_ = s.copy()
	motor_index = action & 3 
	action = (action - motor_index * 3)
	# remain the angle of the motor
	if action == 0:  
		s_[motor_index] = s_[motor_index]
		s_[24: 29] = np.hstack((action, s_[24:28]))
	# increase the motor by 5 degree
	elif action == 1: 
		s_[motor_index] = s_[motor_index] + 5
		s_[24: 29] = np.hstack((action, s_[24:28]))
	# decrease the motor by 5 degree
	elif action == 2:
		s_[motor_index] = s_[motor_index] - 5
		s_[24: 29] = np.hstack((action, s_[24:28]))

	# here we simply assume Spyndra want's to move in x direction
	x_acc, y_acc, z_acc = s_[29:32]

	# euclidean norm of accleration
	reward = x_acc / np.linalg.norm([x_acc, y_acc, z_acc])
	
	# threshold TBD
	THRESHOLD = 1
	if x_acc > THRESHOLD:
		done = True
	else:
		done = False

	return s_, reward, done