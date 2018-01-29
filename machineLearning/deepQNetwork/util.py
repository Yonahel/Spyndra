import gym
import numpy as np
import gazebo_env

from std_srvs.srv import Empty
from spyndra.msg import MotorSignal
from sensor_msgs.msg import Imu

class SpyndraEnv(gazebo_env.GazeboEnv):
	def __init__(self):
		# Launch the simulation with the given launchfile name
		gazebo_env.GazeboEnv.__init__(self, "GazeboCircuit2TurtlebotLidar_v0.launch")

		self.action_publisher = rospy.Publisher('motor_signal', MotorSignal, queue_size=5)
		
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

		self.action_space = spaces.Discrete(3) #Maintain, Increase, Decrease
		self.reward_range = (-np.inf, np.inf)

		self._seed()

	def _seed(self, seed=None):
		self.np_random, seed = seeding.np_random(seed)
		return [seed]

	def _reset(self):
		# return the initial state
		# here should assign some initial values, eg. standing gait data
		#position = np.zeros(8)
		#speed    = np.zeros(8)
		#load     = np.zeros(8)
		#prev_actions = np.zeros(5)
		#imu = np.zeros(6)
		#positions = np.random.randint(180, size=8)
		#speed = np.random.randint(10, size=8)
		#load = np.random.randint(10, size=8)
		#imu = np.random.randint(10, size=6)
		
		rospy.wait_for_service('/gazebo/reset_simulation')
        try:
            #reset_proxy.call()
            self.reset_proxy()
        except (rospy.ServiceException) as e:
            print ("/gazebo/reset_simulation service call failed")

        # Unpause simulation to make observation
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
            #resp_pause = pause.call()
            self.unpause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/unpause_physics service call failed")

        #read laser data
        imu_data = None
        while imu_data is None:
            try:
                imu_data = rospy.wait_for_message('imu/data', Imu, timeout=5)
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            #resp_pause = pause.call()
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        state = self.discretize_observation(data,5)

        return state

		return np.hstack((position, speed, load, prev_actions, imu))


	def _step(self, action, s):
		# take action and update the observation(state)
	
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
				self.unpause()
		except (rospy.ServiceException) as e:
				print ("/gazebo/unpause_physics service call failed")
	
	
		s_ = s.copy()
		motor_index = action & 3 
		action = (action - motor_index * 3)
	
		# publish motor signal into Spyndra
		motor_signal = MotorSignal()
		motor_signal.idx = motor_index
		motor_signal.action = action
		self.action_publisher(motor_signal)
	
		# update the observation based on new action
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
		
		imu_data = None
		while imu_data is None:
			try:
				imu_data = rospy.wait_for_message('imu/data', Imu, timeout=5)
			except:
				pass
		s_[29:35] = imu_data

		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")
	
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