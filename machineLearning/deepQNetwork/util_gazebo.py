import numpy as np
import gazebo_env
import roslaunch
import os
import rospy
import time
from std_srvs.srv import Empty
from spyndra.msg import MotorSignal
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates
import sys


class SpyndraEnv(gazebo_env.GazeboEnv):
	def __init__(self):
		# Launch the simulation with the given launchfile name
		print "PYTHONLOG: Launching GAZEBO"
		gazebo_env.GazeboEnv.__init__(self, "~/catkin_ws/src/spyndra_gazebo/launch/spyndra_world.launch")
		rospy.wait_for_service('/gazebo/unpause_physics')
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.expanduser('~') + "/catkin_ws/src/spyndra_control/launch/spyndra_control.launch"])
		launch.start()
		print "PYTHONLOG: Launch finished. Start node initiation"
		rospy.init_node('spyndra_env', anonymous=True)

		self.action_publisher = rospy.Publisher('motor_signal', MotorSignal, queue_size=5)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
		print "node initiaiton finished"
		self.reward_range = (-np.inf, np.inf)
		self.prev_signal = []

		self.START_TIME = time.time()
		self.INIT_POS = None
		self.GOAL = None
		self.INIT_DIST = None
		#self._seed()

	#def _seed(self, seed=None):
	#	self.np_random, seed = seeding.np_random(seed)
	#	return [seed]

	def _reset(self):
		# return the initial state
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

		try:
			# Wait for robot to be ready to accept signal
			rospy.wait_for_message('motor_state', MotorSignal, timeout=5)
			time.sleep(.5)
			motor_signal = MotorSignal()
			motor_signal.motor_type = 1
			motor_signal.signal = [512, 512, 512, 512, 820, 820, 820, 820]
			self.action_publisher.publish(motor_signal)
			self.prev_signal = motor_signal.signal[:]
		except:
			print ("cannot publish action")
		time.sleep(3)
		
		s_ = np.zeros(90)
		# imu data update
		imu_data = None
		while imu_data is None:
			try:
				imu_data = rospy.wait_for_message('imu', Imu)
				s_[9: 15] = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, \
							 imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z]
			except:
				pass
		# TODO: Parse imu_data
		
		# motor data update
		motor_data = None
		while motor_data is None:
			try:
				motor_data = rospy.wait_for_message('motor_state', MotorSignal)
				# s_[:8] = motor_data.signal
				s_[:8] = self.prev_signal[:]
			except:
				pass
		
		# position data update
		position_data = None
		while position_data is None:
		#	print "Waiting for motor message..."
			try:
				position_data = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
				index = position_data.name.index("spyndra")
				s_[15: 18] = [position_data.pose[index].position.x, position_data.pose[index].position.y, position_data.pose[index].position.y]
				self.INIT_POS = np.array(s_[15: 18])
				self.GOAL = np.array(self.INIT_POS, copy=True)
				self.GOAL[0] += 10
				self.INIT_DIST = np.linalg.norm(self.INIT_POS - self.GOAL)
			except:
				pass
		
		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			#resp_pause = pause.call()
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		return s_


	def _step(self, action, s):
		# take action and update the observation(state)
		
		rospy.wait_for_service('/gazebo/unpause_physics')
		try:
				self.unpause()
		except (rospy.ServiceException) as e:
				print ("/gazebo/unpause_physics service call failed")
	
		
		s_ = s.copy()
		
		# update previous state
		s_ = np.hstack((np.zeros(18), s_[ :-18]))
                # store action into state
                s_[8] = action	

                # parse action to motor signal
		motor_index = action / 3 
		# action = 0(decrease by MOTORSTEP), 1(maintain), 1(increase by MOTORSTEP)
		action = (action - motor_index * 3) - 1
		
		# publish motor signal into Spyndra / gazebo
		MOTORSTEP = 5
		try:
			motor_signal = MotorSignal()
			motor_signal.motor_type = 1
			# motor_signal.signal = s_[18:26]
			motor_signal.signal = self.prev_signal[:]
			motor_signal.signal[motor_index] += action * MOTORSTEP
			self.action_publisher.publish(motor_signal)
			self.prev_signal = motor_signal.signal[:]
		except:
			print ("cannot publish action")
		#time.sleep(1)
		# imu data update
		imu_data = None
		while imu_data is None:
		#	print "Waiting for IMU message..."
			try:
				imu_data = rospy.wait_for_message('imu', Imu, timeout=5)
				s_[9: 15] = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, \
							 imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z]
			except:
				pass
		
                # motor data update
		motor_data = None
		while motor_data is None:
		#	print "Waiting for motor message..."
			try:
				motor_data = rospy.wait_for_message('motor_state', MotorSignal, timeout=5)
				#s_[8:8] = motor_data.signal
				s_[:8] = self.prev_signal[:]
			except:
				pass
		
		# position data update
		position_data = None
		while position_data is None:
		#	print "Waiting for motor message..."
			try:
				position_data = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
				index = position_data.name.index("spyndra")
				s_[15: 18] = [position_data.pose[index].position.x, position_data.pose[index].position.y, position_data.pose[index].position.y]
			except:
				pass

		rospy.wait_for_service('/gazebo/pause_physics')
		try:
			self.pause()
		except (rospy.ServiceException) as e:
			print ("/gazebo/pause_physics service call failed")

		# Time reward
		time_reward = 60 - .1 * (time.time() - self.START_TIME)
		
		# Distance reward
		dist2goal = np.linalg.norm(np.array(s_[15:18]) - self.GOAL)
		dist_reward = (self.INIT_DIST - dist2goal) * 15. / self.INIT_DIST
                print "dist to goal:", dist2goal, "dist reward:", dist_reward, "motor signal:", s_[:8]
		# Angel reward
		#angl_reward = (2 * np.pi - angl2goal) * 15. / (2 * np.pi)

		# euclidean norm of accleration
		reward = time_reward + dist_reward * 2 #+ angl_reward
	
		# threshold TBD
		if dist_reward == 15:
			done = True
		else:
			done = False
	
		return s_, reward, done
	
