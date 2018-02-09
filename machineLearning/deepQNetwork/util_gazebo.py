import numpy as np
import gazebo_env
import roslaunch
import os
import rospy
from std_srvs.srv import Empty
from spyndra.msg import MotorSignal
from sensor_msgs.msg import Imu
import sys


class SpyndraEnv(gazebo_env.GazeboEnv):
	def __init__(self):
		# Launch the simulation with the given launchfile name
		gazebo_env.GazeboEnv.__init__(self, "~/catkin_ws/src/spyndra_gazebo/launch/spyndra_world.launch")
		rospy.wait_for_service('/gazebo/unpause_physics')
		uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
		roslaunch.configure_logging(uuid)
		launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.expanduser('~') + "/catkin_ws/src/spyndra_control/launch/spyndra_control.launch"])
		launch.start()
		
		rospy.init_node('spyndra_env', anonymous=True)

		self.action_publisher = rospy.Publisher('motor_signal', MotorSignal, queue_size=5)
		self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
		self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
		self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)

		self.reward_range = (-np.inf, np.inf)
		
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
		
		s_ = np.zeros(35)
		# imu data update
		imu_data = None
		while imu_data is None:
			try:
				imu_data = rospy.wait_for_message('imu', Imu)
				s_[29:35] = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, \
							 imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z]
			except:
				pass
		# TODO: Parse imu_data
		
		# motor data update
		motor_data = None
		while motor_data is None:
			try:
				motor_data = rospy.wait_for_message('motor_state', MotorSignal)
				s_[:8] = motor_data
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
		motor_index = action & 3 
		# action = 0(decrease by MOTORSTEP), 1(maintain), 1(increase by MOTORSTEP)
		action = (action - motor_index * 3) - 1
		
		# publish motor signal into Spyndra / gazebo
		MOTORSTEP = 5
		try:
			motor_signal = MotorSignal()
			motor_signal.motor_type = 1
			motor_signal.signal = s_[:8]
			motor_signal.signal[motor_index] += action * MOTORSTEP
			self.action_publisher.publish(motor_signal)
		except:
			print ("cannot publish action")

		# update the observation based on new action
		# last 5 actions update (shift right)
		s_[24: 29] = np.hstack((action, s_[24:28]))
		
		# imu data update
		imu_data = None
		while imu_data is None:
			try:
				imu_data = rospy.wait_for_message('imu', Imu, timeout=5)
				s_[29:35] = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, \
							 imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z]
			except:
				pass
		# TODO: Parse imu_data
		
		# motor data update
		motor_data = None
		while motor_data is None:
			try:
				motor_data = rospy.wait_for_message('motor_signal', MotorSignal, timeout=5)
				s_[:8] = motor_data
			except:
				pass
		
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
	