import numpy as np
import gazebo_env
import roslaunch
import os
import rospy
import time
from std_srvs.srv import Empty
from spyndra.msg import MotorSignal
from sensor_msgs.msg import Imu
from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState, SetModelConfiguration
import sys


class SpyndraEnv(gazebo_env.GazeboEnv):
    def __init__(self, nS, nA):
        self.nS = nS
        self.nA = nA
        # Launch the simulation with the given launchfile name
        print "PYTHONLOG: Launching GAZEBO"
        gazebo_env.GazeboEnv.__init__(self, "~/catkin_ws/src/spyndra_gazebo/launch/spyndra_world.launch", gui=True)
        rospy.wait_for_service('/gazebo/unpause_physics')
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        launch = roslaunch.parent.ROSLaunchParent(uuid, [os.path.expanduser('~') + "/catkin_ws/src/spyndra_control/launch/spyndra_control.launch"])
        launch.start()
        time.sleep(.5)
        print "PYTHONLOG: Launch finished. Start node initiation"
        rospy.init_node('spyndra_env', anonymous=True)

        self.action_publisher = rospy.Publisher('motor_signal', MotorSignal, queue_size=5)
        self.unpause = rospy.ServiceProxy('/gazebo/unpause_physics', Empty)
        self.pause = rospy.ServiceProxy('/gazebo/pause_physics', Empty)
        self.reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
        self.set_model_state_proxy = rospy.ServiceProxy('/gazebo/set_model_state', SetModelState)
        self.set_model_config_proxy = rospy.ServiceProxy('/gazebo/set_model_configuration', SetModelConfiguration)
        print "node initiaiton finished"
        self.reward_range = (-np.inf, np.inf)

        self.START_TIME = time.time()
        self.INIT_POS = None
        self.GOAL = None
        self.INIT_DIST = None

    def _reset(self):       
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
            motor_signal = MotorSignal()
            motor_signal.motor_type = 1
            motor_signal.signal = [512, 512, 512, 512, 512, 512, 512, 512]
            self.action_publisher.publish(motor_signal)
        except:
            print ("cannot publish action")
            time.sleep(.5)
        
        # return to initial joint positions
        rospy.wait_for_service('/gazebo/set_model_configuration')
        try:
            joint_names = ['base_to_femur_1', 'base_to_femur_2', 'base_to_femur_3', 'base_to_femur_4', 'femur_to_tibia_1', 'femur_to_tibia_2', 'femur_to_tibia_3', 'femur_to_tibia_4']
            joint_positions = [0, 0, 0, 0, 0, 0, 0, 0]
            self.set_model_config_proxy('spyndra', 'spyndra_description', joint_names, joint_positions)
        except (rospy.ServiceException) as e:
            print ("/gazebo/set_model_configuration service call failed")
        time.sleep(.5)

        # return to initial model position
        rospy.wait_for_service('/gazebo/set_model_state')
        try:
            model_state = ModelState()
            model_state.model_name = 'spyndra'
            model_state.pose.position.x, model_state.pose.position.y, model_state.pose.position.z = 0, 0, 0.5
            model_state.pose.orientation.x, model_state.pose.orientation.y, model_state.pose.orientation.z, model_state.pose.orientation.w = 0, 0, 0, 0 
            model_state.twist.linear.x, model_state.twist.linear.y, model_state.twist.linear.z = 0, 0, 0
            model_state.twist.angular.x, model_state.twist.angular.y, model_state.twist.angular.z = 0, 0, 0
            self.set_model_state_proxy(model_state)
        except (rospy.ServiceException) as e:
            print ("/gazebo/set_model_state service call failed")
        time.sleep(1)

        # stand up
	try:
       	# Wait for robot to be ready to accept signal
		rospy.wait_for_message('motor_state', MotorSignal, timeout=5)
	        motor_signal = MotorSignal()
	        motor_signal.motor_type = 1
		# femeor 400~630
		# tibia  650~900
		motor_signal.signal = [512, 512, 512, 512, 820, 820, 820, 820]
	        self.action_publisher.publish(motor_signal)
	except:
	        print ("cannot publish action")
        time.sleep(7)
        # initiate observation
        s_ = np.zeros(self.nS)
        # imu data update
#        imu_data = None
#        while imu_data is None:
#            try:
#                imu_data = rospy.wait_for_message('imu', Imu)
#                s_[8: 14] = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, \
#                             imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z]
#            except:
#                pass
        
        # motor data update
        motor_data = None
        while motor_data is None:
            try:
                motor_data = rospy.wait_for_message('motor_state', MotorSignal)
                s_[:8] = motor_data.signal
            except:
                pass
        
        # position data update
        position_data = None
        while position_data is None:
            try:
                position_data = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
                index = position_data.name.index("spyndra")
                #s_[8: 11] = 
		x, y, z = [position_data.pose[index].position.x, position_data.pose[index].position.y, position_data.pose[index].position.y]
                self.INIT_POS = np.array([x, y, z])
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

        self.START_TIME = time.time()
        return s_

    def clip_signal(self, x, Min, Max):
	if x < Min or x > Max:
		self.torque += 1.
	return max(min(Max, x), Min)

    def _step(self, action, s):
        # take action and update the observation(state)
        rospy.wait_for_service('/gazebo/unpause_physics')
        try:
                self.unpause()
        except (rospy.ServiceException) as e:
                print ("/gazebo/unpause_physics service call failed")
    
        s_ = s.copy()
        
        # if the observation contains previous state
        if len(s) > 8:
            s_ = np.hstack((np.zeros(8), s_[ :-8]))
        
        # update motor position
	self.torque = 0.
        try:
            motor_signal = MotorSignal()
            motor_signal.motor_type = 1
            s_[:8] = s[:8] + np.array(action).round()
	    for i in range(4):
		s_[i] = self.clip_signal(s_[i], 350, 630)
	    for i in range(4, 8, 1):
		s_[i] = self.clip_signal(s_[i], 650, 890)
            motor_signal.signal = s_[:8]
            #print "published signal", motor_signal.signal
            self.action_publisher.publish(motor_signal)
        except:
            print ("cannot publish action")
        s_time = time.time()
        
        # imu data update
#        imu_data = None
#        while imu_data is None:
#            try:
#                imu_data = rospy.wait_for_message('imu', Imu, timeout=5)
#                s_[8: 14] = [imu_data.linear_acceleration.x, imu_data.linear_acceleration.y, imu_data.linear_acceleration.z, \
#                             imu_data.angular_velocity.x,    imu_data.angular_velocity.y,    imu_data.angular_velocity.z]
#            except:
#                pass
        
        motor_data = None
        #while (time.time() - s_time) < .1:
        #motor_data = None
        while motor_data is None:
            try:
                motor_data = rospy.wait_for_message('motor_state', MotorSignal, timeout=5)
                s_[:8] = np.array(motor_data.signal)
            	
	    except:
                pass
        
        # position data update
        position_data = None
	while position_data is None:
            try:
                position_data = rospy.wait_for_message('/gazebo/model_states', ModelStates, timeout=5)
                index = position_data.name.index("spyndra")
                position_data =  [position_data.pose[index].position.x, position_data.pose[index].position.y, position_data.pose[index].position.y]
            except:
                pass

        rospy.wait_for_service('/gazebo/pause_physics')
        try:
            self.pause()
        except (rospy.ServiceException) as e:
            print ("/gazebo/pause_physics service call failed")

        # Reward Function
        ## Time reward
        time_reward = 10 - .2 * (time.time() - self.START_TIME)
        #time_reward = 0.
        
        ## Distance reward
        curr_dist2goal = np.linalg.norm(np.array(position_data) - self.GOAL)
        dist_reward = (self.INIT_DIST - curr_dist2goal) * 15. / self.INIT_DIST
        #print "dist to goal:", curr_dist2goal, "dist reward:", dist_reward, "motor signal:", s_[:8]
        
        ## Angel reward
	angl_reward = 0
        #angl_reward = (2 * np.pi - angl2goal) * 15. / (2 * np.pi)

        ## Torque reward
	torq_reward = -self.torque * 5
	print(self.torque)
	reward = time_reward + dist_reward * 5 + torq_reward + angl_reward
    
        # threshold TBD
        if curr_dist2goal < 1:
            done = True
        else:
            done = False
    
        return s_, reward, done, curr_dist2goal
    
