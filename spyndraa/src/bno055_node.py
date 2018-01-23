#!/usr/bin/env python
import rospy
import rosbag
from sensor_msgs.msg import Imu
#from BNO055 import *
import math
import time

def IMUstart():
	# Turn on the IMU
	bno = BNO055()
	if bno.begin() is not True:
		print "Error initializing IMU"
		exit()
	time.sleep(1)
	bno.setExternalCrystalUse(True)
	return bno

def main():
	# initialize a IMU node
	imu_data = Imu()
	rospy.init_node("bno055_node")
	rate = rospy.Rate(100) # 100Hz
	# sensor measurements publishers
	pub_imu = rospy.Publisher('imu/data', Imu, queue_size=1)

	# saving data 2 rosbag
	bag = rosbag.Bag('imu_data_'+ str(time.time()).split(".")[0] +'.bag', 'w')
		
	
	#bno = IMUstart()
	
	seq = 0
	while not rospy.is_shutdown():
		# data preparation
		#grav  = bno.getVector(BNO055.VECTOR_GRAVITY)
		#euler = bno.getVertor(BNO055.VECTOR_EULER)
		grav = [1,2,3]
		euler = [-1,-2,-3]
		# round the values to the thousandth place
		rPlace = 1000
		grav_rounded  = [math.ceil(x*rPlace)/rPlace for x in grav]
		euler_rounded = [math.ceil(x*rPlace)/rPlace for x in euler]
		euler_rounded[0] = (abs(euler_rounded[0]) - 360) if euler_rounded[0] > 180 else euler_rounded[0]

		# publishing imu data
		imu_data.header.stamp = rospy.Time.now()
		imu_data.header.frame_id = "bno05_node"
		imu_data.header.seq = seq
		imu_data.orientation_covariance[0] = -1
		imu_data.linear_acceleration.x = grav_rounded[0]
		imu_data.linear_acceleration.y = grav_rounded[1]
		imu_data.linear_acceleration.z = grav_rounded[2]
		imu_data.linear_acceleration_covariance[0] = -1
		imu_data.angular_velocity.x = euler_rounded[0]
		imu_data.angular_velocity.y = euler_rounded[1]
		imu_data.angular_velocity.z = euler_rounded[2]
		imu_data.angular_velocity_covariance[0] = -1
		pub_imu.publish(imu_data)
		bag.write('/imu/data', imu_data)
		seq += 1
		rate.sleep()
	bag.close()





if __name__ == '__main__':
	main()
