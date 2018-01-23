#import Adafruit_PCA9685
 
class SpyndraMotor:

	def __init__(self, freq=60, motor_type=1):
		# motro min, max constants
		self.motor0_min = 250
		self.motor0_max = 300
		self.motor1_min = 250
		self.motor1_max = 300
		self.motor2_min = 250
		self.motor2_max = 300
		self.motor3_min = 250
		self.motor3_max = 300
		self.motor4_min = 250
		self.motor4_max = 300
		self.motor5_min = 250
		self.motor5_max = 300
		self.motor6_min = 250
		self.motor6_max = 300
		self.motor7_min = 250
		self.motor7_max = 300
		self.tibia_min = 250
		self.tibia_max = 300
		self.chassis_min = 250
		self.chassis_max = 300

		# internal objects
		self.pwm = Adafruit_PCA9685.PCA9685()
		self.pwm.set_pwm_freq(freq)
		self.motor_type = motor_type

	def set_motor_type(motor_type):
		self.motor_type = motor_type

	def output_motor(chassisOutput, tibiaOutput, chassisNum, tibiaNum):
		self.pwm.set_pwm(chassisNum, 0, int(chassisOutput))
		self.pwm.set_pwm(tibiaNum, 0, int(tibiaOutput))

		# Notice: what's this for? if this is setting the constants for motors then it should be set as internal variable
	# servo_settings.json
	def set_motor_value(filepath):
		json_data = open(filepath).read()
		parsed_json = json.loads(json_data)
		if(motorType == 1):
			self.motor0_min = parsed_json['motor 0 min']
			self.motor0_max = parsed_json['motor 0 max']
			self.motor1_min = parsed_json['motor 1 min']
			self.motor1_max = parsed_json['motor 1 max']
			self.motor2_min = parsed_json['motor 2 min']
			self.motor2_min = parsed_json['motor 2 max']
			self.motor3_min = parsed_json['motor 3 min']
			self.motor3_max = parsed_json['motor 3 max']
			self.motor4_min = parsed_json['motor 4 min']
			self.motor4_max = parsed_json['motor 4 max']
			self.motor5_min = parsed_json['motor 5 min']
			self.motor5_max = parsed_json['motor 5 max']
			self.motor6_min = parsed_json['motor 6 min']
			self.motor6_max = parsed_json['motor 6 max']
			self.motor7_min = parsed_json['motor 7 min']
			self.motor7_max = parsed_json['motor 7 max']
		elif(motorType == 2):
			self.tibia_min = parsed_json['digital tibia min']
			self.tibia_max = parsed_json['digital tibia max']
			self.chassis_min = parsed_json['digital chassis min']
			self.chassis_max = parsed_json['digital chassis max']