from motorModule import SpyndraMotor
import time

# send particular behavior to spyndra motor
class SpyndraBehavior:

    def __init__(self):
        self.motor = SpyndraMotor() 

    def set_motor_type(motor_type):
        self.motor.set_motor_type(motor_type)

    def stand():
        startFemur = 255
        startTibia = 570
        self.motor.output_motor(startFemur, startTibia, 0, 1)
        self.motor.output_motor(startFemur, startTibia, 2, 3)   
        self.motor.output_motor(startFemur, startTibia, 4, 5)
        self.motor.output_motor(startFemur, startTibia, 6, 7)
        while startTibia > 275:
            startTibia += -1
            self.motor.output_motor(startFemur, startTibia, 0, 1)
            self.motor.output_motor(startFemur, startTibia, 2, 3)
            self.motor.output_motor(startFemur, startTibia, 4, 5)
            self.motor.output_motor(startFemur, startTibia, 6, 7)
            time.sleep(0.001) 

    def sit():
        endFemur = 255
        endTibia = 275
        self.motor.output_motor(endFemur, endTibia, 0, 1)
        self.motor.output_motor(endFemur, endTibia, 2, 3)
        self.motor.output_motor(endFemur, endTibia, 4, 5)
        self.motor.output_motor(endFemur, endTibia, 6, 7)
        while endTibia < 550:
            endTibia += 1
            self.motor.output_motor(endFemur, endTibia, 0, 1)
            self.motor.output_motor(endFemur, endTibia, 2, 3)
            self.motor.output_motor(endFemur, endTibia, 4, 5)
            self.motor.output_motor(endFemur, endTibia, 6, 7)
            time.sleep(0.01)


    # #Outputs the splines according to a phase, note: type is refactored to the inner class
    # def spline_run(chassis, tibia, phase, bno, flat_matrix,dataIMU):
    #     if dataIMU==1:
    #         tick = datetime.now()

    #         #with open(IMU_file, 'a') as csvfile:
    #         #        IMU_writer = csv.writer(csvfile, delimiter = ' ')
    #         #        IMU_writer.writerow("The flat matrix is " + str(flatMatrix) + " \n")
            
    #         with open(IMU_file, 'a') as csvfile:
    #                 IMU_writer = csv.writer(csvfile, delimiter = ' ')
    #                 flatMatrixWrite = [flatMatrix[0][0], flatMatrix[0][1], flatMatrix[0][2], flatMatrix[1][0],  flatMatrix[1][1], flatMatrix[1][2]]
    #                 IMU_writer.writerow(flatMatrixWrite)

    #         # IMU_output.writerow("The flat matrix is " + str(flatMatrix) + " \n")

    #         #this determines the number of times the IMU reads data
    #         IMU_cycleThreshold = 5
    #         IMU_cycleCounter = 1
        
    #     leg1_counter = ((4.0*phase)/360.0)*len(chassis)
    #     leg2_counter = ((3.0*phase)/360.0)*len(chassis)
    #     leg3_counter = ((2.0*phase)/360.0)*len(chassis)
    #     leg4_counter = ((1.0*phase)/360.0)*len(chassis)

    #     #In the case of a phase greater than 180, leg1 and leg2 must be corrected back to 0 and 180 degrees
    #     if(phase >= 180):
    #         leg2_counter = ((1.0*phase)/360.0)*len(chassis)
    #         leg1_counter = ((2.0*phase)/360.0)*len(chassis)
            

    #     for i in range(3):
    #         for i in range(len(chassis)):
    #             if leg1_counter >= len(chassis):
    #                 leg1_counter -= len(chassis)
    #             if leg2_counter >= len(chassis):
    #                 leg2_counter -= len(chassis)
    #             if leg3_counter >= len(chassis):
    #                 leg3_counter -= len(chassis)
    #             if leg4_counter >= len(chassis):
    #                 leg4_counter -= len(chassis)

    #             #determines whether the IMU will read in the current cycle
    #             if dataIMU==1:
    #                 if IMU_cycleCounter == IMU_cycleThreshold:
    #                         IMU_cycleCounter = 1
    #                 else:
    #                         IMU_cycleCounter+= 1
                
    #             #run for percentages
    #             if self.motor.motor_type == 1 or self.motor.motor_type == 2:
    #                 chassisOutput1 = chassis[leg1_counter]*(motor0_max-motor0_min) + motor0_min
    #                 tibiaOutput1 = tibia[leg1_counter]*(motor1_max-motor1_min)+motor1_min
                    
    #                 chassisOutput2 = chassis[leg2_counter]*(motor2_max-motor2_min) + motor2_min
    #                 tibiaOutput2 = tibia[leg2_counter]*(motor3_max-motor3_min)+motor3_min
        
    #                 chassisOutput3 = chassis[leg3_counter]*(motor4_max-motor4_min) + motor4_min
    #                 tibiaOutput3 = tibia[leg3_counter]*(motor5_max-motor5_min)+motor5_min
        
    #                 chassisOutput4 = chassis[leg4_counter]*(motor6_max-motor6_min) + motor6_min
    #                 tibiaOutput4 = tibia[leg4_counter]*(motor7_max-motor7_min)+motor7_min   

    #                 self.motor.output_motor(chassisOutput1, tibiaOutput1, 0, 1)
    #                 self.motor.output_motor(chassisOutput2, tibiaOutput2, 2, 3)                 
    #                 self.motor.output_motor(chassisOutput3, tibiaOutput3, 4, 5)
    #                 self.motor.output_motor(chassisOutput4, tibiaOutput4, 6, 7)

    #                 #read data from IMU
    #                 if dataIMU==1:
    #                     if IMU_cycleCounter == IMU_cycleThreshold:
    #                         currentTimeD = tick - datetime.now()
    #                         currentReading, errorCounter = produceVector(bno)
    #                         currentEditedReading = currentReading - flatMatrix

    #                         dataWrite = [currentTimeD.total_seconds(), currentEditedReading[0][0], currentEditedReading[0][1], currentEditedReading[0][2], currentEditedReading[1][0], currentEditedReading[1][1], currentEditedReading[1][2], chassisOutput1, tibiaOutput1, chassisOutput2, tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4] 
                            
    #                         #IMU_output.writerow(str(currentTimeD.total_seconds()) + " ")
    #                         #IMU_output.write(str(currentReading[0])+ ' ' + str(currentReading[1]))
    #                         #IMU_output.writerow(str(currentEditedReading[0]) + ' ' + str(currentEditedReading[1]))

    #                         #the below line also outputs tibia and femur positions
    #                         #IMU_output.writerow(str(chassisOutput1) + " " + str(tibiaOutput1) + " ")
    #                         #IMU_output.writerow(str(chassisOutput2) + " " + str(tibiaOutput2) + " ")
    #                         #IMU_output.writerow(str(chassisOutput3) + " " + str(tibiaOutput3) + " ")
    #                         #IMU_output.writerow(str(chassisOutput4) + " " + str(tibiaOutput4) + " ")

    #                         with open(IMU_file, 'a') as csvfile:
    #                                 IMU_writer = csv.writer(csvfile, delimiter = ' ')
    #                                 IMU_writer.writerow(dataWrite)
    #                         #IMU_output.writerow('\n')

    #             #run for motor angles
    #             elif self.motor.motor_type == 3:
    #                 chassisOutput1 = chassis[leg1_counter]
    #                 tibiaOutput1 = tibia[leg1_counter]
                    
    #                 chassisOutput2 = chassis[leg2_counter]
    #                 tibiaOutput2 = tibia[leg2_counter]

    #                 chassisOutput3 = chassis[leg3_counter]
    #                 tibiaOutput3 = tibia[leg3_counter]

    #                 chassisOutput4 = chassis[leg4_counter]
    #                 tibiaOutput4 = tibia[leg4_counter]  
                    
    #                 self.motor.output_motor(chassisOutput1, tibiaOutput1, 0, 1)
    #                 self.motor.output_motor(chassisOutput2, tibiaOutput2, 2, 3) 
    #                 self.motor.output_motor(chassisOutput3, tibiaOutput3, 4, 5)
    #                 self.motor.output_motor(chassisOutput4, tibiaOutput4, 6, 7)

    #                                 #read data from IMU
    #                 if dataIMU==1:
    #                     if IMU_cycleCounter == IMU_cycleThreshold:
    #                         currentTimeD = tick - datetime.now()
    #                         currentReading, errorCounter = produceVector(bno)
    #                         currentEditedReading = currentReading - flatMatrix

    #                         dataWrite = [currentTimeD.total_seconds(), currentEditedReading[0][0], currentEditedReading[0][1], currentEditedReading[0][2], currentEditedReading[1][0], currentEditedReading[1][1], currentEditedReading[1][2], chassisOutput1, tibiaOutput1, chassisOutput2, tibiaOutput2, chassisOutput3, tibiaOutput3, chassisOutput4, tibiaOutput4] 
                            
    #                         #IMU_output.writerow(str(currentTimeD.total_seconds()) + " ")
    #                         #IMU_output.write(str(currentReading[0])+ ' ' + str(currentReading[1]))
    #                         #IMU_output.writerow(str(currentEditedReading[0]) + ' ' + str(currentEditedReading[1]))

    #                         #the below line also outputs tibia and femur positions
    #                         #IMU_output.writerow(str(chassisOutput1) + " " + str(tibiaOutput1) + " ")
    #                         #IMU_output.writerow(str(chassisOutput2) + " " + str(tibiaOutput2) + " ")
    #                         #IMU_output.writerow(str(chassisOutput3) + " " + str(tibiaOutput3) + " ")
    #                         #IMU_output.writerow(str(chassisOutput4) + " " + str(tibiaOutput4) + " ")

    #                         with open(IMU_file, 'a') as csvfile:
    #                                 IMU_writer = csv.writer(csvfile, delimiter = ' ')
    #                                 IMU_writer.writerow(dataWrite)
    #                         #IMU_output.writerow('\n')

    #             leg1_counter+=1
    #             leg2_counter+=1
    #             leg3_counter+=1
    #             leg4_counter+=1