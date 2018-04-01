'''
Based on Jesse Merritt's script:
https://github.com/jes1510/python_dynamixels

and Josue Alejandro Savage's Arduino library:
http://savageelectronics.blogspot.it/2011/01/arduino-y-dynamixel-ax-12.html

and Thiago Hersan's library:
https://github.com/thiagohersan/memememe/tree/master/Python/ax12
'''

from time import sleep
import time
import serial
import serial.tools.list_ports
# import RPi.GPIO as GPIO


# serial port helper functions
def get_ports():
    return serial.tools.list_ports.comports()

def print_ports():
    i = 1
    for port in serial.tools.list_ports.comports():
        print 'port %d name: %s' %(i, port.name)
    print 'found %d ports in total' %(i)

class Ax12:
    # important AX-12 constants
    # /////////////////////////////////////////////////////////// EEPROM AREA
    AX_MODEL_NUMBER_L = 0
    AX_MODEL_NUMBER_H = 1
    AX_VERSION = 2
    AX_ID = 3
    AX_BAUD_RATE = 4
    AX_RETURN_DELAY_TIME = 5
    AX_CW_ANGLE_LIMIT_L = 6
    AX_CW_ANGLE_LIMIT_H = 7
    AX_CCW_ANGLE_LIMIT_L = 8
    AX_CCW_ANGLE_LIMIT_H = 9
    AX_SYSTEM_DATA2 = 10
    AX_LIMIT_TEMPERATURE = 11
    AX_DOWN_LIMIT_VOLTAGE = 12
    AX_UP_LIMIT_VOLTAGE = 13
    AX_MAX_TORQUE_L = 14
    AX_MAX_TORQUE_H = 15
    AX_RETURN_LEVEL = 16
    AX_ALARM_LED = 17
    AX_ALARM_SHUTDOWN = 18
    AX_OPERATING_MODE = 19
    AX_DOWN_CALIBRATION_L = 20
    AX_DOWN_CALIBRATION_H = 21
    AX_UP_CALIBRATION_L = 22
    AX_UP_CALIBRATION_H = 23

    # ////////////////////////////////////////////////////////////// RAM AREA
    AX_TORQUE_STATUS = 24
    AX_LED_STATUS = 25
    AX_CW_COMPLIANCE_MARGIN = 26
    AX_CCW_COMPLIANCE_MARGIN = 27
    AX_CW_COMPLIANCE_SLOPE = 28
    AX_CCW_COMPLIANCE_SLOPE = 29
    AX_GOAL_POSITION_L = 30
    AX_GOAL_POSITION_H = 31
    AX_GOAL_SPEED_L = 32
    AX_GOAL_SPEED_H = 33
    AX_TORQUE_LIMIT_L = 34
    AX_TORQUE_LIMIT_H = 35
    AX_PRESENT_POSITION_L = 36
    AX_PRESENT_POSITION_H = 37
    AX_PRESENT_SPEED_L = 38
    AX_PRESENT_SPEED_H = 39
    AX_PRESENT_LOAD_L = 40
    AX_PRESENT_LOAD_H = 41
    AX_PRESENT_VOLTAGE = 42
    AX_PRESENT_TEMPERATURE = 43
    AX_REGISTERED_INSTRUCTION = 44
    AX_PAUSE_TIME = 45
    AX_MOVING = 46
    AX_LOCK = 47
    AX_PUNCH_L = 48
    AX_PUNCH_H = 49

    # /////////////////////////////////////////////////////////////// Status Return Levels
    AX_RETURN_NONE = 0
    AX_RETURN_READ = 1
    AX_RETURN_ALL = 2

    # /////////////////////////////////////////////////////////////// Instruction Set
    AX_PING = 1
    AX_READ_DATA = 2
    AX_WRITE_DATA = 3
    AX_REG_WRITE = 4
    AX_ACTION = 5
    AX_RESET = 6
    AX_SYNC_WRITE = 131

    # /////////////////////////////////////////////////////////////// Lengths
    AX_RESET_LENGTH = 2
    AX_ACTION_LENGTH = 2
    AX_ID_LENGTH = 4
    AX_LR_LENGTH = 4
    AX_SRL_LENGTH = 4
    AX_RDT_LENGTH = 4
    AX_LEDALARM_LENGTH = 4
    AX_SHUTDOWNALARM_LENGTH = 4
    AX_TL_LENGTH = 4
    AX_VL_LENGTH = 6
    AX_AL_LENGTH = 7
    AX_CM_LENGTH = 6
    AX_CS_LENGTH = 5
    AX_COMPLIANCE_LENGTH = 7
    AX_CCW_CW_LENGTH = 8
    AX_BD_LENGTH = 4
    AX_TEM_LENGTH = 4
    AX_MOVING_LENGTH = 4
    AX_RWS_LENGTH = 4
    AX_VOLT_LENGTH = 4
    AX_LOAD_LENGTH = 4
    AX_LED_LENGTH = 4
    AX_TORQUE_LENGTH = 4
    AX_POS_LENGTH = 4
    AX_GOAL_LENGTH = 5
    AX_MT_LENGTH = 5
    AX_PUNCH_LENGTH = 5
    AX_SPEED_LENGTH = 5
    AX_GOAL_SP_LENGTH = 7

    # /////////////////////////////////////////////////////////////// Specials
    AX_BYTE_READ = 1
    AX_INT_READ = 2
    AX_ACTION_CHECKSUM = 250
    AX_BROADCAST_ID = 254
    AX_START = 255
    AX_CCW_AL_L = 255
    AX_CCW_AL_H = 3
    AX_LOCK_VALUE = 1
    LEFT = 0
    RIGTH = 1
    RX_TIME_OUT = 10
    TX_DELAY_TIME = .05

    # RPi constants
    RPI_DIRECTION_PIN = 18
    # RPI_DIRECTION_TX = GPIO.HIGH
    # RPI_DIRECTION_RX = GPIO.LOW
    RPI_DIRECTION_SWITCH_DELAY = 0.0001

    # # static variables
    # port = None
    # gpioSet = False

    def __init__(self, port = None, baudrate=1000000, totalServos=8):
        if(port == None):
            # self.port = serial.Serial("/dev/ttyAMA0", baudrate=1000000, timeout=0.001)
            self.port = serial.Serial("/dev/ttyUSB0", baudrate, timeout=0.001)
        else:
            self.port = serial.Serial(port, baudrate, timeout=0.001)

        self.connectedServos = self.scanServos(maxValue=totalServos)

    # Error lookup dictionary for bit masking
    dictErrors = {  1 : "Input Voltage",
            2 : "Angle Limit",
            4 : "Overheating",
            8 : "Range",
            16 : "Checksum",
            32 : "Overload",
            64 : "Instruction"
            }

    # Custom error class to report AX servo errors
    class axError(Exception) : pass

    # Servo timeout
    class timeoutError(Exception) : pass

    # read data from the register
    def readData(self,id):
        # self.direction(Ax12.RPI_DIRECTION_RX)
        reply = self.port.read(5) # [0xff, 0xff, origin, length, error]
        try:
            assert ord(reply[0]) == 0xFF
            assert ord(reply[1]) == 0xFF
        except:
            e = "Timeout on servo " + str(id)
            raise Ax12.timeoutError(e)

        try :
            length = ord(reply[3]) - 2
            error = ord(reply[4])
            # print "length:", length, "error:", error
            if (error != 0):
                print "Error from servo: " + Ax12.dictErrors[error] + ' (code  ' + hex(error) + ')'
                #return -error
                returnValue = -error
            # just reading error bit
            elif(length == 0):
                return error
            # return data
            if(length > 1):
                reply = self.port.read(2)
                returnValue = (ord(reply[1])<<8) + (ord(reply[0])<<0)
            else:
                reply = self.port.read(1)
                returnValue = ord(reply[0])
            return returnValue
        except Exception, detail:
            raise Ax12.axError(detail)

    # return 0 when found
    def ping(self,id):
        self.port.flushInput()
        checksum = (~(id + Ax12.AX_READ_DATA + Ax12.AX_PING))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_PING)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    # reset ID and other preset values
    def factoryReset(self,id, confirm = False):
        if(confirm):
            self.port.flushInput()
            checksum = (~(id + Ax12.AX_RESET_LENGTH + Ax12.AX_RESET))&0xff
            outData = chr(Ax12.AX_START)
            outData += chr(Ax12.AX_START)
            outData += chr(id)
            outData += chr(Ax12.AX_RESET_LENGTH)
            outData += chr(Ax12.AX_RESET)
            outData += chr(checksum)
            self.port.write(outData)
            sleep(Ax12.TX_DELAY_TIME)
            return self.readData(id)
        else:
            print "nothing done, please send confirm = True as this fuction reset to the factory default value, i.e reset the motor ID"
            return

    def setID(self, id, newId):
        self.port.flushInput()
        checksum = (~(id + Ax12.AX_ID_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_ID + newId))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_ID_LENGTH)
        outData += chr(Ax12.AX_WRITE_DATA)
        outData += chr(Ax12.AX_ID)
        outData += chr(newId)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    # move servo to position (0~300 degree: 0~1023)
    def move(self, id, position, instant=False):
        if not instant:
            while self.readMovingStatus(id) == 1:
                pass
        self.port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + Ax12.AX_GOAL_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_GOAL_LENGTH)
        outData += chr(Ax12.AX_WRITE_DATA)
        outData += chr(Ax12.AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    # move at speed (0~114 rpm: 0~1023) to position
    # instant means don't wait previous movement finish
    def moveSpeed(self, id, position, speed, instant=False):
        if not instant:
            while self.readMovingStatus(id) == 1:
                pass
        self.port.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + Ax12.AX_GOAL_SP_LENGTH + Ax12.AX_WRITE_DATA + Ax12.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_GOAL_SP_LENGTH)
        outData += chr(Ax12.AX_WRITE_DATA)
        outData += chr(Ax12.AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(s[0])
        outData += chr(s[1])
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    # reg-write mode, similar to write-data, but remain in the standby state until ACTION command
    def moveRW(self, id, position, instant=False):
        if not instant:
            while self.readMovingStatus(id) == 1:
                pass
        self.port.flushInput()
        p = [position&0xff, position>>8]
        checksum = (~(id + Ax12.AX_GOAL_LENGTH + Ax12.AX_REG_WRITE + Ax12.AX_GOAL_POSITION_L + p[0] + p[1]))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_GOAL_LENGTH)
        outData += chr(Ax12.AX_REG_WRITE)
        outData += chr(Ax12.AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def moveSpeedRW(self, id, position, speed, instant=False):
        if not instant:
            while self.readMovingStatus(id) == 1:
                pass
        self.port.flushInput()
        p = [position&0xff, position>>8]
        s = [speed&0xff, speed>>8]
        checksum = (~(id + Ax12.AX_GOAL_SP_LENGTH + Ax12.AX_REG_WRITE + Ax12.AX_GOAL_POSITION_L + p[0] + p[1] + s[0] + s[1]))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_GOAL_SP_LENGTH)
        outData += chr(Ax12.AX_REG_WRITE)
        outData += chr(Ax12.AX_GOAL_POSITION_L)
        outData += chr(p[0])
        outData += chr(p[1])
        outData += chr(s[0])
        outData += chr(s[1])
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def action(self):
        self.port.flushInput()
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(Ax12.AX_BROADCAST_ID)
        outData += chr(Ax12.AX_ACTION_LENGTH)
        outData += chr(Ax12.AX_ACTION)
        outData += chr(Ax12.AX_ACTION_CHECKSUM)
        self.port.write(outData)
        #sleep(Ax12.TX_DELAY_TIME)

    def readTemperature(self, id):
        self.port.flushInput()
        self.port.flushOutput()
        checksum = (~(id + Ax12.AX_TEM_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_TEMPERATURE + Ax12.AX_BYTE_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_TEM_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_PRESENT_TEMPERATURE)
        outData += chr(Ax12.AX_BYTE_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readPosition(self, id):
        self.port.flushInput()
        self.port.flushOutput()
        checksum = (~(id + Ax12.AX_POS_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_POSITION_L + Ax12.AX_INT_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_POS_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_PRESENT_POSITION_L)
        outData += chr(Ax12.AX_INT_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readVoltage(self, id):
        self.port.flushInput()
        self.port.flushOutput()
        checksum = (~(id + Ax12.AX_VOLT_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_VOLTAGE + Ax12.AX_BYTE_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_VOLT_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_PRESENT_VOLTAGE)
        outData += chr(Ax12.AX_BYTE_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    # return current moving speed
    # error SPEED_LENGTH value! present uses LOAD length
    def readPresentSpeed(self, id):
        self.port.flushInput()
        checksum = (~(id + Ax12.AX_LOAD_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_SPEED_L + Ax12.AX_INT_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_LOAD_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_PRESENT_SPEED_L)
        outData += chr(Ax12.AX_INT_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readGoalSpeed(self, id):
        self.port.flushInput()
        checksum = (~(id + Ax12.AX_LOAD_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_GOAL_SPEED_L + Ax12.AX_INT_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_LOAD_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_GOAL_SPEED_L)
        outData += chr(Ax12.AX_INT_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def readLoad(self, id):
        self.port.flushInput()
        self.port.flushOutput()
        checksum = (~(id + Ax12.AX_LOAD_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_PRESENT_LOAD_L + Ax12.AX_INT_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_LOAD_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_PRESENT_LOAD_L)
        outData += chr(Ax12.AX_INT_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    # 1 is moving, 0 is still
    def readMovingStatus(self, id):
        self.port.flushInput()
        checksum = (~(id + Ax12.AX_MOVING_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_MOVING + Ax12.AX_BYTE_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_MOVING_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_MOVING)
        outData += chr(Ax12.AX_BYTE_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    # if instruction is registered (1) or not (0)
    def readRWStatus(self, id):
        self.port.flushInput()
        checksum = (~(id + Ax12.AX_RWS_LENGTH + Ax12.AX_READ_DATA + Ax12.AX_REGISTERED_INSTRUCTION + Ax12.AX_BYTE_READ))&0xff
        outData = chr(Ax12.AX_START)
        outData += chr(Ax12.AX_START)
        outData += chr(id)
        outData += chr(Ax12.AX_RWS_LENGTH)
        outData += chr(Ax12.AX_READ_DATA)
        outData += chr(Ax12.AX_REGISTERED_INSTRUCTION)
        outData += chr(Ax12.AX_BYTE_READ)
        outData += chr(checksum)
        self.port.write(outData)
        sleep(Ax12.TX_DELAY_TIME)
        return self.readData(id)

    def scanServos(self,minValue=1, maxValue=8, verbose=False) :
        servoList = []
        for i in range(minValue, maxValue + 1):
            try :
                temp = self.ping(i)
                if temp != None:
                    servoList.append(i)
                    if verbose: 
                        print "Found servo #" + str(i) +" with code " + str(temp)
                time.sleep(0.1)

            except Exception, detail:
                if verbose : 
                    print "Error pinging servo #" + str(i) + ': ' + str(detail)
                pass
        self.connectedServos = servoList
        return servoList