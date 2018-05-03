#!/usr/bin/env python
import rospy
from std_msgs.msg import String
import message_filters
from std_msgs.msg import String
from sensor_msgs.msg import Imu
from spyndra import vectorModule
from spyndra.msg import MotorSignal
from spyndra.msg import BeaconPos
from Tkinter import *
import tkMessageBox
import math
class Listener:
    def __init__(self):
        self.mode = 'none'
        root = Tk()
        self.gui = GUI(root)
        
        rospy.Subscriber("/imu/data", Imu, self.imu_callback)
        rospy.Subscriber("/user_cmd", String, self.user_callback)
        rospy.Subscriber("/gps/data", BeaconPos, self.gps_callback)
        root.mainloop()

    # called when imu sensor returns data
    def imu_callback(self, imu):
        # when bno055.py node is spinning, imu data is packed in imu variable
        # s = 'received imu data, current mode: ' + str(self.mode)
        # rospy.loginfo(s)
        self.gui.setText(self.gui.imu, str(imu))
        # control node may neet to subscribe IMU data and feed them into RL models for action genetaion
        
        # gravity = msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z 
        # euler   = msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z        

        # take action based on subscribed msg "/user_cmd"
        chassis, tibia = [], []
        if self.mode == 'cmd_0':
            # calibration section

            pass
        else: 
            pass

    # called when user types a command in the user node
    def user_callback(self, user_input):
        self.mode = user_input.data
        s = 'mode set to be:'+ str(user_input)
        rospy.loginfo(s)
        self.gui.setText(self.gui.user_mode, s)

    # called when user types a command in the user node
    def gps_callback(self, gps_pos):
        log_msg = 'GPS position: \n' + str(gps_pos) + '\n'
        distance_3d = math.sqrt(gps_pos.x_m**2 + gps_pos.y_m**2 + gps_pos.z_m**2)
        distance_2d = math.sqrt(gps_pos.x_m**2 + gps_pos.y_m**2)
        log_msg += 'distance in 2d: ' + str(distance_2d)+ '\n'
        log_msg += 'distance in 3d: ' + str(distance_3d)+ '\n'
        rospy.loginfo(log_msg)
        self.gui.setText(self.gui.gps, log_msg)

class GUI:
    def __init__(self, master):
        self.master = master
        master.title("Spyndra Control Panel")

        frame_connection = Frame(master)
        frame_connection.pack()
        self.status = Label(frame_connection, text='Not connected, Enter socket address, port#:')
        self.status.pack(side=LEFT)
        self.socket = Entry(frame_connection)
        self.socket.pack(side=LEFT)
        self.connect_button = Button(frame_connection, text='Connect', command = self.connect)
        self.connect_button.pack(side=LEFT)

        frame_mode = Frame(master)
        frame_mode.pack()
        self.mode_notice = Label(frame_mode, text="Choose mode")
        self.mode_notice.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Calibration', command = self.calibration)
        self.standing_mode.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Standing', command = self.stand)
        self.standing_mode.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Training', command = self.train)
        self.standing_mode.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Movement', command = self.move)
        self.standing_mode.pack(side=LEFT)
        
        placeholder = 'signal placeholder'

        frame_motor = Frame(master)
        frame_motor.pack()
        self.motor = Text(frame_motor, height=10, width=80)
        self.motor.insert(INSERT, "motor "+ placeholder)
        self.motor.pack()        

        frame_gps= Frame(master)
        frame_gps.pack(side=BOTTOM)
        self.gps = Text(frame_gps, height=10, width=80)
        self.gps.insert(INSERT, "gps "+ placeholder)
        self.gps.pack()

        frame_imu = Frame(master)
        frame_imu.pack()
        self.imu = Text(frame_imu, height=25, width=80)
        self.imu.insert(INSERT, "imu "+ placeholder)
        self.imu.pack()

    def setText(self, text_obj, text):
        text_obj.delete('1.0', END)
        text_obj.insert(INSERT, text)

    def setLabel(self, label_obj, text):
        label_obj['text'] = text

    def connect(self):
        socket = self.socket.get()
        if (socket==''):
            tkMessageBox.showinfo(message='Please Enter socket address, port # (127.0.0.1, 80)')
        else:
            tkMessageBox.showinfo(message='Connected!')
            self.setLabel(self.status, 'Connected')

    def calibration(self):
        if self.status['text'] != 'Connected':
            tkMessageBox.showinfo(message='Please connect first')
        else:
            tkMessageBox.showinfo(message='Start calibration on Spyndra')

    def stand(self):
        if self.status['text'] != 'Connected':
            tkMessageBox.showinfo(message='Please connect first')
        else:
            tkMessageBox.showinfo(message='Sent standing signal to Spyndra')

    def train(self):
        if self.status['text'] != 'Connected':
            tkMessageBox.showinfo(message='Please connect first')
        else:
            tkMessageBox.showinfo(message='Start training on Spyndra')

    def move(self):
        if self.status['text'] != 'Connected':
            tkMessageBox.showinfo(message='Please connect first')
        else:
            tkMessageBox.showinfo(message='Sent random moving signal to Spyndra')




def main():
    rospy.init_node("control_node")
    n = Listener()
    rospy.spin()

if __name__ == '__main__':
    main()
