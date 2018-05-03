# import Tkinter as tk
from Tkinter import *
import tkMessageBox
import socket
import threading


class GUI:
    def __init__(self, master):
        #------------------------socket----------------------------
        self.sock = None
        self.mode = 0
        self.connected = False


        #------------------------graphics----------------------------
        self.master = master
        master.title("Spyndra Control Panel")

        frame_connection = Frame(master)
        frame_connection.pack()
        self.status = Label(frame_connection, text='Not connected, Enter socket address, port#:')
        self.status.pack(side=LEFT)
        self.socket_entry = Entry(frame_connection)
        self.socket_entry.pack(side=LEFT)
        self.connect_button = Button(frame_connection, text='Connect', command = self.connect)
        self.connect_button.pack(side=LEFT)

        frame_mode = Frame(master)
        frame_mode.pack()
        self.mode_notice = Label(frame_mode, text="Choose mode")
        self.mode_notice.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Calibration', command = self.calibration)
        self.standing_mode.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Stand', command = self.stand)
        self.standing_mode.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Training', command = self.train)
        self.standing_mode.pack(side=LEFT)
        self.standing_mode = Button(frame_mode, text='Movement', command = self.move)
        self.standing_mode.pack(side=LEFT)
        
        # placeholder = 'signal placeholder'

        # frame_motor = Frame(master)
        # frame_motor.pack()
        # self.motor = Text(frame_motor, height=10, width=80)
        # self.motor.insert(INSERT, "motor "+ placeholder)
        # self.motor.pack()        

        # frame_gps= Frame(master)
        # frame_gps.pack(side=BOTTOM)
        # self.gps = Text(frame_gps, height=10, width=80)
        # self.gps.insert(INSERT, "gps "+ placeholder)
        # self.gps.pack()

        # frame_imu = Frame(master)
        # frame_imu.pack()
        # self.imu = Text(frame_imu, height=25, width=80)
        # self.imu.insert(INSERT, "imu "+ placeholder)
        # self.imu.pack()

    def setText(self, text_obj, text):
        text_obj.delete('1.0', END)
        text_obj.insert(INSERT, text)

    def setLabel(self, label_obj, text):
        label_obj['text'] = text

    def connect(self):
        if self.connect_button['text'] == 'Connect':
            addr = self.socket_entry.get()
            if (addr=='') or len(addr.split(',')) != 2:
                tkMessageBox.showinfo(message='Please Enter socket address, port # (127.0.0.1, 80)')
            else:
                host, port = addr.split(',')
                self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                try:
                    self.sock.connect((host, int(port)))
                    tkMessageBox.showinfo(message='Connected!')
                    self.setLabel(self.status, 'Connected')
                    self.connect_button['text'] = 'Disconnect'
                    self.connected = True
                    self.start_recv()
                except:
                    tkMessageBox.showinfo(message='Connection failed, Please try again')
        elif self.connect_button['text'] == 'Disconnect':
            # stop the connection by sending -1
            self.sock.send('-1')
            self.sock.close()
            tkMessageBox.showinfo(message='Disconnected!')
            self.setLabel(self.status, 'Not connected, Enter socket address, port#:')
            self.connect_button['text'] = 'Connect'  
            self.connected = False          

    def start_recv(self):
        t = threading.Thread(target=self.threaded_recv, args=[])
        t.start()

    def threaded_recv(self):
        while self.connected:
            data = self.sock.recv(1024)
            print(data)

            # # set received text to section
            # self.setText(self.gps, data)

            # stop the connection by sending -1
            if data == '-1':
                try:
                    self.sock.send('-1')
                except:
                    pass
                self.sock.close()
                self.connected = False
                sys.exit(0)

    def calibration(self):
        if not self.connected:
            tkMessageBox.showinfo(message='Please connect first')
        else:
            self.mode = 0
            self.sock.sendall('0')
            tkMessageBox.showinfo(message='Start calibration on Spyndra')

    def stand(self):
        if not self.connected:
            tkMessageBox.showinfo(message='Please connect first')
        else:
            self.mode = 1
            self.sock.sendall('1')
            tkMessageBox.showinfo(message='Sent standing signal to Spyndra')

    def train(self):
        if not self.connected:
            tkMessageBox.showinfo(message='Please connect first')
        else:
            self.mode = 2
            self.sock.sendall('2')
            tkMessageBox.showinfo(message='Start training on Spyndra')

    def move(self):
        if not self.connected:
            tkMessageBox.showinfo(message='Please connect first')
        else:
            self.mode = 3
            self.sock.sendall('3')
            tkMessageBox.showinfo(message='Sent random moving signal to Spyndra')

def main():
    root = Tk()
    gui = GUI(root)
    root.mainloop()

if __name__ == '__main__':
    main()