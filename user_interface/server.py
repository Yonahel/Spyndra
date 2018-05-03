# server program
import socket
import sys
import threading
import subprocess

class Server:
    def __init__(self, host, port):
        self.host = host
        self.port = port
        self.connected = False
        self.connections = []
        self.sock = socket.socket()
        self.sock.bind((host, port))
        self.start_listen(5)

    def start_listen(self, n):
        print "started listening..."
        self.sock.listen(5)
        # while True:
        c, addr = self.sock.accept()
        print "connected to: ", addr[0], ", ", addr[1]
        self.connected = True
        # #start running roscore
        # t_core = threading.Thread(target=self.threaded_run, args=['roscore'])
        # t_core.start()

        # #source dir to add spyndra to ros path, requires sudo, better to add to path permanently
        # t_src = threading.Thread(target=self.threaded_run, args=['. catkin_ws/devel/setup.bash'])
        # t_src.start()            
        
        # #start control node
        # t_ctrl = threading.Thread(target=self.threaded_run, args=['rosrun spyndra control_node.py'])
        # t_ctrl.start()

        # start recv messages
        t = threading.Thread(target=self.threaded_recv, args=[c])
        t.start()

    def threaded_recv(self, connection):
        while self.connected:
            data = connection.recv(1024)
            print data

            # calibration
            if data == '0':
                # note there might be a bad descriptor problem when user node publishes command code
                t_user = threading.Thread(target=self.threaded_run, args=['rosrun spyndra user.py 0'])#, kwargs={'stdin_msg':data})
                t_user.start()  

            # stand
            if data == '1':
                # start user node
                t_user = threading.Thread(target=self.threaded_run, args=['rosrun spyndra user.py 1'])#, kwargs={'stdin_msg':data})
                t_user.start()     

            # train
            if data == '2':
                # start user node
                t_user = threading.Thread(target=self.threaded_run, args=['python /catkin_ws/src/spyndra/src/A3C_spyndra.py'])
                t_user.start()      

            # movement
            if data == '3':
                # start user node
                t_user = threading.Thread(target=self.threaded_run, args=['rosrun spyndra user.py 3'])
                t_user.start()      

            # stop the connection when receive -1
            elif data == '-1':
                try:
                    self.sock.send('-1')
                except:
                    pass
                subprocess.Popen('killall -9 roscore')
                subprocess.Popen('rosnode kill --all')
                self.connected = False
                sys.exit(0)

    def threaded_run(self, command,stdin_msg=''):
        p = subprocess.Popen(command.split(), stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        # if want to input to the subprocess' stdin, use the optional arg stdin_msgs
        if stdin_msg:
            p.communicate(input=stdin_msg)


    def send_signal(self):
        while True:
            self.sock.send('0thisisatestmsg\n')



def main():
    s = Server('localhost', 8888)

if __name__ == '__main__':
    main()