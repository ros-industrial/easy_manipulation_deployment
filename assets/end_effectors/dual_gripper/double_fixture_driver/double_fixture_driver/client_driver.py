#!/usr/bin/env python

import socket
import time
import os
import rospkg

class DoubleFixtureDriver:
    def __init__(self, host, port):
        self.HOST = host
        self.PORT = port
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    def connect(self):
        self.s.connect((self.HOST, self.PORT))

    def activate_suction(self):
        cmd = "set_digital_out(0,True)" + "\n"
        self.s.send(cmd.encode('utf-8'))
        self.s.close()
        time.sleep(0.3)
        
    def deactivate_suction(self):
        cmd = "set_digital_out(0,False)" + "\n"
        self.s.send(cmd.encode('utf-8'))
        self.s.close()
        time.sleep(0.3)

    def activate_convey(self):
        cmd = "set_digital_out(4,True)" + "\n"
        self.s.send(cmd.encode('utf-8'))
        self.s.close()
        time.sleep(0.3)

    def deactivate_convey(self):
        cmd = "set_digital_out(4,False)" + "\n"
        self.s.send(cmd.encode('utf-8'))
        self.s.close()
        time.sleep(0.3)    

    def cmd_robotiq(self, cmd):
        path = os.path.dirname(__file__)
        # path = os.path.join(dirname)
        f = open("{}/scripts/{}_script.script".format(path, cmd), "rb")   #Robotiq Gripper
        with open("{}/scripts/{}_script.script".format(path, cmd), "rb") as f:
            self.s.sendall(f.read())
        time.sleep(0.8)