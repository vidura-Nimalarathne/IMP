#!/usr/bin/env python
import socket
import time
import rospy
from std_msgs.msg import Bool

HOST = "127.0.0.1"
PORT = 5005


class AutonomousDogSearch(object):
    def __init__(self):
        rospy.init_node("autonomous_dog_search")

        rospy.Subscriber("/dog_detected", Bool, self.dog_callback)
        rospy.Subscriber("/start_autonomous", Bool, self.start_callback)

        self.dog_found = False
        self.running = False
        self.sock = None

    def start_callback(self, msg):
        if msg.data and not self.running:
            rospy.loginfo("Received start signal from GUI")
            self.run_sequence()

    def connect_server(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((HOST, PORT))
            return True
        except:
            return False

    def send_cmd(self, cmd):
        if not self.sock:
            return
        try:
            self.sock.sendall(f"<CMD:{cmd}>".encode())
        except:
            pass

    def stop_robot(self):
        self.send_cmd("S")

    def dog_callback(self, msg):
        if msg.data and self.running:
            rospy.logwarn("DOG FOUND!")
            self.dog_found = True
            self.stop_robot()

    def run_motion(self, cmd, duration):
        start = time.time()
        rate = rospy.Rate(10)

        while (time.time() - start) < duration:
            if self.dog_found or not self.running:
                self.stop_robot()
                return False

            self.send_cmd(cmd)
            rate.sleep()

        self.stop_robot()
        time.sleep(0.3)
        return True

    def run_sequence(self):
        self.running = True
        self.dog_found = False

        if not self.connect_server():
            rospy.logerr("Cannot connect to robot")
            self.running = False
            return

        rospy.loginfo("START AUTONOMOUS SEARCH")

        if not self.run_motion("F", 3.0): return
        if not self.run_motion("D", 1.2): return
        if not self.run_motion("F", 2.0): return

        rospy.loginfo("SEARCH DONE")
        self.stop_robot()
        self.running = False


if __name__ == "__main__":
    node = AutonomousDogSearch()
    rospy.spin()
