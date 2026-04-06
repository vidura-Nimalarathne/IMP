#!/usr/bin/env python
import socket
import time
import urllib2

import rospy
from std_msgs.msg import Bool

HOST = "127.0.0.1"
PORT = 5005

DETECT_ON_URL = "http://127.0.0.1:8080/detect_on"
DETECT_OFF_URL = "http://127.0.0.1:8080/detect_off"


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
            rospy.loginfo("Connected to local robot server")
            return True
        except Exception as e:
            rospy.logerr("Cannot connect to robot server: %s", e)
            self.sock = None
            return False

    def close_server(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
        self.sock = None

    def send_cmd(self, cmd):
        if not self.sock:
            return
        try:
            self.sock.sendall("<CMD:{}>".format(cmd).encode())
            rospy.loginfo("Sent command: %s", cmd)
        except Exception as e:
            rospy.logerr("Send failed for %s: %s", cmd, e)

    def stop_robot(self):
        self.send_cmd("S")

    def set_detection(self, enabled):
        url = DETECT_ON_URL if enabled else DETECT_OFF_URL
        label = "ON" if enabled else "OFF"

        try:
            urllib2.urlopen(url, timeout=1)
            rospy.loginfo("Detection %s", label)
        except Exception as e:
            rospy.logwarn("Failed to switch detection %s: %s", label, e)

    def dog_callback(self, msg):
        if msg.data and self.running:
            rospy.logwarn("DOG FOUND!")
            self.dog_found = True
            self.stop_robot()
            self.set_detection(False)

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
        if self.running:
            rospy.logwarn("Autonomous search already running")
            return

        self.running = True
        self.dog_found = False

        if not self.connect_server():
            self.running = False
            return

        self.set_detection(True)

        try:
            rospy.loginfo("START AUTONOMOUS SEARCH")

            rospy.loginfo("Step 1: Move forward for 3 seconds")
            if not self.run_motion("F", 3.0):
                return

            if self.dog_found:
                return

            rospy.loginfo("Step 2: Turn clockwise about 90 degrees")
            if not self.run_motion("D", 1.2):
                return

            if self.dog_found:
                return

            rospy.loginfo("Step 3: Move forward for 2 seconds")
            if not self.run_motion("F", 2.0):
                return

            if self.dog_found:
                return

            rospy.loginfo("SEARCH DONE")
            self.stop_robot()

        finally:
            self.stop_robot()
            self.set_detection(False)
            self.close_server()
            self.running = False


if __name__ == "__main__":
    node = AutonomousDogSearch()
    rospy.spin()
