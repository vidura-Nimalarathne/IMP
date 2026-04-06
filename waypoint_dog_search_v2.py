#!/usr/bin/env python
import rospy
import actionlib

from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus

class WaypointDogSearch(object):
    def __init__(self):
        rospy.init_node('waypoint_dog_search')

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('/dog_detected', Bool, self.dog_callback)

        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)

        rospy.loginfo("Waiting for move_base action server...")
        connected = self.move_base.wait_for_server(rospy.Duration(60))
        if not connected:
            rospy.logerr("Could not connect to move_base action server.")
            raise rospy.ROSException("move_base not available")

        rospy.loginfo("Connected to move_base")

        self.dog_found = False
        self.pause_at_waypoint = rospy.get_param('~pause_at_waypoint', 2.0)
        self.goal_timeout = rospy.get_param('~goal_timeout', 90.0)
        self.retry_limit = rospy.get_param('~retry_limit', 1)
        self.rotate_speed = rospy.get_param('~rotate_speed', 0.45)
        self.rotate_duration = rospy.get_param('~rotate_duration', 5.0)

        self.waypoints = [
            ("HOME", -0.153910006488, -0.80646830771, 0.0, 0.0, 0.703697446404, 0.710499756456),
            ("WAY1", -0.0572225828625, -1.4411768054, 0.0, 0.0, 0.810552631646, 0.585665801744),
            ("WAY2", -0.45811827775, -1.71443295523, 0.0, 0.0, 0.465990015378, 0.884789978225),
        ]

    def dog_callback(self, msg):
        if msg.data and not self.dog_found:
            rospy.logwarn("DOG DETECTED! Cancelling current goal.")
            self.dog_found = True
            self.move_base.cancel_all_goals()
            self.stop_robot()

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    def build_goal(self, wp):
        name, x, y, qx, qy, qz, qw = wp

        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = Pose(
            Point(x, y, 0.0),
            Quaternion(qx, qy, qz, qw)
        )
        return goal

    def rotate_in_place(self):
        rospy.loginfo("Rotating in place to scan for dog...")
        twist = Twist()
        twist.angular.z = self.rotate_speed

        start_time = rospy.Time.now()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            if self.dog_found:
                break

            elapsed = (rospy.Time.now() - start_time).to_sec()
            if elapsed >= self.rotate_duration:
                break

            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        self.stop_robot()
        rospy.sleep(0.5)

    def go_to_waypoint(self, wp):
        name = wp[0]
        retries = 0

        while retries <= self.retry_limit and not rospy.is_shutdown():
            if self.dog_found:
                return False

            goal = self.build_goal(wp)
            rospy.loginfo("Sending goal: %s", name)
            self.move_base.send_goal(goal)

            finished = self.move_base.wait_for_result(rospy.Duration(self.goal_timeout))

            if self.dog_found:
                return False

            if not finished:
                rospy.logwarn("Timeout reaching %s. Cancelling goal.", name)
                self.move_base.cancel_goal()
                retries += 1
                rospy.sleep(1.0)
                continue

            state = self.move_base.get_state()

            if state == GoalStatus.SUCCEEDED:
                rospy.loginfo("Reached waypoint: %s", name)
                rospy.sleep(self.pause_at_waypoint)
                self.rotate_in_place()
                return True
            else:
                rospy.logwarn("Failed waypoint %s with state %d", name, state)
                retries += 1
                rospy.sleep(1.0)

        rospy.logwarn("Giving up on waypoint: %s", name)
        return False

    def run(self):
        index = 0
        total = len(self.waypoints)

        rospy.loginfo("Starting waypoint dog search with %d waypoints", total)

        while not rospy.is_shutdown():
            if self.dog_found:
                rospy.logwarn("Dog found. Stopping search.")
                self.stop_robot()
                break

            wp = self.waypoints[index]
            self.go_to_waypoint(wp)

            if self.dog_found:
                rospy.logwarn("Dog found after waypoint action.")
                self.stop_robot()
                break

            index = (index + 1) % total

if __name__ == '__main__':
    try:
        node = WaypointDogSearch()
        node.run()
    except rospy.ROSInterruptException:
        pass
