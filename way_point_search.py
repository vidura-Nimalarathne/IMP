#!/usr/bin/env python
import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from geometry_msgs.msg import Pose, Point, Quaternion, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import Bool
from tf.transformations import quaternion_from_euler

class WaypointDogSearch:
    def __init__(self):
        rospy.init_node('waypoint_dog_search')

        # Publisher to stop robot if needed
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

        # Subscribe to a simple dog detection flag
        # Your detector should publish True when dog confidently detected
        rospy.Subscriber('/dog_detected', Bool, self.dog_callback)

        self.move_base = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base.wait_for_server(rospy.Duration(60))
        rospy.loginfo("Connected to move_base")

        self.dog_found = False
        self.current_waypoint_index = 0
        self.pause_at_waypoint = rospy.get_param("~pause_at_waypoint", 2.0)
        self.retry_limit = rospy.get_param("~retry_limit", 1)

        self.waypoints = self.build_waypoints()

    def build_waypoints(self):
        # Replace these with your real map coordinates from RViz
        points = [
            (0.5, 0.0, 0.0),
            (1.5, 0.0, 0.0),
            (1.5, 1.0, 1.57),
            (0.5, 1.0, 3.14),
        ]

        waypoints = []
        for x, y, yaw in points:
            q = quaternion_from_euler(0, 0, yaw)
            pose = Pose(
                Point(x, y, 0.0),
                Quaternion(q[0], q[1], q[2], q[3])
            )
            waypoints.append(pose)
        return waypoints

    def dog_callback(self, msg):
        if msg.data and not self.dog_found:
            rospy.logwarn("DOG DETECTED! Cancelling current goal.")
            self.dog_found = True
            self.move_base.cancel_all_goals()
            self.stop_robot()

    def stop_robot(self):
        twist = Twist()
        self.cmd_vel_pub.publish(twist)

    def send_goal(self, pose):
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = pose

        self.move_base.send_goal(goal)
        rospy.loginfo("Sent waypoint goal")

    def run(self):
        rate = rospy.Rate(2)

        while not rospy.is_shutdown():
            if self.dog_found:
                rospy.loginfo("Dog found. Search stopped.")
                self.stop_robot()
                break

            pose = self.waypoints[self.current_waypoint_index]
            retries = 0
            reached = False

            while retries <= self.retry_limit and not rospy.is_shutdown():
                self.send_goal(pose)

                finished = self.move_base.wait_for_result(rospy.Duration(90))

                if self.dog_found:
                    break

                if not finished:
                    rospy.logwarn("Waypoint timeout. Cancelling and retrying.")
                    self.move_base.cancel_goal()
                    retries += 1
                    continue

                state = self.move_base.get_state()

                if state == GoalStatus.SUCCEEDED:
                    rospy.loginfo("Reached waypoint %d", self.current_waypoint_index)
                    reached = True
                    rospy.sleep(self.pause_at_waypoint)
                    break
                else:
                    rospy.logwarn("Failed to reach waypoint %d, state=%d",
                                  self.current_waypoint_index, state)
                    retries += 1
                    rospy.sleep(1.0)

            if self.dog_found:
                break

            # Move to next waypoint whether success or after retries
            self.current_waypoint_index = (self.current_waypoint_index + 1) % len(self.waypoints)
            rate.sleep()

if __name__ == '__main__':
    try:
        node = WaypointDogSearch()
        node.run()
    except rospy.ROSInterruptException:
        pass
