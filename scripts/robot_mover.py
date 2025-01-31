#!/usr/bin/env python3

import rospy, cv2, cv_bridge
import numpy as np
# import the moveit_commander, which allows us to control the arms
import moveit_commander
import math

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3
from lab_f_traffic_bot.msg import Traffic

from q_learning_project.msg import RobotMoveObjectToTag, QLearningReward
from time import sleep

class RobotMover:
    def __init__(self):

        rospy.init_node("robot_mover")

        self.bridge = cv_bridge.CvBridge()

        #cv2.namedWindow("window", 1)

        self.action_sub = rospy.Subscriber('q_learning/robot_action', 
            RobotMoveObjectToTag, self.action_callback)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        self.reward_pub = rospy.Publisher('q_learning/reward', QLearningReward, queue_size=10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        self.image_sub = rospy.Subscriber('camera/rgb/image_raw',
            Image, self.image_callback)

        self.current_color = -1
        self.current_tag = -1

        self.colored_centers = [(-1, -1) for _ in range(3)]
        self.tag_centers = []
        self.tag_ids = []
        self.front_distance = 1.0

        self.img_width = 100

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator arm
        self.move_group_arm = moveit_commander.MoveGroupCommander("arm")

        # the interface to the group of joints making up the turtlebot3
        # openmanipulator gripper
        self.move_group_gripper = moveit_commander.MoveGroupCommander("gripper")

        # Reset arm position
        self.move_group_arm.go([0.0, 
                                math.radians(-60.0), 
                                math.radians(60.0),
                                math.radians(0.0)], wait=True)
        self.move_group_gripper.go([0.01, 0.01], wait=True)

    def set_colored_object_centers(self, image):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        color_ranges = [
            # pink
            (np.array([150, 128, 75]), np.array([170, 255, 255])),
            # green
            (np.array([40, 100, 75]), np.array([70, 255, 255])),
            # blue
            (np.array([67, 100, 60]), np.array([110, 255, 255])),
        ]

        centers = []

        for lower, upper in color_ranges:
            mask = cv2.inRange(hsv, lower, upper)

            # this limits our search scope to only view a slice of the image near the ground
            h, w, d = image.shape
            search_top = int(h/2)
            search_bot = int(h)
            mask[0:search_top, 0:w] = 0
            mask[search_bot:h, 0:w] = 0

            M = cv2.moments(mask)

            if M['m00'] > 100:
                # center of the yellow pixels in the image
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])

                # a red circle is visualized in the debugging window to indicate
                # the center point of the yellow pixels
                cv2.circle(image, (cx, cy), 20, (0,0,255), -1)

                centers.append((cx, cy))
            else:
                centers.append((-1, -1))

        self.colored_centers = centers

    def set_tag_centers(self, image):
        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

        self.tag_centers = [(corners[i][0, 0] + corners[i][0, 2]) / 2 for i in range(len(corners))]
        if ids is None:
            self.tag_ids = []
        else:
            self.tag_ids = ids[:, 0]

    def image_callback(self, msg):
        #print("got image")
        if not msg.data:
            # no image
            print("no image")
            return

        self.img_width = msg.width

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array([67, 100, 75]), np.array([110, 255, 255]))

        cv2.imshow("window", image)
        cv2.imshow("window", mask)
        cv2.waitKey(3)

        self.set_colored_object_centers(image)

        self.set_tag_centers(image)

    def scan_callback(self, data):
        self.front_distance = np.mean([data.ranges[i] for i in [0, 1, 2, 359, 358]])
    
    def move_object(self, color_id, tag_id):
        center = self.colored_centers[color_id]
        # go in front of object
        print("Robot mover: Approaching object")
        r = rospy.Rate(10)
        while not (abs(center[0] - self.img_width / 2) < 10 and abs(self.front_distance - 0.15) < 0.02):
            center = self.colored_centers[color_id]

            if center == (-1, -1):
                # object is not detected, rotate in place
                lin = Vector3(0.0, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, 0.6)
            else:
                # move towards object
                # proportional control for forward and angle
                lin = Vector3(min((self.front_distance - 0.15) * 0.4, 0.5), 0.0, 0.0)
                ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[0]) * 0.01)
            twist = Twist(linear=lin, angular=ang)
            self.vel_pub.publish(twist)

            r.sleep()

        lin = Vector3(0.0, 0.0, 0.0)
        ang = Vector3(0.0, 0.0, 0.0)
        twist = Twist(linear=lin, angular=ang)
        self.vel_pub.publish(twist)

        print("Robot mover: grabbing")
        self.claw_grab()

        print("Robot mover: moving to tag")
        # go to the tag
        center = (-1, -1)
        while not (abs(center[0] - self.img_width / 2) < 10 and abs(self.front_distance - 0.5) < 0.02):
            try:
                # move towards tag
                center = self.tag_centers[list(self.tag_ids).index(tag_id)]
                #print(f'centers are {self.tag_centers}')
                #print(f'ids are {self.tag_ids}')

                # proportional control for forward and angle
                lin = Vector3(min((self.front_distance - 0.5) * 0.4, 0.5), 0.0, 0.0)
                ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[0]) * 0.01)
                twist = Twist(linear=lin, angular=ang)
                self.vel_pub.publish(twist)
            except ValueError:
                # tag is not detected, rotate in place
                lin = Vector3(0.0, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, 0.6)
                twist = Twist(linear=lin, angular=ang)
                self.vel_pub.publish(twist)

            r.sleep()

        lin = Vector3(0.0, 0.0, 0.0)
        ang = Vector3(0.0, 0.0, 0.0)
        twist = Twist(linear=lin, angular=ang)
        self.vel_pub.publish(twist)

        print("Robot mover: putting down")
        self.claw_open()

        print("Robot mover: task finished")
    
    def claw_grab(self):
        # controls the arm joints
        # intention: have the second arm join to move forward so that the gripper is in front of the bot
        arm_joint_goal = [0.0, 0.0, math.radians(20.0), math.radians(-20.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        rospy.sleep(3.0)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

        # Controls the gripper
        # intention: close gripper onto object
        # unknown: how far the gripper is and how tight the gripper is on the object
        gripper_joint_goal = [-0.01, -0.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        rospy.sleep(1.0)
        self.move_group_gripper.stop()

        # controls arm joints to move up
        # intention: have the third arm joint to move upwards 
        arm_joint_goal = [0.0, math.radians(-60.0), math.radians(50.0), math.radians(-90.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        rospy.sleep(3.0)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

    def claw_open(self):
        # controls the arm joints
        arm_joint_goal = [0.0, math.radians(30.0), math.radians(-30.0), math.radians(-15.0)]
        self.move_group_arm.go(arm_joint_goal, wait=True)
        rospy.sleep(5.0)
        # Calling ``stop()`` ensures that there is no residual movement
        self.move_group_arm.stop()

        # Controls the gripper
        # intention: open gripper
        gripper_joint_goal = [0.01, 0.01]
        self.move_group_gripper.go(gripper_joint_goal, wait=True)
        rospy.sleep(1.0)
        self.move_group_gripper.stop()

        # maybe have the robot back out after dropping the object
        self.move_group_arm.go([0.0, 
                                math.radians(-60.0), 
                                math.radians(60.0),
                                math.radians(0.0)], wait=True)
        rospy.sleep(2.0)



    def action_callback(self, data):

        colors = {
            "pink": 0,
            "green": 1,
            "blue": 2
        }
        color = colors[data.robot_object]
        tag = data.tag_id

        self.move_object(color, tag)
        print("Action done")

        # Signifies that the whole movement is finished
        self.reward_pub.publish(reward = 1)
        

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = RobotMover()
    sleep(0.1)
    node.run()