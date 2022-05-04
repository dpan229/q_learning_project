import rospy, cv2, cv_bridge
import numpy as np

from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, Vector3

from q_learning_project.msg import RobotMoveObjectToTag

class RobotMover:
    def __init__(self):

        self.bridge = cv_bridge.CvBridge()

        cv2.namedWindow("window", 1)

        self.image_sub = rospy.Subscriber('camera/rbg/image_raw',
            Image, self.image_callback)

        self.action_sub = rospy.Subscriber('q_learning/robot_action', 
            RobotMoveObjectToTag, self.action_callback)

        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        self.current_color = -1
        self.current_tag = -1

        self.colored_centers = []
        self.tag_centers = []
        self.tag_ids = []
        self.front_distances = [1.0 for _ in range(10)]
        self.front_distance = 1.0

    def set_colored_object_centers(self, image):

        color_ranges = [
            # red
            (numpy.array([0, 0, 200]), numpy.array([60, 60, 255])),
            # green
            (numpy.array([0, 200, 0]), numpy.array([60, 255, 60])),
            # blue
            (numpy.array([200, 0, 0]), numpy.array([255, 60, 60])),
        ]

        centers = []

        for lower, upper in color_ranges:
            mask = cv2.inRange(image, lower, upper)
            center = cv2.moments(mask)

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

        self.tag_centers = [(corners[i, 0, 0] + corners[i, 0, 2]) / 2 for i in range(corners.shape[0])]
        self.tag_ids = ids[:, 0]

    def image_callback(self, msg):
        if not msg.data:
            # no image
            return

        self.img_width = msg.width

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        self.set_colored_object_centers(image)

        self.set_tag_centers(image)

    def scan_callback(self, data):
        self.front_distances = [data.ranges[0]] + self.front_distances[1:]
        self.front_distance = np.mean(self.front_distances)
    
    def move_object(self, color_id, tag_id):
        center = self.centers[color_id]
        # go in front of object
        r = rospy.Rate(10)
        while not (abs(center - self.img_width / 2) < 10 and abs(self.front_distance - 0.2) < 0.02):
            center = self.colored_centers[color_id]

            if center == (-1, -1):
                # object is not detected, rotate in place
                lin = Vector3(0.0, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, 0.3)
            else:
                # move towards object
                # proportional control for forward and angle
                lin = Vector3((self.front_distance - 0.2) * 0.1, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[1]) * 0.5)
            twist = Twist(linear=lin, angular=ang)
            self.vel_pub.publish(twist)

            r.sleep()

        # pick up object with claw

        # go to the tag
        center = 0
        while not (abs(center - self.img_width / 2) < 10 and abs(self.front_distance - 0.2) < 0.02):
            try:
                # move towards tag
                tag_center = self.tag_centers[list(self.tag_ids).index(tag_id)]

                # proportional control for forward and angle
                lin = Vector3((self.front_distance - 0.2) * 0.1, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[1]) * 0.5)
                twist = Twist(linear=lin, angular=ang)
                self.vel_pub.publish(twist)
            except ValueError:
                # tag is not detected, rotate in place
                lin = Vector3(0.0, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, 0.3)
                twist = Twist(linear=lin, angular=ang)
                self.vel_pub.publish(twist)

            r.sleep()

        # put the object down


    def action_callback(self, data):

        colors = {
            "red": 0,
            "green": 1,
            "blue": 2
        }
        color = colors[data.robot_object]
        tag = data.tag_id

        self.move_object(color, tag)


    def run(self):
