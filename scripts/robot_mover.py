import rospy, cv2, cv_bridge, 
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
        self.corners = []

        self.centers = []
        self.scan = []

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

        self.centers = centers

    def image_callback(self, msg):
        if not msg.data:
            # no image
            return

        self.img_width = msg.width

        image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        #hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        self.set_colored_object_centers(image)

        grayscale_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        corners, ids, rejected_points = cv2.aruco.detectMarkers(grayscale_image, self.aruco_dict)

        self.corners = corners
        self.tag_ids = ids[:, 0]

    def scan_callback(self, data):
        self.scan = data.ranges
    
    def move_object(self, color_id, tag_id):
        center = self.centers[color_id]
        # go in front of object
        r = rospy.Rate(10)
        while not (np.abs(center - self.img_width / 2) < 10 and self.scan[0] < 0.2):
            center = self.centers[color_id]

            if center == (-1, -1):
                # object is not detected, rotate in place
                lin = Vector3(0.0, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, 0.3)
            else:
                # move towards object
                lin = Vector3(0.1, 0.0, 0.0)
                ang = Vector3(0.0, 0.0, (self.img_width / 2 - center[1]) * 0.5)
            twist = Twist(linear=lin, angular=ang)
            self.vel_pub.publish(twist)

            r.sleep()

        # pick up object with claw

        # go to the tag
        
        center = 0
        while not (np.abs(center - self.img_width / 2) < 10 and self.scan[0] < 0.2):
            try:
                # move towards tag
                tag_corners = self.corners[list(self.tag_ids).index(tag_id)][0]

                center = (tag_corners[0, 0] + tag_corners[2, 0]) / 2

                lin = Vector3(0.1, 0.0, 0.0)
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

        # go to colored object
        self.move_object(color, tag)


    def run(self):
