import rclpy
import sys
import cv2
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError


class FollowLine(Node):

    def __init__(self):
        super().__init__('follow_line')
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.listener_callback, qos_profile = qos_policy)
        #[ , 10) ]as alternative choice
        self.bridge = CvBridge()
        #self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile=qos_policy)

        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0
        self.img = 0


    def ColRec(self, img, i, j):
        color = "Undefined"
        pixel_center = img[i, j]

        b_val = pixel_center[0]
        g_val = pixel_center[1]
        r_val = pixel_center[2]
        #AFTER TESTS: the bgr color is inverse to the normal colorpicker
        #             we used in the daily life so the difference between
        #             the 255 and the value would be the bgr we use in cv

        if b_val > 150 and g_val > 150 and r_val > 150:
            color = "White"
        elif b_val < 100 and g_val < 100 and r_val < 100:
            color = "Black"
        else:
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            pixel_center = hsv_img[i, j]
            h_val = pixel_center[0]
            if h_val < 5:
                color = "Red"
            elif h_val < 22:
                color = "Orange"
            elif h_val < 33:
                color = "Yellow"
            elif h_val < 78:
                color = "Green"
            elif h_val < 131:
                color = "Blue"
            elif h_val < 167:
                color = "Violet"
            else:
                color = "Red"
        return color


    def ColDet(self, img, height, width, i):
        Col = []
        # i = height - 50
        for j in range(width):
            color = self.ColRec(img, i, j)
            if color == "White":
            #sucess to find the white color
                temp = [i, j]
                Col.append(temp)
        return Col


    def LinePos(self, img, height, width, i):
        Line = self.ColDet(img, height, width, i)
        #print(Line)
        #sucess to finde the white color in one line
        length = len(Line)
        if Line != []:
            Line_max = Line[length - 1]
            cy_max = Line_max[1]
            self.FindMax(Line, length, cy_max)
            #print('cy_max = ', cy_max)
            ##sucess to finde the right white point
            cy_min = self.FindMin(Line, length)
            #print('cy_min = ', cy_min)
            #sucess to finde the left white point
            return cy_max, cy_min


    def FindMin(self, Line, length):
        Line_max = Line[length - 1]
        maximum = Line_max[1]
        Line_min = Line[0]
        minimum = Line_min[1]
        for k in range(length-1):
            Line_temp = Line[k]
            tempmin = Line_temp[1]
            while abs(tempmin - maximum) <= 10:
                minimum = tempmin
                #print(minimum)
                Line_min = Line_temp
                return minimum


    def FindMax(self, Line, length, maximum):
        for k in range(length-1):
            Line_temp = Line[k+1]
            if maximum > Line_temp[1]:
                maximum = maximum
            elif maximum < Line_temp[1]:
                maximum = Line_temp[1]
        return maximum


    def LineForm(self, img, height, width, i_min, i_max):
        L = []
        for i in range (i_min, i_max):
            # print('i = ', i, ', min = ', i_min, ', max = ', i_max)
            a = self.LinePos(img, height, width, i)[0]
            b = self.LinePos(img, height, width, i)[1]
            temp = [int(self.MidCal(a, b)), i]
            # print('[', MidCal(a, b), ',', i, ']')
            L.append(temp)
        return L


    def MidCal(self, a, b):
        num1 = (a - b)/2
        num2 = b + int(num1)
        num2 = int(num2)
        return num2


    def LineSlope(self, Line):
        Pnt1 = Line[0]
        Pnt2 = Line[len(Line) - 1]
        slope = self.GetAngl(Pnt1, Pnt2)
        print('slope = ', slope)
        if slope > 0.75:
            slope = slope - 0.75
        elif slope <= 0.75:
            slope = 0.0
        return slope

    def GetAngl(self, Pnt1, Pnt2):
        opp = Pnt2[0] - Pnt1[0]
        adj = Pnt2[1] - Pnt1[1]
        angle = np.arctan(opp / adj)
        '''
        angle = np.arctan(opp / adj)
        print(angle)
        Ang.append(angle)
        add = 0
        for i in range(len(Ang)):
            add += abs(Ang[i])
        angle = add / (len(Ang))'''

        return angle



    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame...')
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        self.img = img_cv
        height, width, _ = img_cv.shape
        i_bot = height - 5
        i_mid = height - 30
        Line = self.LineForm(self.img, height, width, i_mid, i_bot)
        print(self.LineSlope(Line))
        #publish the cv image back to publisher


        #to test if the Project will find the correct line
        for k in range(len(Line)):
            temp1 = Line[k]
            # print('(', temp1[0], temp1[1], ')')
            a = int(temp1[0])
            cv2.circle(self.img, (a, temp1[1]), 2, (0, 0, 255), 2)


        cv2.imshow("IMG", img_cv)
        #to show the cv image
        cv2.waitKey(1)


    def timer_callback(self):
        '''
        if(self.direction >= 1.1):
        #call the class variable
        #since the publisher create the msg itself
        #you are not allow to use the msg in publisher
        #[ self.i < 20): ]
            msg = Twist()
            msg.linear.x = 0.05
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("driving")
        else:
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("STOP!")
        '''
        self.get_logger().info('Publishing: "%i"' % self.i)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    node = FollowLine()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
