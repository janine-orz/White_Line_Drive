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
        self.L = []
        self.LoC = 0
        self.slope = 0


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
        Col = [] #initialize the color array
        for j in range(width):
            color = self.ColRec(img, i, j)
            if color == "White":
            #sucess to find the white color
                temp = [i, j]
                Col.append(temp)
        return Col


    def LinePos(self, img, height, width, i):
        Line = []
        length = 0
        Line = self.ColDet(img, height, width, i)

        #for i in range(len(Line)):
            #print(i, ':', Line[i])

        #sucess to finde the white color in one line
        length = len(Line) - 2
        if Line != []:
            INDEX = self.FindRef(Line, length)
            #print('INDEX = ', INDEX)
            Line_N = self.DelRef(Line, INDEX)
            #print('Line_N = ', Line_N)
            return Line_N


    def FindRef(self, Line, length):
        INDEX = []
        idx = 0
        INDEX.append(idx)
        #for idx in range(idx, len(Line)):
        for idx in range(length + 2):
            if(idx == length + 1):
                idx_n = length + 1
                INDEX.append(idx_n)
                return INDEX
            else:
                idx_n = self.bubblesort(length, idx, Line)
                #print('idx_n = ', idx_n)
                if(idx == idx_n):
                    INDEX.append(idx_n)


    def DelRef(self, Line, INDEX):
        Line_n = []
        #print('INDEX = ', INDEX)
        for i in range(0, len(INDEX)-1):
            diff = abs(INDEX[i+1] - INDEX[i])
            idx = INDEX[i+1]
            #print('idx = ', idx)
            #print('i = ', i, '; diff = ', diff)

            while (diff >= 5 and diff <= 13):
                num = Line[idx]
                if(len(INDEX) < 3) or (num[1] < 310):
                    Line_n = []
                    for j in range(INDEX[i], INDEX[i+1]):
                        #print('INDEX[i] = ', INDEX[i])
                        #print('j = ', j+1, ';\tLine[j]', Line[j+1])
                        Line_n.append(Line[j+1])
                i+=1
                break

        return Line_n


    def bubblesort(self, length, idx, Line):
        j = idx
        #while i >= idx and i <= length:
        for i in range(idx, length + 1):
            Pos1 = Line[i]
            Num1 = Pos1[1]
            Pos2 = Line[i+1]
            Num2 = Pos2[1]
            if((Num2 - Num1) == 1):
                j += 1
            else:
                idx = j
                return idx


    def LineForm(self, img, height, width, i_min, i_max):
        L = []
        for i in range (i_min, i_max):
            # print('i = ', i, ', min = ', i_min, ', max = ', i_max)
            Line_temp = self.LinePos(img, height, width, i)
            l = len(Line_temp)
            a = Line_temp[0]
            num1 = a[1]
            b = Line_temp[l-1]
            num2 = b[1]
            if(num1 != None)and(num2 != None):
                temp = [int(self.MidCal(num1, num2)), i]
                # print('[', MidCal(a, b), ',', i, ']')
                L.append(temp)
        return L


    def MidCal(self, a, b):
        num1 = (a - b) / 2.0
        num2 = b + int(num1)
        num2 = int(num2)
        return num2


    def LineorCurve(self, Line):
        LoC = 0
        Pnt1 = Line[0]
        #print('Pnt1 = ', Pnt1)
        Pnt2 = Line[len(Line) - 1]
        #print('Pnt2 = ', Pnt2)
        num = len(Line) - 15
        num = int(num)
        Pnt3 = Line[num]
        #print('Pnt3 = ', Pnt3[1])
        y1 = Pnt3[1]
        Pnt4 = Pnt3
        slope = (Pnt2[1] - Pnt1[1])/(Pnt2[0] - Pnt1[0])
        #print(slope)
        const = Pnt1[1] - (Pnt1[0] * slope)
        y2 = (slope * Pnt4[0]) + const
        #print('Pnt4 = ', y2)
        diff = y1 - y2
        print('Diff = ', diff)
        if(abs(diff) < 2.5):
            LoC = 0
        else:
            LoC = 1
        return LoC


    def LineSlope(self, Line):
        slope = 0.0 #initialize the slope
        Pnt1 = Line[0]
        Pnt2 = Line[len(Line) - 1]
        slope = self.GetAngl(Pnt1, Pnt2)
        print('slope = ', slope)
        if slope > 1.0015:
        # it should be 0.8015
        # the PERSPECTIVE angle 0.5
        #           plus
        # DIFFERENCE between pi and max radians of turtlebot Burger 0.3015
        # HOWEVER
        # we still need to concider about the extra slope
        # caused by the distance of the camera 0.2
            slope = slope - 1.0015
        elif (slope <= 1.0015) and (slope > 0):
            slope = 0.0
        elif (slope < -1.0015):
            slope = slope + 1.0015
        return slope


    def GetAngl(self, Pnt1, Pnt2):
        angle = 0.0
        opp = Pnt2[0] - Pnt1[0]
        adj = Pnt2[1] - Pnt1[1]
        angle = np.arctan(opp / adj)
        return angle


    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame...')
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        self.img = img_cv
        height, width, _ = img_cv.shape
        i_bot = height - 5
        i_mid = height - 45

        Line = self.LineForm(self.img, height, width, i_mid, i_bot)

        for i in range(len(Line)):
            Pnt = Line[i]
            print(Pnt)
            cv2.circle(self.img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 5)

        self.LoC = self.LineorCurve(Line)
        print('LoC = ', self.LoC)
        self.slope = self.LineSlope(Line)
        #publish the cv image back to publisher


        ''''''
        #to test if the Project will find the correct line
        for k in range(len(Line)):
            temp1 = Line[k]
            # print('(', temp1[0], temp1[1], ')')
            a = int(temp1[0])
            cv2.circle(self.img, (a, temp1[1]), 2, (0, 0, 255), 2)


        cv2.imshow("IMG", img_cv)
        #to show the cv image
        cv2.waitKey(1)
        ''''''


    def timer_callback(self):
        ''''''
        if(self.slope == 0.0):
        #call the class variable
        #since the publisher create the msg itself
        #you are not allow to use the msg in publisher
        #[ self.i < 20): ]
            #msg = Twist()
            #msg.linear.x = 0.02
            #msg.angular.z = 0.00
            #self.publisher_.publish(msg)
            print("Straight Driving!")
        elif(self.slope != 0.0):
            #if(self.i < 10):
                #msg = Twist()
                #msg.linear.x = 0.02
                #msg.angular.z = 0.00
                #self.publisher_.publish(msg)
                #print("Straight Driving!")
            #else:
            #msg = Twist()
            #msg.linear.x = 0.00
            #msg.angular.z = 0.00
            #self.publisher_.publish(msg)
            #self.i += 1
            print("Turning!")
            #self.i = 0
        ''''''


def main(args=None):
    rclpy.init(args=args)

    node = FollowLine()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
