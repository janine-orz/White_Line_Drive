import rclpy
import sys
import cv2
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

'''
DONE:
    - almost ok with left turning
    - maybe a great idea:
        if the robot can make sure it is just parallel to the line
    - but might sometimes still bothered from the reflections
TO DO:
    - still need to find out a way for the right turning
    - also how much does the slope should be
    - and fill the explanation:
        why I substract the constant to correct the slope

'''

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
        self.data = []


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


    #We use this function to find out all white points
    def ColDet(self, img, height, width, i):
        Col = [] #initialize the color array
        for j in range(width):
            color = self.ColRec(img, i, j)
            if color == "White":
            #sucess to find the white color
                temp = [i, j]
                Col.append(temp)
        return Col


    #We use this function to find out the white line
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
            Line_N = self.DelRef(Line, INDEX)
            return Line_N


    #This function is used to find out the reflection on the floor
    #by using the part function bubblesort
    def FindRef(self, Line, length):
        INDEX = []
        idx = 0
        INDEX.append(idx)
        for idx in range(length + 2):
            if(idx == length + 1):
                idx_n = length + 1
                INDEX.append(idx_n)
                return INDEX
            else:
                idx_n = self.bubblesort(length, idx, Line)
                if(idx == idx_n):
                    INDEX.append(idx_n)


    #This function is used to delete the reflection on the floor
    #by comparing the width of the white point groups
    #the white line will only be inbetween 5-13
    def DelRef(self, Line, INDEX):
        Line_n = []
        for i in range(0, len(INDEX)-1):
            diff = abs(INDEX[i+1] - INDEX[i])
            idx = INDEX[i+1]

            while(diff >= 8 and diff <= 33):
                num = Line[idx]
                for k in range(len(Line)-1, -1, -1):

                    if(num[1] < 320-k*5):
                        Line_n = []
                        for j in range(INDEX[i], INDEX[i+1]):
                            Line_n.append(Line[j+1])

                break
        return Line_n


    #This function is used to check all the white points
    #if there are next to each other
    #if yes return index of the last point
    def bubblesort(self, length, idx, Line):
        j = idx
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


    #This function is used to form a Line
    def LineForm(self, img, height, width, i_min, i_max):
        L = []
        for i in range (i_min, i_max):
        #inbetween the maximum height(lower bound) and
        #the minimim height(upper bound)
            Line_temp = self.LinePos(img, height, width, i)
            #we will find the white point in each height pixel
            if(Line_temp == None):
            #if there is no white point
            #then return a empty array L
                L = []
                return L
            elif(len(Line_temp) != 0):
            #if there is a white point
            #then add them to L
                l = len(Line_temp)
                a = Line_temp[0]
                num1 = a[1]
                b = Line_temp[l-1]
                num2 = b[1]
                if(num1 != None)and(num2 != None):
                #since we only need one point to calculate the line
                #we will use the middle Point
                #[>>>]Here is still a problem
                #would it be better if we use the point on the most left
                    temp = [int(self.MidCal(num1, num2)), i]
                    L.append(temp)
        return L


    #This function is used to calculate the middle point between two points
    def MidCal(self, a, b):
        num1 = (a - b)/2
        num2 = b + num1
        num2 = int(num2)
        #Since the Point should be a integer
        return num2


    #This function is used to check if the line is straight or not
    #if it is straight then LoC = 0
    #if it is a curve then Loc = 1
    def LineorCurve(self, Line):
        LoC = 0
        if(Line != []):
            Pnt1 = Line[0]
            Pnt2 = Line[len(Line) - 1]
            slope = (Pnt2[1] - Pnt1[1])/(Pnt2[0] - Pnt1[0])
            const = Pnt1[1] - (Pnt1[0] * slope)

            num1 = len(Line)/4
            num1 = int(num1)
            Pnt3 = Line[num1]
            LoC = self.LoCCal(Pnt3, slope, const)
            return LoC
        else:
            LoC = 2
            return LoC


    #This function is a part of the function LineorCurve
    #we check if the point Pnt3 stay on the stragith line
    #by calculating a Pnt4 through the slope and const
    def LoCCal(self, Pnt3, slope, const):
        y1 = Pnt3[1]
        Pnt4 = Pnt3
        y2 = (slope * Pnt4[0]) + const
        diff = y1 - y2
        print('diff = ', diff)
        if(abs(diff) < 6):
            LoC = 0
        else:
            LoC = 1
        return LoC





    #This function is used to calculate slope of a line
    def LineSlope(self, Line, LoC):
        slope = 0.0
        idx = 0
        if(LoC == 0):
        #if the line is a straight line
        #then we need to find three different pairs of points
        #and their corresponding slope
            idx = (len(Line))/4
            idx = 3 * idx
            idx = int(idx)
            slope = self.CalSlope(Line, idx)
            print('slope = ', slope)
        elif(LoC == 1):
        #if the line is a curve
        #then we only need to calculate the slope of the pairs at bottom
            idx = (len(Line))/4
            idx = 3 * idx
            idx = int(idx)
            slope = self.CalSlope(Line, idx)
            print('slope = ', slope)
        elif(LoC == 2):
        #if the robot cannot find a line
        #then it will give the value of LoC
        #so that the robot will not give back an Error
            return LoC

        if slope > 0.1835:#[!!!]MIGHT NOT CORRECT
        # it should be 0.8015
        # the PERSPECTIVE angle 0.5
        #           plus
        # DIFFERENCE between pi and max radians of turtlebot Burger 0.3015
        # HOWEVER
        # we still need to concider about the extra slope
        # caused by the distance of the camera 0.2
            slope = slope - 0.1565
        elif (slope <= 0.1835) and (slope > 0.0615):
            slope = 0.0
        elif (slope < 0.0615):
            slope = slope - 0.0615
        return slope


    #This function is used to calculate the slope of
    #a straight linebetween two points
    def CalSlope(self, Line, idx):
        l = len(Line) - 1
        Pnt1 = Line[l]
        Pnt2 = Line[idx]
        slope = self.GetAngl(Pnt2, Pnt1)
        return slope


    #use this function to calculate the opposite edge and the adjacent edge
    #so that we can calculate the angle by using tanh
    def GetAngl(self, Pnt1, Pnt2):
        angle = 0.0
        opp = Pnt2[0] - Pnt1[0]
        adj = Pnt2[1] - Pnt1[1]
        angle = np.arctan(opp / adj)
        return angle


    #the callback function of the subscription
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame...')
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        height, width, _ = img_cv.shape
        i_bot = height - 5
        i_mid = height - 40
        width_n = width - 180

        out_pnt = np.float32([[0, 0],
                        [0, height-1],
                        [width-1, height-1],
                        [width-1, 0]])
        inp_pnt = np.float32([[width_n - 58, i_mid],
                        [width_n, i_bot],
                        [width-15, i_bot],
                        [300, i_mid]])
        maxw = width - 1 - width_n
        maxh = height - 1 - i_mid
        M = cv2.getPerspectiveTransform(inp_pnt, out_pnt)
        img_cv = cv2.warpPerspective(img_cv, M, (width, height), flags = cv2.INTER_LINEAR)

        minwid = 0
        maxwid = height

        Line = self.LineForm(img_cv, height, width, minwid, maxwid)

        self.LoC = self.LineorCurve(Line)
        print('LoC = ', self.LoC)
        self.slope = self.LineSlope(Line, self.LoC)
        #print('Slope = ', self.slope)

        #if self.i < 10:
            #self.data.append([self.LoC, self.slope])
            #print(self.data)
        #elif self.i >= 10:
            #del(self.data[0])
            #self.data.append([self.LoC, self.slope])


        #publish the cv image back to publisher

        ''''''
        #to test if the Project will find the correct line
        for i in range(len(Line)):
            Pnt = Line[i]
            cv2.circle(img_cv, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 5)
        ''''''

        img_hsv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HSV)
        cv2.imshow("IMG_HSV", img_hsv)
        cv2.imshow("IMG_CV", img_cv)
        #to show the cv image
        cv2.waitKey(1)
        ''''''


    def timer_callback(self):
        if(self.LoC == 0):
        #call the class variable
        #since the publisher create the msg itself
        #you are not allow to use the msg in publisher
        #[ s    elf.i < 20): ]
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("Straight Driving!")
        elif(self.LoC == 1):
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = self.slope
            self.publisher_.publish(msg)
            print("Turning!")
        elif(self.LoC == 2):
        #'LoC=2' means that the robot didn't find a white line
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("STOP!")

        #self.i += 1
        ''''''

    #colcon build --packages-select pub_img
    #source install/setup.bash
    #ros2 run pub_img follow
    #ros2 run turtlebot3_teleop teleop_keyboard


def main(args=None):
    rclpy.init(args=args)

    node = FollowLine()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()
