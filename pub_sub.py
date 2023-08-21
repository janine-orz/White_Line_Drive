import rclpy
import sys
import cv2
import time
import numpy as np
from rclpy.node import Node

from std_msgs.msg import String
from geometry_msgs.msg import Twist
from sensor_msgs.msg import CompressedImage
from cv_bridge import CvBridge, CvBridgeError

'''
DONE:
    - ok to find the white line and green line, as well as the action path
    - even if the robot cannot see the whole line, it will fill the line by itself
TO DO:
    - the color still need to be correct
    17.30  2023-08-09
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
        # self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile=qos_policy)

        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.img = 0
        self.LoC = 0
        self.slope = 0


    def ColRec(self, img, i, j):
        color = "Undefined"
        pixel_center = []
        pixel_center = img[i, j]
        b_val = pixel_center[0]
        g_val = pixel_center[1]
        r_val = pixel_center[2]

        if b_val > 175 and g_val > 175 and r_val > 175:
            color = "White"
        elif b_val < 90 and g_val < 110 and r_val < 90:
            color = "Black"
        else:
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            pixel_center = hsv_img[i, j]
            h_val = pixel_center[0]
            s_val = pixel_center[1]
            v_val = pixel_center[2]
            if s_val >= 65 and v_val >= 65:
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


    # We use this function to find out all white points
    def ColDet(self, img, height, width, i, string):
        Col = []
        if(string == 'White'):
            temp = 0
            for j in range(width - 150, width, 1):
                color = self.ColRec(img, i, j)
                if color == string:
                #sucess to find the white color
                    color_a = self.ColRec(img, i, j-1)

                    ###########################################
                    #       how to avoid the reflection       #
                    # at the same time: would bother the line #
                    ###########################################

                    if (temp == 0) or (color_a != string):
                        temp = j
                    elif (j > temp) and (abs(temp-j) <= 20): 
                        temp = j
                    if(temp != 0):
                        temp_pos = [temp, i]
                        Col = temp_pos
        elif(string == 'Green'):
            temp = 0
            for j in range(0, width - 170, 1):
                color = self.ColRec(img, i, j)
                if color == string:
                #sucess to find the white color
                    color_a = self.ColRec(img, i, j-1)
                    if (temp == 0) or (color_a != string):
                        temp = j
                    elif (j > temp) and (abs(temp-j) <= 20): 
                        temp = j
                    if(temp != 0):
                        temp_pos = [temp, i]
                        Col = temp_pos
        return Col


    def LineForm(self, img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
        L = []
        L1 = []
        L2 = []
        WL = []
        GL = []
        i_min = max(Wi_min, Gi_min)
        i_max = min(Wi_max, Gi_max)
        # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        for i in range(i_min, i_max):
            L1 = self.ColDet(img, height, width, i, string1)
            L1 = self.IfLineBreak(WL, L1, i, i, string1, img, height, width)
            WL.append(L1)
            
            L2 = self.ColDet(img, height, width, i, string2)
            L2 = self.IfLineBreak(GL, L2, i, i, string2, img, height, width)
            GL.append(L2)
            # print("L1 = ", L1, '\tL1 bgr_value: ', img[L1[1], L1[0]], "\nL2 = ", L2, '\tL2 hsv_value: ', hsv_img[L2[1], L2[0]])

            x_temp = self.MidCal(L1[0], L2[0])
            temp = [x_temp, i]
            
            L.append(temp)

        L = self.UpperLine(img, height, width, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min)
        L = self.BottomLine(img, height, width, L, WL, GL, string1, string2, i_max, Gi_max, Wi_max)

        return L


    def IfLineBreak(self, L, Li, i, j, string, img, height, width):
        if((Li == [])or(Li == None)):
            # start = time.time()
            
            while (Li == []):
                i = i-1
                Li = L[-1]
                # print("i = ", i)
                # print("L[-1] = ", L[-1])
            if isinstance(Li, list):
                pos = Li[0]
                temp = [pos, j]
            elif isinstance(Li, int):
                temp = [Li, j]
            # while (Li == []):
            #     i = i-1
            #     Li = ColDet(img, height, width, i, string)
            # pos = Li

            
            # end = time.time()
            # print("\tLineForm IfLineBreak(): ", end-start)
            return temp
        else:
            return Li


    def UpperLine(self, img, height, width, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min):
        WWL = WL[0]
        GGL = GL[0]
        ##############################################
        #               if (GGL == []):              #
        # which position should be the green line in #
        ##############################################
        if(i_min == Gi_min): 
            # The lower bound of Whiteline is smaller then the Greenline
            for h in range (Gi_min, Wi_min, -1):
                print("------------ h = ", h, "------------")
                L1 = self.ColDet(img, height, width, h, string1)
                L1 = self.IfLineBreak(WWL, L1, h, h, string1, img, height, width)
                WL.append(L1)
                L2 = L1[0] - 220
                x_temp = self.MidCal(L1[0], L2)
                temp = [x_temp, h]
                L.append(temp)
        elif(i_min == Wi_min):
            for h in range (Wi_min, Gi_min, -1):
                L2 = self.ColDet(img, height, width, h, string2)
                L2 = self.IfLineBreak(GGL, L2, h, h, string2, img, height, width)
                GL.append(L2)
                L1 = L2[0] + 220
                x_temp = self.MidCal(L1, L2[0])
                temp = [x_temp, h]
                L.append(temp)
        return L


    def BottomLine(self, img, height, width, L, WL, GL, string1, string2, i_max, Gi_max, Wi_max):
        WWL = WL[-1]
        GGL = GL[-1]
        if(i_max == Gi_max): 
            # The lower bound of Whiteline is smaller then the Greenline
            for h in range (Gi_max, Wi_max, 1):
                L1= self.ColDet(img, height, width, h, string1)
                L1 = self.IfLineBreak(WWL, L1, h, h, string1, img, height, width)
                WL.append(L1)
                L2 = 0
                x_temp = self.MidCal(L1[0], L2)
                temp = [x_temp, h]
                L.append(temp)
        elif(i_max == Wi_max):
            for h in range (Wi_max, Gi_max):
                L2 = self.ColDet(img, height, width, h, string2)
                L2 = self.IfLineBreak(GGL, L2, h, h, string2, img, height, width)
                GL.append(L2)
                L1 = 319
                x_temp = self.MidCal(L1, L2[0])
                temp = [x_temp, h]
                L.append(temp)
        return L


    # This function is used to calculate the middle point between two points
    def MidCal(self, a, b):
        num1 = (a - b)/2
        if(a > b):
            num2 = b + num1
            num2 = int(num2)
            return num2
        elif(b > a):
            num2 = a + num1
            num2 = int(num2)
            # Since the Point should be a integer
            return num2


    # This function is used to check if the line is straight or not
    # if it is straight then LoC = 0
    # if it is a curve then Loc = 1
    def LineorCurve(self, Line, Gmaxhigt, Gminhigt, Wmaxhigt, Wminhigt):
        LoC = 0
        if(Line != []):
            Pnt1 = Line[0]
            Pnt2 = Line[len(Line) - 1]
            slope = (Pnt2[0] - Pnt1[0])/(Pnt2[1] - Pnt1[1])
            const1 = Pnt1[0] - (Pnt1[1] * slope)
            const2 = Pnt2[0] - (Pnt2[1] * slope)
            const = (const1 + const2)/2

            # print("slope = ", slope, "const = ", const)
            # calculate the slope ofthe whole line

            num1 = len(Line)/4
            num1 = int(num1)
            Pnt1 = Line[num1]
            LoC1 = self.LoCCal(Pnt1, slope, const)
            # comparing the LoC of a point at 1/4 of the line
            
            num2 = len(Line)/2
            num2 = int(num2)
            Pnt2 = Line[num2]
            LoC2 = self.LoCCal(Pnt2, slope, const)
            # comparing the LoC of a point at 1/2 of the line

            num3 = (len(Line)/4)*3
            num3 = int(num3)
            Pnt3 = Line[num3]
            LoC3 = self.LoCCal(Pnt3, slope, const)
            # comparing the LoC of a point at 3/4 of the line

            LoC_arr = np.array([LoC1, LoC2, LoC3])

            if(np.sum(LoC_arr == 0) >= 2):
            # if there is two or more LoC is judged as straight line
                LoC = 0
            elif(np.sum(LoC_arr == 1) >= 2):
            # if there is two or more LoC is judged as curve
                LoC = 1

            return LoC
        elif (Gmaxhigt == Gminhigt) and (Wmaxhigt == Wminhigt):
            LoC = 2
            return LoC
        else:
            LoC = 3
            # LoC is a situation that the line is not a straight line or a curve
            return LoC


    # This function is a part of the function LineorCurve
    # we check if the point Pnt3 stay on the stragith line
    # by calculating a Pnt4 through the slope and const
    def LoCCal(self, Pnt3, slope, const):
        y1 = Pnt3[0]
        Pnt4 = Pnt3
        y2 = (slope * Pnt4[1]) + const
        diff = y1 - y2
        if(abs(diff) < 2):
            LoC = 0
        else:
            LoC = 1
        return LoC


    # This function is used to calculate slope of a line
    def LineSlope(self, Line, LoC):
        slope = 0.0
        idx = 0
        l = len(Line) - 1

        idx2 = (len(Line))/5
        idx2 = 4 * idx2
        idx2 = int(idx2)
        
        if(LoC == 2):
        # if the robot cannot find a line
        # then it will give the value of LoC
        # so that the robot will not give back an Error
            slope = 0.00
            return slope
        
        # if the line is a straight line or a curve line
        # then we need to find three different pairs of points
        # and their corresponding slope
        slope = self.CalSlope(Line, l, idx2)
        return slope


    # This function is used to calculate the slope of
    # a straight linebetween two points
    def CalSlope(self, Line, idx1, idx2):
        Pnt1 = Line[idx1] # height of Pnt1 should be lower as the one of Pnt2
        Pnt2 = Line[idx2]
        # print('we will calculate the tangent line with Point', Pnt1, 'and Point', Pnt2)
        slope = self.GetAngl(Pnt1, Pnt2)
        return slope


    # use this function to calculate the opposite edge and the adjacent edge
    # so that we can calculate the angle by using tanh
    def GetAngl(self, Pnt1, Pnt2):
        angle = 0.0
        opp = Pnt2[0] - Pnt1[0]
        adj = Pnt2[1] - Pnt1[1]
        angle = np.arctan(opp / adj)
        return angle


    def DeterHeight1(self, img, height, width, idx1, idx2, string1, string2):
        if(idx1 <= idx2):
            a = 1
        elif(idx1 > idx2):
            a = -1
        for i in range(idx1, idx2, a):
        # for idx1 < idx2: Check from upper to bottom, if there is "string" points
        # for idx1 > idx2: Check from bottom to upper, if there is "string" points
            J = (self.ColDet(img, height, width, i, string1))
            U = ((J == None)or(J == []))
            if((J == None)or(J == [])):
                idx1 = i
                if((idx1 == height-1)and(string2 == 'min')):
                    return 120
                elif((idx1 == 0)and(string2 == 'max')):
                    return 120
            else:
                return i


    # the callback function of the subscription
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame...')
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # receive data of the image 
        height, width, _ = img_cv.shape
        i_bot = height - 5
        i_mid = height - 50
        width_n = width - 250

        # resize the image so that we can only focus on the right bottom of the image
        out_pnt = np.float32([[0, 0],
                            [0, height-1],
                            [width-1, height-1],
                            [width-1, 0]])
        inp_pnt = np.float32([[0, i_mid],   # A_IMG
                        [0, i_bot],         # B_IMG
                        [width-1, i_bot],   # C_IMG
                        [width-1, i_mid]])  # D_IMG
        
        M = cv2.getPerspectiveTransform(inp_pnt, out_pnt)
        img = cv2.warpPerspective(img_cv, M, (width, height), flags = cv2.INTER_LINEAR)
        img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        Wminhigt = self.DeterHeight1(img, height, width, 0, height, "White", "min")
        Gminhigt = self.DeterHeight1(img, height, width, 0, height, "Green", "min")
        Wmaxhigt = self.DeterHeight1(img, height, width, height-1, -1, "White", "max")
        Gmaxhigt = self.DeterHeight1(img, height, width, height-1, -1, "Green", "max")
        print("Wminhigt = ", Wminhigt, "\t\tGminhigt = ", Gminhigt)
        print("Wmaxhigt = ", Wmaxhigt, "\t\tGmaxhigt = ", Gmaxhigt)
        
        # find the action path
        Line = self.LineForm(img, height, width, "White", "Green", Wminhigt, Wmaxhigt, Gminhigt, Gmaxhigt)

        self.LoC = self.LineorCurve(Line, Gmaxhigt, Gminhigt, Wmaxhigt, Wminhigt)
        print('LoC = ', self.LoC)

        slope_n = self.LineSlope(Line, self.LoC)
        self.slope = slope_n - self.slope
        print('slope = ', self.slope)

        # publish the cv image back to publisher

        ''''''
        # to test if the Project will find the correct line
        # if((Line != 0)and(Line != 1)):
        for i in range(len(Line)):
            Pnt = Line[i]
            cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 3)
        ''''''

        # to show the cv image
        # img_hsv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HSV)
        # cv2.imshow("IMG_HSV", img_hsv)
        # cv2.imshow("IMG_CV", img_cv)
        cv2.imshow("IMG", img)
        cv2.waitKey(1)


    def timer_callback(self):
        if(self.LoC == 2):
        #'LoC=2' means that the robot didn't find a white line
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("STOP!")
        elif(abs(self.slope) <= 0.03):
        #call the class variable
        #since the publisher create the msg itself
        #you are not allow to use the msg in publisher
        #[ self.i < 20): ]
            msg = Twist()
            msg.linear.x = 0.02
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            print("Straight Driving!")
        elif(abs(self.slope) > 0.03):
            msg = Twist()
            msg.linear.x = 0.02
            msg.angular.z = self.slope
            self.publisher_.publish(msg)
            print("Turning!")
        else: 
            msg = Twist()
            msg.linear.x = 0.005
            msg.angular.z = 0.0
            self.publisher_.publish(msg)
            print("Straight Driving!")


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
