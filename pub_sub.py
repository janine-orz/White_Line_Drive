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
        self.i = 0
        self.img = 0
        self.L = []
        self.LoC = 0
        self.slope = 0
        self.slope_bot = 0
        self.slope_line = 0


    def ColRec(self, img, i, j):
        color = "Undefined"
        pixel_center = []
        # print('i = ', type(i), i, ';j = ', type(j), j)
        # i = int(i)
        # j = int(j)
        pixel_center = img[i, j]
        b_val = pixel_center[0]
        g_val = pixel_center[1]
        r_val = pixel_center[2]

        if b_val > 170 and g_val > 170 and r_val > 170:
            color = "White"
        elif b_val < 60 and g_val < 80 and r_val < 40:
            color = "Black"
        else:
            # print("...............Searching in HSV_image...................")
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            pixel_center = hsv_img[i, j]
            h_val = pixel_center[0]
            s_val = pixel_center[1]
            v_val = pixel_center[2]
            # print(h_val, s_val, v_val)
            # if(s_val < 50 and v_val < 50):
            #     # print("s_val < 50 and v_val < 65")
            #     color = "White"
            if h_val < 5:
                color = "Red"
            elif h_val < 22:
                color = "Orange"
            elif h_val < 33:
                color = "Yellow"
            elif h_val < 70:
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
            for j in range(width - 161, width, 1):
                color = self.ColRec(img, i, j)
                # print('[', i, ',', j, '] = ', color)
                if color == string:
                #sucess to find the white color
                    temp = [i, j]
                    Col.append(temp)
            return Col
        elif(string == 'Green'):
            for j in range(0, width - 159, 1):
                color = self.ColRec(img, i, j)
                # print('[', i, ',', j, '] = ', color)
                if color == string:
                #sucess to find the white color
                    temp = [i, j]
                    Col.append(temp)
            return Col


    def LineForm(self, img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
        # print('i_min, i_max =', i_min, i_max)
        L = []
        L1 = []
        L2 = []
        Line1_temp = []
        Line2_temp = []
        i_min = max(Wi_min, Gi_min)
        # print('Wi_min = ', Wi_min, '\t\tGi_min = ', Gi_min)
        i_max = min(Wi_max, Gi_max)
        # print('Wi_max = ', Wi_max, '\t\tGi_max = ', Gi_max)
        if(i_min == Gi_min): 
            # print("i_min == Gi_min")# The lower bound of Whiteline is smaller then the Greenline
            for h in range (Wi_min, Gi_min):
                Line1_temp = self.ColDet(img, height, width, h, string1)
                # print('color = ', string1, '\t\tLine1_temp = ', Line1_temp)
                L1 = self.Lineform(Line1_temp, L1, h, h, string1, img, height, width)
                L2 = L1[0] - 200
                # print('in range (0, Gi_min)\tL1 = ', L1, '\t\tL2 = ', L2)
                x_temp = self.MidCal(L1[0], L2)
                temp = [x_temp, h]
                L.append(temp)
        elif(i_min == Wi_min):
            # print("i_min == Wi_min")
            for h in range (Gi_min, Wi_min):
                Line2_temp = self.ColDet(img, height, width, h, string2)
                # print('color = ', string2, '\t\tLine2_temp = ', Line2_temp)
                L2 = self.Lineform(Line2_temp, L2, h, h, string2, img, height, width)
                L1 = L2[0] + 200
                # print('in range (0, Wi_min)\tL1 = ', L1, '\t\tL2 = ', L2)
                x_temp = self.MidCal(L1, L2[0])
                temp = [x_temp, h]
                L.append(temp)
        for i in range(i_min, i_max):
            # print('i = ', i, ', min = ', i_min, ', max = ', i_max)
            # print(string1, "\n")
            Line1_temp = self.ColDet(img, height, width, i, string1)
            # print(string2, "\n")
            Line2_temp = self.ColDet(img, height, width, i, string2)
            # print('Line1_temp = ', Line1_temp, '\tLine2_temp = ', Line2_temp)
            L1 = self.Lineform(Line1_temp, L1, i, i, string1, img, height, width)
            L2 = self.Lineform(Line2_temp, L2, i, i, string2, img, height, width)
            # print('\tL1 = ', L1, '\t\tL2 = ', L2)
            x_temp = self.MidCal(L1[0], L2[0])
            temp = [x_temp, i]
            L.append(temp)
        if(i_max == Gi_max): # The Greenline is shorter then the Whiteline
            for k in range (Gi_max, Wi_max):
                Line1_temp = self.ColDet(img, height, width, k, string1)
                # print('color = ', string1, '\t\tLine1_temp = ', Line1_temp)
                L1 = self.Lineform(Line1_temp, L1, k, k, string1, img, height, width)
                L2 = L1[0] - 200
                # print('in range (Gi_max, height)\twhiteline = ', L1, '\t\tgreenline = ', L2)
                x_temp = self.MidCal(L1[0], L2)
                temp = [x_temp, k]
                L.append(temp)
        elif(i_max == Wi_max):
            for k in range (Wi_max, Gi_max):
                Line2_temp = self.ColDet(img, height, width, k, string2)
                # print('color = ', string2, '\t\tLine2_temp = ', Line2_temp)
                L2 = self.Lineform(Line2_temp, L2, k, k, string2, img, height, width)
                L1 = L2[0] + 300
                # print('in range (Wi_max, height)\twhiteline = ', L1, '\t\tgreenline = ', L2)
                x_temp = self.MidCal(L1, L2[0])
                temp = [x_temp, k]
                L.append(temp)
        return L


    def Lineform(self, Line_temp, L, i, j, string, img, height, width):
        # print('i = ', i, ', string = ', string)
        LL = []
        LL.append(L)
        if((Line_temp == [])or(Line_temp == None)):
            # print('i = ', i, 'i-1 = ', i-1, ', string = ', string)
            # Line_temp = ColDet(img, height, width, i-1, string)
            # temp = Lineform(Line_temp, i-1, j, string, img, height, width)
            # print("\t", temp)
            # print("len L = ", len(LL), "L = ", LL)
            if len(LL) < 2:
                pos = L
            else:
                pos = L[-1]
            # print("\tpos", pos)
            num3 = pos[0]
            # print("\tnum3", num3)
            temp = [num3, j]
            return temp
        elif((Line_temp != None)or(Line_temp != [])):
            l = len(Line_temp)
            a = Line_temp[0]
            num1 = a[1]
            b = Line_temp[l-1]
            num2 = b[1]
            # we will choose the point on the most right
            if(num2 != None) and (string == 'Green'):
                temp = [num2, j]
                # print('\tgreen\ttemp = ', temp)
            elif(num1 != None) and (string == 'White'):
                temp = [num1, j]
                # print('\t', string, '\ttemp = ', temp)
            return temp


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


    # def ActionPath(self, img, height, width, minhigt, maxhigt, Rstring, Lstring):
    #     Line = []
    #     WLine = self.LineForm(img, height, width, Rstring, minhigt, maxhigt)
    #     # print('WLine = ', WLine)
    #     # WLine = checkagain(img, WLine, height, width, Rstring)
    #     GLine = self.LineForm(img, height, width, Lstring, minhigt, maxhigt)
    #     # print('GLine = ', GLine)
    #     # GLine = checkagain(img, GLine, height, width, Lstring)
    #     # start = time.time()
    #     if((WLine != [])and(GLine != [])):
    #         m = min(len(WLine), len(GLine))
    #         for i in range(m):
    #             Pnt1 = WLine[i]
    #             Pnt2 = GLine[i]
    #             x = self.MidCal(Pnt1[0], Pnt2[0])
    #             y = Pnt1[1]
    #             # print(x, y)
    #             Pnt0 = [x, y]
    #             Line.append(Pnt0)
    #         return Line
    #     elif(WLine == []):
    #         LCol = 0
    #         return LCol
    #     elif(GLine == []):
    #         LCol = 1
    #         return LCol


    # This function is used to check if the line is straight or not
    # if it is straight then LoC = 0
    # if it is a curve then Loc = 1
    def LineorCurve(self, Line):
        LoC = 0
        if(Line != []):
            Pnt1 = Line[0]
            Pnt2 = Line[len(Line) - 1]
            slope = (Pnt2[0] - Pnt1[0])/(Pnt2[1] - Pnt1[1])
            print(Pnt1, '\t', Pnt2)
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
        else:
            LoC = 2
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

        idx2 = (len(Line))/3
        idx2 = 2 * idx2
        idx2 = int(idx2)

        if(LoC == 0):
        # if the line is a straight line
        # then we need to find three different pairs of points
        # and their corresponding slope
            slope = self.CalSlope(Line, l, idx2)
            return slope

        elif(LoC == 1):
        # if the line is a curve
        # then we only need to calculate the slope of the pairs at bottom
            slope = self.CalSlope(Line, l, idx2)
            # the slope that the robot uses to test, how much should the robot turn
            return slope
        elif(LoC == 2):
        # if the robot cannot find a line
        # then it will give the value of LoC
        # so that the robot will not give back an Error
            return LoC


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
            # print('J = ', J)
            U = ((J == None)or(J == []))
            # print(string, 'at', i, 'cd.ColDet(img,', height, ',', width, ',', i, ',', string, '):', U)
            if((J == None)or(J == [])):
                idx1 = i
                # print('\t i = ', i, '\tidx1 == height-1:', idx1 == height-1)
                if((idx1 == height-1)and(string2 == 'min')):
                    return i
                elif((idx1 == 0)and(string2 == 'max')):
                    return i
            else:
                return i


    # the callback function of the subscription
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame...')
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # receive data of the image 
        height, width, _ = img_cv.shape
        print('height = ', height)
        print('width = ', width)
        i_bot = height - 5
        i_mid = height - 60
        width_n = width - 250

        start = time.time()

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

        # find the action path
        # WLine = self.LineForm(img, height, width, "White", minhigt, maxhigt)
        # GLine = self.LineForm(img, height, width, "Green" or "Yellow", minhigt, maxhigt)
        # Line = self.ActionPath(img, height, width, minhigt, maxhigt, "White", "Green" or "Yellow")
        Line = self.LineForm(img, height, width, "White", "Green", Wminhigt, Wmaxhigt, Gminhigt, Gmaxhigt)
        

        self.LoC = self.LineorCurve(Line)
        print('LoC = ', self.LoC)
        slope_n = self.LineSlope(Line, self.LoC)
        self.slope = slope_n - self.slope
        print('slope = ', self.slope, 'slope_n = ', slope_n)

        # publish the cv image back to publisher

        ''''''
        # to test if the Project will find the correct line
        # if((Line != 0)and(Line != 1)):
        for i in range(len(Line)):
            Pnt = Line[i]
            # Pnt1 = WLine[i]
            # Pnt2 = GLine[i]
            # print('i = ', i, '; Pos = ', Pnt2)
            cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 3)
            # cv2.circle(img, (Pnt1[0], Pnt1[1]), 1, (125, 255, 0), 3)
            # cv2.circle(img, (Pnt2[0], Pnt2[1]), 1, (0, 175, 175), 3)
        ''''''
        
        end = time.time()
        print('\t\t\t', end-start)

        # to show the cv image
        # img_hsv = cv2.cvtColor(img_cv, cv2.COLOR_BGR2HSV)
        # cv2.imshow("IMG_HSV", img_hsv)
        # cv2.imshow("IMG_CV", img_cv)
        cv2.imshow("IMG", img)
        cv2.waitKey(1)


    def timer_callback(self):
        if(abs(self.slope) <= 0.005):
        #call the class variable
        #since the publisher create the msg itself
        #you are not allow to use the msg in publisher
        #[ self.i < 20): ]
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("Straight Driving!")
        elif(abs(self.slope) > 0.005):
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00 # self.slope
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
