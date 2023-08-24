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

    # We use this function to find out the color at each pixel_center
    def ColRec(self, img, i, j, val):
        # i: height     j: width    val: value of bgr for White color(since the light will change)

        # initialize the color and pixel_center
        color = "Undefined"
        pixel_center = []

        # the bgr value of the image will be used to detect White and Black color
        pixel_center = img[i, j]
        b_val = pixel_center[0]
        g_val = pixel_center[1]
        r_val = pixel_center[2]
        # print("\tB = ", b_val, "\tG = ", g_val, "\tR = ", r_val)

        if b_val > val and g_val > val and r_val > val:
            color = "White"
        elif b_val < 90 and g_val < 110 and r_val < 90:
            color = "Black"
        else:
            # the hsv value of the image will be used to detect the other colors
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            pixel_center = hsv_img[i, j]
            h_val = pixel_center[0]
            s_val = pixel_center[1]
            v_val = pixel_center[2]
            # print("[", j, ", ", i, "]H = ", h_val, "S = ", s_val, "V = ", v_val)
            
            # But the s_val and v_val should be set which means the color is clear enough
            if s_val >= 100 and v_val >= 100:
                if h_val < 5:
                    color = "Red"
                elif h_val < 10:
                    color = "Orange"
                elif h_val < 25:
                    color = "Yellow"
                elif h_val < 75:
                    color = "Green"
                elif h_val < 131:
                    color = "Blue"
                elif h_val < 167:
                    color = "Violet"
                else:
                    color = "Red"
        return color

    # We use this function to find out all white and green points of the line
    def ColDet(self, img, height, width, i, string):
        Col = []

        # to check the white line
        # "White": it should the line on the right side
        if(string == 'White'):
            # initialize temp
            temp = 0

            # limit to find white color on the right side
            for j in range(width - 150, width, 1):
                color = self.ColRec(img, i, j)
                # sucess to find the white color
                if color == string:

                    # print("color = ", color, "\tposition = [", j, ",", i, "]")
                    # case 1 : at the beginning
                    if (temp == 0):
                        temp = j
                    # case 2 : [ j > temp ]:          only take the max width
                    #          [ abs(temp-j) ]:       if width* is not far away from width
                    #          [ color_a == string ]: the pixel on the left hand side is also white
                    elif (j > temp) and (abs(temp-j) <= 15) and (color_a == string): 
                        temp = j
                    # only output the value of white point if (temp != 0)
                    if(temp != 0):
                        temp_pos = [temp, i]
                        Col = temp_pos
                    # if(i == ??):
                    #     print("\t\t", Col)
                    #     if(Col[0] == ??):
                    #         print("color_a = ", color_a)
                
                # save the color of the pixel on the left hand side                    
                color_a = color
        
        # to check the green line
        # "Green": it should the line on the left side
        elif(string == 'Green'):
            # initialize temp
            temp = 0

            # limit to find white color on the left side
            for j in range(0, width - 170, 1):
                color = self.ColRec(img, i, j)
                if color == string:
                    # print("color = ", color, "\tposition = [", j, ",", i, "]", "\t bgr = ", img[i, j])
                    if (temp == 0):
                        temp = j
                    elif (j > temp) and (abs(temp-j) <= 15) and (color_a == string): 
                        temp = j
                    if(temp != 0):
                        temp_pos = [temp, i]
                        Col = temp_pos
                color_a = color
        return Col

    def LineForm(self, img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
        # initialize L, L1, L2, WL, GL
        # WL, GL: save the last position so that we can reuse the width later
        L = []
        L1 = []
        L2 = []
        WL = []
        GL = []

        # only the part between i_min and i_max contain both white and green line
        i_min = max(Wi_min, Gi_min)
        i_max = min(Wi_max, Gi_max)
        # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # the part that contain both white and green lines
        for i in range(i_min, i_max):
            # print("==========white line==========")
            L1 = self.ColDet(img, height, width, i, 180, string1)
            L1 = self.IfLineBreak(WL, L1, i, i, string1, img, height, width, -1)
            WL.append(L1)
            # print("L1 = ", L1)
            
            # print("==========green line==========")
            L2 = self.ColDet(img, height, width, i, 180, string2)
            L2 = self.IfLineBreak(GL, L2, i, i, string2, img, height, width, -1)
            GL.append(L2)
            # print("L1 = ", L1, '\tL1 bgr_value: ', img[L1[1], L1[0]], "\nL2 = ", L2, '\tL2 hsv_value: ', hsv_img[L2[1], L2[0]])

            x_temp = self.MidCal(L1[0], L2[0])
            temp = [x_temp, i]
            
            L.append(temp)
        
        print("============ UpperLine() ============")
        L = UpperLine(img, height, width, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min)
        print("============ BottomLine() ============")
        L = BottomLine(img, height, width, L, WL, GL, string1, string2, i_max, Gi_max, Wi_max)
        # print("To fill the upper and bottom part of the line: ", end-start)

        return L


    def IfLineBreak(self, L, Li, i, j, string, img, height, width, rr):
        if((Li == [])or(Li == None)):
            # start = time.time()
            # print("L = ", L)
            if(L == []):
                L.append([0, i])
            
            # for the upper part of the line: we pick the first point of L
            if(rr == 0):
                while (Li == []):
                    i = i-1
                    Li = L[0]
                #      0 1 2 3 4 5 ...
                # L =  ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ 
                # Li = ↑
                if isinstance(Li, list):
                    pos = Li[0]
                    temp = [pos, j]
                # if Li only contains one position
                # then we can should give another case 
                # so as to avoid the error report which may break the programm
                elif isinstance(Li, int):
                    temp = [Li, j]
            # for the bottom part of the line: we pick the last point of L
            elif(rr == -1):
                while (Li == []):
                    i = i-1
                    Li = L[-1]
                #                    ...          -3-2-1
                # L =  ▅ ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ 
                # Li =                                 ↑
                if isinstance(Li, list):
                    pos = Li[0]
                    temp = [pos, j]
                elif isinstance(Li, int):
                    temp = [Li, j]
            return temp
        else:
            return Li


    def UpperLine(self, img, height, width, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min):
        # The lower bound of Greenline is smaller then the Whiteline
        if(i_min == Gi_min): 
            for h in range (Gi_min, Wi_min, -1):
                L1 = self.ColDet(img, height, width, h, 165, string1)
                L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
                WL.insert(0, L1)
                L2 = L1[0] - 220
                x_temp = self.MidCal(L1[0], L2)
                temp = [x_temp, h]
                # insert at the beginning of the list
                # ↓
                # ❑ ▅ ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ 
                L.insert(0, temp)
        
        # The lower bound of Whiteline is smaller then the Greenline
        elif(i_min == Wi_min):
            for h in range (Wi_min, Gi_min, -1):
                L2 = self.ColDet(img, height, width, h, 165, string2)
                L2 = self.IfLineBreak(GL, L2, h, h, string2, img, height, width, 0)
                GL.insert(0, L2)
                L1 = L2[0] + 220
                x_temp = self.MidCal(L1, L2[0])
                temp = [x_temp, h]
                L.insert(0, temp)
        return L


    def BottomLine(self, img, height, width, L, WL, GL, string1, string2, i_max, Gi_max, Wi_max):
        if(i_max == Gi_max): 
            # The lower bound of Whiteline is smaller then the Greenline
            for h in range (Gi_max, Wi_max):
                L1 = self.ColDet(img, height, width, h, 165, string1)
                L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, -1)
                WL.append(L1)
                L2 = 0
                x_temp = self.MidCal(L1[0], L2)
                temp = [x_temp, h]
                # insert at the end of the list
                #                             ↓
                # ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ❑ 
                L.append(temp)
        elif(i_max == Wi_max):
            for h in range (Wi_max, Gi_max):
                L2 = self.ColDet(img, height, width, h, 165, string2)
                L2 = self.IfLineBreak(GL, L2, h, h, string2, img, height, width, -1)
                GL.append(L2)
                L1 = 319
                x_temp = self.MidCal(L1, L2[0])
                temp = [x_temp, h]
                L.append(temp)
        return L


    # This function is used to calculate the middle point between two points
    def MidCal(self, a, b):
        # num1 is half of the value between a and b
        num1 = (a - b)/2
        # then the middle point is (min(a,b) + num1)
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
            J = (self.ColDet(img, height, width, i, 165, string1))
            U = ((J == None)or(J == []))
            if((J == None)or(J == [])):
                idx1 = i
                if((idx1 == height-1)and(string2 == 'min')):
                    return 0
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

        # find the min and max height of white and green lines
        Wminhigt = self.DeterHeight1(img, height, width, 0, height, "White", "min")
        Gminhigt = self.DeterHeight1(img, height, width, 0, height, "Green", "min")
        Wmaxhigt = self.DeterHeight1(img, height, width, height-1, -1, "White", "max")
        Gmaxhigt = self.DeterHeight1(img, height, width, height-1, -1, "Green", "max")
        if(Wminhigt == Wmaxhigt):
            Wminhigt = Gminhigt
            Wmaxhigt = Gminhigt
        elif(Gminhigt == Gmaxhigt):
            Gminhigt = Wminhigt
            Gmaxhigt = Wminhigt
        print("Wminhigt = ", Wminhigt, "\t\tGminhigt = ", Gminhigt)
        print("Wmaxhigt = ", Wmaxhigt, "\t\tGmaxhigt = ", Gmaxhigt)
        
        start = time.time()

        # find the action path
        Line = self.LineForm(img, height, width, "White", "Green", Wminhigt, Wmaxhigt, Gminhigt, Gmaxhigt)

        end = time.time()
        print("time for LineForm():", end-start)

        self.LoC = self.LineorCurve(Line, Gmaxhigt, Gminhigt, Wmaxhigt, Wminhigt)
        slope_n = self.LineSlope(Line, self.LoC)
        self.slope = slope_n - self.slope
        print('LoC = ', self.LoC, '\tslope = ', self.slope)

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
            msg.angular.z = 0.00
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
