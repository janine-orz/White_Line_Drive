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
    - it goes well if the road divides
TO DO:
    - put different BGR-value into the ColDet if no white line then change BGR-value
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
        self.slope = 0.00
        self.slope_do = 0.00


    # We use this function to find out the color at each pixel_center
    def ColRec(self, img, i, j, BGR):
        # i: height     j: width    BGR: bgr value for White color(since the light will change)
        # initialize the color and pixel_center
        color = "Undefined"

        # the bgr value of the image will be used to detect White and Black color
        pixel_center = np.array(img[i, j])
        b_val = pixel_center[0]
        g_val = pixel_center[1]
        r_val = pixel_center[2]

        if b_val > BGR[0] and g_val > BGR[1] and r_val > BGR[2]:
            color = "White"
        elif b_val < 90 and g_val < 110 and r_val < 90:
            color = "Black"
        else:
            # the hsv value of the image will be used to detect the other colors
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            pixel_center = np.array(hsv_img[i, j])
            h_val = pixel_center[0]
            s_val = pixel_center[1]
            v_val = pixel_center[2]
            
            # But the s_val and v_val should be set which means the color is clear enough
            if s_val >= 100 and v_val >= 100:
                if h_val < 5:
                    color = "Red"
                elif h_val < 10:
                    color = "Orange"
                elif h_val < 25:
                    color = "Yellow"
                elif h_val < 80:
                    color = "Green"
                elif h_val < 131:
                    color = "Blue"
                elif h_val < 167:
                    color = "Violet"
                else:
                    color = "Red"
        return color


    # We use this function to find out all white and green points of the line
    def ColDet(self, img, height, width, i, BGR, string, Col):
        # initialize color and color_a
        color = "Undefined"
        color_a = "Undefined"

        # to check the white line
        # "White": it should the line on the right side
        if(string == 'White'):
            temp = -100
            white_wid = []
            
            # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            for j in range(width - 120, width - 1, 1):
                # print("============= i : ", i, "=============")
                color = self.ColRec(img, i, j, BGR)
                if color == string:
                #sucess to find the white color
                    white_wid.append(j)
            
            Col = self.White_Wid(white_wid, Col, i)
            
        # to check the green line
        # "Green": it should the line on the left side
        elif(string == 'Green'):
            temp = 0
            count = 0
            count_a = 0
            # limit to find white color on the left side
            for j in range(0, 140, 1):
                color = self.ColRec(img, i, j, BGR)
                if color == string:
                    
                    if (temp == 0):
                        temp = j
                    elif (j > temp) and (abs(temp-j) <= 5) and (color_a == string): 
                        temp = j

                    Col[0] = temp
                color_a = color
            Col[1] = i
            
        return Col


    def White_Wid(self, white_wid, Col, i):
        if(white_wid != []):
            temp = white_wid[0]
            for j in range(1, len(white_wid), 1):
                if (j == 1):
                    temp = min(white_wid[j-1], white_wid[j])
                elif (abs(white_wid[j-1] - white_wid[j]) >= 8) and (abs(Col[0] - temp) < 10):
                    temp = min(temp, white_wid[j])
            Col[0] = temp
        Col[1] = i
        return Col
                

    def LineForm(self, img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
        # initialize L, L1, L2, WL, GL
        # WL, GL: save the last position so that we can reuse the width later
        L1 = np.zeros((2), dtype = 'int')
        L2 = np.zeros((2), dtype = 'int')
        # only the part between i_min and i_max contain both white and green line
        i_min = max(Wi_min, Gi_min)
        i_max = min(Wi_max, Gi_max)
        # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        L = np.array([[-100, i_min]], dtype = 'int')
        WL = L
        GL = L
        Ldiff = []
        diff_min = 105
        diff_max = 135


        # the part that contain both white and green lines
        for i in range(i_min, i_max):
            L1[0] = -100
            L2[0] = -100

            BGR = [170, 170, 170]

            L1 = self.ColDet(img, height, width, i, BGR, string1, L1)
            L1 = self.IfLineBreak(WL, L1, i, i, string1, img, height, width, -1)
            # print("L1 = ", L1, '\tL1 bgr_value: ', img[L1[1], L1[0]])
            WL = np.vstack((WL, L1))  
            
            L2 = self.ColDet(img, height, width, i, BGR, string2, L2)
            L2 = self.IfLineBreak(GL, L2, i, i, string2, img, height, width, -1)
            GL = np.vstack((GL, L2))

            diff_temp = abs(L1[0] - L2[0])
            Ldiff.append(diff_temp)

            x_temp = self.MidCal(L1[0], L2[0])
            temp = [x_temp, i]
            
            if(i == i_min):
                WL = np.delete(WL, 0, 0)
                GL = np.delete(GL, 0, 0)
                L = np.delete(L, 0, 0)
                diff_min = L1[0] - x_temp
            elif(i == i_max-1):
                diff_max = L1[0] - x_temp
            L = np.vstack((L, temp))

        BGR = [150, 140, 125] # [140, 140, 120]

        #####################################################
        #     IF    there is only one line :                #
        #   THEN    follow the line ;                       #
        #           form the line with the edge 319 or 0 ;  #
        # RETURN    L ;                                     #
        #####################################################

        # if (Ldiff != []) and (Gi_max > 160 and Wi_max > 160):
        #     Ldtop = max(Ldiff[0], Ldiff[1], Ldiff[2])
        #     Ldlow = min(Ldiff[-1], Ldiff[-2], Ldiff[-3])
        #     if Ldtop > Ldlow:
        #         L = self.Follow_White(img, height, width, BGR, L, L1, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, string1, diff_min)
        #         return L
        
        L = self.UpperLine(img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, Gi_min, Wi_min, diff_min)
        L = self.BottomLine(img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, i_max, Gi_max, Wi_max, diff_max)
        return L


    def IfLineBreak(self, L, Li, i, j, string, img, height, width, rr):
        if((Li[0] == -100)):

            if(L[0][0] == -100):
                if(string == 'White'):
                    L = np.array([-100, 0], dtype = 'int')
                    L[0] = -100
                    BGR = [140, 140, 120]
                    L = self.ColDet(img, height, width, i, BGR, string, L)
                elif(string == 'Green'):
                    L = np.array([0, j])
                return L

            # for the upper part of the line: we pick the first point of L
            if(rr == 0):
                while (Li[0] == -100):
                    i = i-1
                    Li = L[0]
                #      0 1 2 3 4 5 ...
                # L =  ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ 
                # Li = ↑
            
            # for the bottom part of the line: we pick the last point of L
            elif(rr == -1):
                while (Li[0] == -100):
                    i = i-1
                    Li = L[-1]
                #                    ...          -3-2-1
                # L =  ▅ ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ 
                # Li =                                 ↑
            
            pos = Li[0]
            temp = [pos, j]
            return temp
        else:
            return Li


    def Follow_White(self, img, height, width, BGR, L, L1, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, string1, diff):
        print("=============  Follow_White()  =============")
        i_min = min(Wi_min, Gi_min)
        i_max = max(Wi_max, Gi_max)
        L = np.array([[-100, i_min]], dtype = 'int')
        WL = L
        GL = L

        # The lower bound of Whiteline is smaller then the Greenline
        for h in range (i_min, i_max, 1):
            L1 = self.ColDet(img, height, width, h, BGR, string1, L1)
            L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
            WL = np.vstack((L1, WL))
            L2 = L1[0] - (2*diff)
            if (L2 <= 0):
                x_temp = self.MidCal(L1[0], L2)
            else:
                x_temp = L1[0] - diff
            temp = [x_temp, h]
            if(h == i_min):
                WL = np.delete(WL, 0, 0)
                GL = np.delete(GL, 0, 0)
                L = np.delete(L, 0, 0)
            L = np.vstack((L, temp))
        return L


    def UpperLine(self, img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, Gi_min, Wi_min, diff):
        print("===========  UpperLine()  ===========")
        # The lower bound of Greenline is smaller then the Whiteline
        if(i_min == Gi_min) and (Gi_min != 0): 
            for h in range (Gi_min, Wi_min, -1):
                L1 = self.ColDet(img, height, width, h, BGR, string1, L1)
                L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
                WL = np.vstack((L1, WL))
                x_temp = L1[0] - diff
                temp = [x_temp, h]
                # insert at the beginning of the list
                # temp
                #  ↓
                #  ❑ ▅ ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ 
                L = np.vstack((temp, L))
        
        # The lower bound of Whiteline is smaller then the Greenline
        elif(i_min == Wi_min) and (Wi_min != 0):
            for h in range (Wi_min, Gi_min, -1):
                L2 = self.ColDet(img, height, width, h, BGR, string2, L2)
                L2 = self.IfLineBreak(GL, L2, h, h, string2, img, height, width, 0)
                GL = np.vstack((L2, GL))
                x_temp = L2[0] + diff
                temp = [x_temp, h]
                L = np.vstack((temp, L))

        return L


    def BottomLine(self, img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, i_max, Gi_max, Wi_max, diff):
        print("===========  BottomLine()  ===========")
        # The lower bound of Whiteline is smaller then the Greenline
        if(i_max == Gi_max) and (Gi_max != 239): 
            for h in range (Gi_max, Wi_max):
                L1 = self.ColDet(img, height, width, h, BGR, string1, L1)
                L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, -1)
                WL = np.vstack((WL, L1))
                L2 = L1[0] - (2*diff)
                if (L2 <= 0):
                    x_temp = self.MidCal(L1[0], L2)
                else:
                    x_temp = L1[0] - diff
                temp = [x_temp, h]
                if(h == i_min):
                    WL = np.delete(WL, 0, 0)
                    GL = np.delete(GL, 0, 0)
                    L = np.delete(L, 0, 0)
                # insert at the end of the list
                #                            temp
                #                             ↓
                # ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ❑ 
                L = np.vstack((L, temp))

        elif(i_max == Wi_max) and (Wi_max != 239):
            for h in range (Wi_max, Gi_max):
                L2 = self.ColDet(img, height, width, h, BGR, string2, L2)
                L2 = self.IfLineBreak(GL, L2, h, h, string2, img, height, width, -1)
                GL = np.vstack((GL, L2))
                L1 = L2[0] + (2*diff)
                if (L1 >= 319):
                    x_temp = self.MidCal(L1, L2[0])
                else:
                    x_temp = L2[0] + diff
                temp = [x_temp, h]
                if(h == i_min):
                    WL = np.delete(WL, 0, 0)
                    GL = np.delete(GL, 0, 0)
                    L = np.delete(L, 0, 0)
                L = np.vstack((L, temp))
        
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
        elif(b >= a):
            num2 = a + num1
            num2 = int(num2)
            # Since the Point should be a integer
            return num2


    # This function is used to check if the line is straight or not
    # if it is straight then LoC = 0
    # if it is a curve then Loc = 1
    def LineorCurve(self, Line):
        slope = 0.0
        LoC = 0

        idx0 = len(Line)-1
        pos0 = Line[idx0]

        idx1 = 0
        pos1 = Line[idx1]
        while(pos1[0] == -100):
            idx1 = idx1 + 1
            pos1 = Line[idx1]
        
        slope1 = self.CalSlope(Line, idx0, idx1)
        const1_0 = pos0[0] - slope1*pos0[1]
        const1_1 = pos1[0] - slope1*pos1[1]
        const1 = (const1_0 + const1_1) / 2

        idx2 = (int)(len(Line)/5)
        pos2 = Line[idx2]

        idx3 = 4 * idx2
        pos3 = Line[idx3]
        
        slope2 = self.CalSlope(Line, idx3, idx2)
        const2_0 = pos2[0] - slope2*pos2[1]
        const2_1 = pos3[0] - slope2*pos3[1]
        const2 = (const2_0 + const2_1) / 2
        
        idx4 = (int)(len(Line)/2)
        pos4 = Line[idx4]

        pos5_1 = pos4[1]
        pos5_0 = (int)(slope1*pos4[1] + const1)

        pos6_1 = pos4[1]
        pos6_0 = (int)(slope2*pos4[1] + const2)

        if (abs(pos5_0-pos4[0]) < 5) and (abs(pos6_0-pos4[0]) < 5):
            LoC = 0
        elif (abs(pos5_0 - pos4[0]) >= 5) or (abs(pos6_0 - pos4[0]) >= 5):
            LoC = 1
        else:
            LoC = 2
        
        return LoC


    # This function is a part of the function LineorCurve
    # we check if the point Pnt3 stay on the stragith line
    # by calculating a Pnt4 through the slope and const
    def LoCCal(self, Pnt3, slope, const):
        y1 = 0
        Pnt4 = Pnt3
        y1 = (slope * Pnt4[1]) + const
        y1 = int(y1)
        diff = Pnt4[0] - y1
        if(abs(diff) < 2):
            LoC = 0
        else:
            LoC = 1
        return LoC


    # This function is used to calculate slope of a line
    def LineSlope(self, Line, LoC):
        slope = 0.0
        idx2 = 0
        l = len(Line) - 1

        idx2 = (len(Line))/5
        idx2 = 1 * idx2
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
        slope = self.GetAngl(Pnt1, Pnt2)
        while slope == -100:
            idx2 = idx2 + 1
            Pnt2 = Line[idx2]
            slope = self.GetAngl(Pnt1, Pnt2)
        return slope


    # use this function to calculate the opposite edge and the adjacent edge
    # so that we can calculate the angle by using tanh
    def GetAngl(self, Pnt1, Pnt2):
        angle = 0.0
        opp = (Pnt2[0] - Pnt1[0])
        adj = (Pnt2[1] - Pnt1[1])
        if adj == 0:
            return -100
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
            J = np.zeros((2), dtype = 'int')
            J[0] = -100
            BGR = [170, 170, 170] # [170, 170, 160]
            J = (self.ColDet(img, height, width, i, BGR, string1, J))
            U = (J[0] == -100)
            if U == True:
                idx1 = i
                if((idx1 == height-1)and(string2 == 'min')):
                    return 0
                elif((idx1 == 0)and(string2 == 'max')):
                    return i
            elif U == False:
                return i

    
    def PrintLine(self, Line, img):
        Pnt1 = np.array([0, 0])
        Pnt2 = np.array([0, 0])
        for i in range (len(Line)-1):
            Pnt1 = Line[i]
            # Pnt2 = Line[i+1]
            # # print("Pnt1 = ", Pnt1, "\tPnt2 = ", Pnt2)
            # if (abs(Pnt1[0]-Pnt2[0]) > 5 ) and (Pnt1[0] != 0):
            #     Line[i+1][0] = Pnt1[0]
            cv2.circle(img, (Pnt1[0], Pnt1[1]), 1, (255, 0, 0), 3)
            return Line


    def DrawTangent(self, Line, img, slope):
        l = len(Line)
        idx1 = (len(Line))/5
        idx1 = 2 * idx1
        idx1 = int(idx1)

        Pnt0 = Line[idx1]
        cv2.circle(img, (Pnt0[0], Pnt0[1]), 2, (255, 0, 255), 5)
        Pnt1 = Line[l-1]
        cv2.circle(img, (Pnt1[0], Pnt1[1]), 2, (255, 0, 255), 5)
        const = Pnt1[0] - (Pnt1[1] * slope)
        for i in range(1, (l - idx1), 1):
            Pnt = Line[l-i]
            # print(Pnt)
            y = Pnt[1]
            x = Pnt[1]*slope + const
            x = int(x)
            # print('[', x, ',', y, ']')
            cv2.circle(img, (x, y), 1, (0, 175, 175), 2)



    # the callback function of the subscription
    def listener_callback(self, data):
        self.get_logger().info('Receiving video frame...')
        img_cv = self.bridge.compressed_imgmsg_to_cv2(data, desired_encoding = 'passthrough')

        # receive data of the image 
        height, width, _ = img_cv.shape
        i_bot = height - 2
        i_mid = height - 30

        # resize the image so that we can only focus on the right bottom of the image
        out_pnt = np.float32([[0,        0       ],
                              [0,        height-1],
                              [width-1,  height-1],
                              [width-1,  0       ]])
        inp_pnt = np.float32([[15,       i_mid],    # A
                              [0,        i_bot],    # B
                              [width-5,  i_bot],    # C
                              [width-15, i_mid]])   # D

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

        Line = self.PrintLine(Line, img)

        self.LoC = self.LineorCurve(Line)
        print("--------------- LoC = ", self.LoC, " --------------")

        slope_n = self.LineSlope(Line, self.LoC)
        self.slope_do = slope_n - self.slope
        self.slope = slope_n
        self.DrawTangent(Line, img, self.slope)
        print("------------- slope_n = ", slope_n, "slope_do = ", self.slope_do, " -------------")
        

        # publish the cv image back to publisher

        ''''''
        # to test if the Project will find the correct line
        # if((Line != 0)and(Line != 1)):
        
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
            msg.angular.z = self.slope
            self.publisher_.publish(msg)
            print("STOP!")
        elif(abs(self.slope) <= 0.035):
        #call the class variable
        #since the publisher create the msg itself
        #you are not allow to use the msg in publisher
        #[ self.i < 20): ]
            msg = Twist()
            msg.linear.x = 0.015
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("Straight Driving!")
        elif(abs(self.slope) > 0.035):
            msg = Twist()
            msg.linear.x = 0.015
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
