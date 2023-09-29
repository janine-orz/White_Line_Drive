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
    -   it goes well except if the road divide
TO DO:
    +   fill all the commands!!!
    +   do some notes and write down why

    17.30  2023-08-09 <- 1st Problem that I need to ask
'''

class FollowLine(Node):

    def __init__(self):
        super().__init__('follow_line')
        qos_policy = rclpy.qos.QoSProfile(
            reliability=rclpy.qos.ReliabilityPolicy.BEST_EFFORT,
            history=rclpy.qos.HistoryPolicy.KEEP_LAST,
            depth=1)
        self.subscription = self.create_subscription(CompressedImage, '/image_raw/compressed', self.listener_callback, qos_profile = qos_policy)
    
        self.bridge = CvBridge()
        # self.subscription = self.create_subscription(LaserScan, 'scan', self.listener_callback, qos_profile=qos_policy)

        self.subscription  # prevent unused variable warning

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 1)

        # every 0.5 seconds reload image from camera
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.img = 0
        self.LoC = 0
        self.slope = 0.00
        self.slope_do = 0.00


    # We use this function to find out the color at each pixel_center
    def ColRec(self, img, i, j, BGR):
        # i: height     j: width
        # BGR: bgr value for White color(since the light will change)
        # initialize the color and pixel_center
        color = "Undefined"

        # set pixel_center as the bgr_value from img at position [j, i]
        pixel_center = np.array(img[i, j])
        b_val = pixel_center[0]
        g_val = pixel_center[1]
        r_val = pixel_center[2]
        
        # the bgr value of the image will be used to detect White and Black color
        if b_val > BGR[0] and g_val > BGR[1] and r_val > BGR[2]:
            color = "White"
        elif b_val < 90 and g_val < 110 and r_val < 90:
            color = "Black"
        else:
            # the hsv value of the image will be used to detect the other colors
            hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
            # set pixel_center as the hsv_value from hsv_img at position [j, i]
            pixel_center = np.array(hsv_img[i, j])
            h_val = pixel_center[0]
            s_val = pixel_center[1]
            v_val = pixel_center[2]
            
            # But the s_val and v_val should be set
            # which means the color is clear enough
            # not too dark or not too bright
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
            # initialize scalar value(temp) and list(white_wid)
            temp = -100
            white_wid = []
            
            # check white color from left(width-170: middle of the graph) to right(right edge of the graph)
            for j in range(width - 170, width - 1, 1):
                color = self.ColRec(img, i, j, BGR)
                if color == string:
                    #if sucess to find the white color
                    # record every white point to list white_wid
                    white_wid.append(j)
            
            Col = self.White_Wid(img, white_wid, Col, i, j, string)
            
        # to check the green line
        # "Green": it should the line on the left side
        elif(string == 'Green'):
            # initialize temp
                #########################
                # change to temp = -100 #
                #########################
            temp = 0 

            # limit to find green color on the left side of the graph
            for j in range(0, 140, 1):
                color = self.ColRec(img, i, j, BGR)
                if color == string:
                    # for the first green point
                    if (temp == 0):
                        temp = j
                    # if there is another green point not far on the right side and the left point of it is also green
                    elif (j > temp) and (abs(temp-j) <= 5) and (color_a == string): 
                        temp = j

                    # update the width
                    Col[0] = temp
                # write down color of the last point
                color_a = color
            # update the height
            Col[1] = i
            
        return Col


    # We use this function to pick out the white line from all white points
    def White_Wid(self, img, white_wid, Col, i, j, string):
        # initialize list(Index) and scalar variable(idx)
        Index = []
        idx = 0

        # if we found too many white points, which means the brightness of the image higher
        if(len(white_wid) > 12):
            # raist the BGR value and 
            BGR = [220, 220, 220]
            color = self.ColRec(img, i, j, BGR)
            if color == string:
                #sucess to find the white color
                white_wid.append(j)
        
        # if we do find white points at hieght i
        if(white_wid != []):
            # if there is only one white point
            if(len(white_wid) == 1):
                Index = [1]
                # update the width as the first and the only one white point 
                temp = white_wid[0]
                Col[0] = temp
                Col[1] = i
                return Col
            
            for k in range(0, len(white_wid)-1, 1):
                # two white point is next to each other
                if(white_wid[k+1] - white_wid[k] < 2):
                    idx = idx + 1
                # two white point is far from each other
                elif(white_wid[k+1] - white_wid[k] >= 2):
                    # reset idx and write down Index
                    Index.append(idx+1)
                    idx = 0
                
                # till end of the white point
                if(k == len(white_wid) - 2):
                    Index.append(idx+1)
            
            # pick the max width white points
            num = max(Index)
            idx = Index.index(num)
            index = Index[idx-1]+1
            if(idx == 0):
                index = 1
            temp = white_wid[index]

            # update the width
            Col[0] = temp
        # update the height
        Col[1] = i

        return Col
                

    # We use this function to find the action line
    def LineForm(self, img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
        # initialize L, L1, L2, WL, GL
        # WL, GL: save the last position so that we can reuse the width later
        L1 = np.zeros((2), dtype = 'int')
        L2 = np.zeros((2), dtype = 'int')
        # only the part between i_min and i_max contain both white and green line
        i_min = max(Wi_min, Gi_min)
        i_max = min(Wi_max, Gi_max)
        L = np.array([[-100, i_min]], dtype = 'int') # the action line
        WL = L # the white line
        GL = L # the green line

        diff_min = 140
        diff_max = 150
        BGR = [190, 190, 190]
        # initialize Ldiff
        Ldiff = []
     
        # if "there is no white line" AND "there is one green line" AND "green line longer"
        if((Wi_min == Wi_max) and (Gi_min != Gi_max) and (Wi_max - Wi_min < Gi_max - Gi_min)):
            diff = 145
            # since there is no white line, then follow the green line
            L = self.Follow_White(img, height, width, BGR, L, L1, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, "Green", diff)
            return L
        # if "there is no green line" AND "there is one white line" AND "white line longer"
        elif((Gi_min == Gi_max) and (Wi_min != Wi_max) and (Gi_max - Gi_min < Wi_max - Wi_min)):
            diff = 145
            # since there is no white line, then follow the white line
            L = self.Follow_White(img, height, width, BGR, L, L1, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, "White", diff)
            return L
        

        # the part that contain both white and green lines
        for i in range(i_min, i_max):
            L1[0] = -100
            L2[0] = -100

            # to find white line
            L1 = self.ColDet(img, height, width, i, BGR, string1, L1)
            L1 = self.IfLineBreak(WL, L1, i, i, string1, img, height, width, -1)
            WL = np.vstack((WL, L1))  
            
            # to find green line
            L2 = self.ColDet(img, height, width, i, BGR, string2, L2)
            L2 = self.IfLineBreak(GL, L2, i, i, string2, img, height, width, -1)
            GL = np.vstack((GL, L2))

            # calculate the distance between white line and green line
            diff_temp = abs(L1[0] - L2[0])
            Ldiff.append(diff_temp)

            # to find the middle point of white line and green line
            x_temp = self.MidCal(L1[0], L2[0])
            temp = [x_temp, i]
            
            # write down (white line - action line) at the smallest(upper) height
            if(i == i_min):
                # delete the initial data
                WL = np.delete(WL, 0, 0) 
                GL = np.delete(GL, 0, 0)
                L = np.delete(L, 0, 0)
                diff_min = L1[0] - x_temp
            # write down (white line - action line) at the biggest(lower) height
            elif(i == i_max-1):
                diff_max = L1[0] - x_temp
            L = np.vstack((L, temp))
        
        # if the road divides then the robot should follow the white line
        if (Ldiff != []) and (Gi_max-Gi_min > 80 or Wi_max-Wi_min > 80):
            Ldtop = max(Ldiff[0], Ldiff[1])
            Ldlow = min(Ldiff[-1], Ldiff[-2])
            if Ldtop >= Ldlow:
                BGR = [190, 190, 190] 
                L = self.Follow_White(img, height, width, BGR, L, L1, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, "White", diff_min)
                return L
        
        # to find the single line from the upper part and the lower part
        L = self.UpperLine(img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, Gi_min, Wi_min, diff_min)
        L = self.BottomLine(img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, i_max, Gi_min, Gi_max, Wi_min, Wi_max, diff_max)
        return L


    # We use this function to find the position if there is a break point
    def IfLineBreak(self, L, Li, i, j, string, img, height, width, rr):
        if((Li[0] == -100)):
            # if the line is empty at the beginning
            if(L[0][0] == -100):
                if(string == 'White'):
                    L = np.array([-100, 0], dtype = 'int')
                    L[0] = -100
                    BGR = [190, 190, 190]
                    # loop if the robot cannot find a point which is white enough
                    while (Li[0] == -100):
                        L = self.ColDet(img, height, width, i, BGR, string, L)
                        BGR[0] = BGR[0] - 10
                        BGR[1] = BGR[1] - 10
                        BGR[2] = BGR[2] - 10
                        if(BGR[0] < 120 and BGR[1] < 110 and BGR[2] < 100):
                            # if there is no white(even grey) point pick the first right point
                            L[0] = 319
                            break
                elif(string == 'Green'):
                    # if there is no green point pick the first left point
                    L = np.array([0, j])

                shape = L.shape
                # if the array is in shape [ , ]
                if(shape == (1,2) or shape == (2,)):
                    L = list(L)
                    num1 = L[0]
                    num2 = L[1]
                    # change the array to the new shape [[ , ]]
                    L = np.array([[num1, num2]], dtype = 'int')
            
            # for the upper part of the line: we pick the first point of L
            if(rr == 0):
                BGR = [180, 170, 160]
                while (Li[0] == -100):
                    Li = self.ColDet(img, height, width, i, BGR, string, Li)
                    BGR[0] = BGR[0] - 10
                    BGR[1] = BGR[1] - 10
                    BGR[2] = BGR[2] - 10
                    if(BGR[0] < 120 and BGR[1] < 110 and BGR[2] < 100):
                        Li = L[-1]
                        break
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


    # We use this function to find a action line following a single line
    def Follow_White(self, img, height, width, BGR, L, L1, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, string1, diff):
        
        if(string1 == "White"):
            i_min = Wi_min
            i_max = Wi_max
        elif(string1 == "Green"):
            i_min = Gi_min
            i_max = Gi_max
        
        L = np.array([[-100, i_min]], dtype = 'int')
        WL = L
        GL = L
        L1 = np.zeros((2), dtype = 'int')
        L2 = np.zeros((2), dtype = 'int')

        # only follow the white line
        if (string1 == "White") :
            for h in range (i_min, i_max, 1):
                L1[0] = -100
                # to find white line
                L1 = self.ColDet(img, height, width, h, BGR, string1, L1)
                L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
                WL = np.vstack((L1, WL))
                # according to the diff find the position of the green line
                L2 = L1[0] - (2*diff)
                # the position cannot be out of the image
                if (L2 <= 0):
                    L2 = 0
                    x_temp = self.MidCal(L1[0], L2)
                else:
                    x_temp = L1[0] - diff
                temp = [x_temp, h]
                print(temp)
                if(h == i_min):
                    WL = np.delete(WL, 0, 0)
                    L = np.delete(L, 0, 0)
                L = np.vstack((L, temp))
        
        # only follow the green line
        elif (string1 == "Green"):
            for h in range (i_min, i_max, 1):
                L2[0] = -100
                # to find green line
                L2 = self.ColDet(img, height, width, h, BGR, string1, L2)
                L2 = self.IfLineBreak(GL, L2, h, h, string1, img, height, width, -1)
                GL = np.vstack((GL, L2))
                # according to the diff find the position of the white line
                L1 = L2[0] + (2*diff)
                # the position cannot be out of the image
                if (L1 >= 319):
                    L1 = 319
                    x_temp = self.MidCal(L1, L2[0])
                else:
                    x_temp = L2[0] + diff
                temp = [x_temp, h]
                if(h == i_min):
                    GL = np.delete(GL, 0, 0)
                    L = np.delete(L, 0, 0)
                L = np.vstack((L, temp))

        return L


    def UpperLine(self, img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, Gi_min, Wi_min, diff):
        # The lower bound of Greenline is smaller then the Whiteline
        if(i_min == Gi_min) and (Gi_min != 0): 
            # from bottom to top
            for h in range (Gi_min, Wi_min, -1):
                L1[0] = -100
                # to find white line
                L1 = self.ColDet(img, height, width, h, BGR, string1, L1)
                L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
                WL = np.vstack((L1, WL))
                # according to the diff find the position of the green line
                x_temp = L1[0] - diff
                temp = [x_temp, h]
                if(h == Gi_min):
                    WL = np.delete(WL, 1, 0)
                    L = np.delete(L, 0, 0)
                # insert at the beginning of the list
                # temp
                #  ↓
                #  ❑ ▅ ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ▅ ▅ 
                L = np.vstack((temp, L))
        
        # The lower bound of Whiteline is smaller then the Greenline
        elif(i_min == Wi_min) and (Wi_min != 0):
            for h in range (Wi_min, Gi_min, -1):
                L2[0] = -100
                # to find green line
                L2 = self.ColDet(img, height, width, h, BGR, string2, L2)
                L2 = self.IfLineBreak(GL, L2, h, h, string2, img, height, width, 0)
                GL = np.vstack((L2, GL))
                # according to the diff find the position of the white line
                x_temp = L2[0] + diff
                temp = [x_temp, h]
                if(h == Wi_min):
                    GL = np.delete(GL, 1, 0)
                    L = np.delete(L, 0, 0)
                L = np.vstack((temp, L))

        return L


    def BottomLine(self, img, height, width, BGR, L, L1, L2, WL, GL, string1, string2, i_min, i_max, Gi_min, Gi_max, Wi_min, Wi_max, diff):
        # The lower bound of Whiteline is smaller then the Greenline
        if(Wi_max < Gi_min): # ???????????????????? if it is necessary ??????????????????
            Wi_max = Gi_max
            i_max = Gi_max
            if(Gi_max == 239):
                Wi_max = 238
                i_max = 238
            i_min = Gi_min
        if(i_max == Gi_max) and (Gi_max != 239): 
            # from top to bottom
            for h in range (Gi_max, Wi_max):
                L1[0] = -100
                # to find the white line
                L1 = self.ColDet(img, height, width, h, BGR, string1, L1)
                L1 = self.IfLineBreak(WL, L1, h, h, string1, img, height, width, -1)
                WL = np.vstack((WL, L1))
                # according to the diff find the position of the green line
                L2 = L1[0] - (2*diff)
                if (L2 <= 0):
                    L2 = 0
                    x_temp = self.MidCal(L1[0], L2)
                else:
                    x_temp = L1[0] - diff
                temp = [x_temp, h]
                if(h == i_min):
                    WL = np.delete(WL, 0, 0)
                    L = np.delete(L, 0, 0)
                # insert at the end of the list
                #                            temp
                #                             ↓
                # ▅ ▅ ▅ ▅ ▅ ▅ ... ▅ ▅ ▅ ▅ ▅ ▅ ❑ 
                L = np.vstack((L, temp))

        elif(i_max == Wi_max) and (Wi_max != 239):
            for h in range (Wi_max, Gi_max):
                L2[0] = -100
                # to find the green line
                L2 = self.ColDet(img, height, width, h, BGR, string2, L2)
                L2 = self.IfLineBreak(GL, L2, h, h, string2, img, height, width, -1)
                GL = np.vstack((GL, L2))
                # according to the diff find the position of the white line
                L1 = L2[0] + (2*diff)
                if (L1 >= 319):
                    L1 = 319
                    x_temp = self.MidCal(L1, L2[0])
                else:
                    x_temp = L2[0] + diff
                temp = [x_temp, h]
                if(h == i_min):
                    WL = np.delete(WL, 0, 0)
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
            # Since the Point should be a integer
            num2 = int(num2)
            return num2
    

    # This function is used to calculate slope of a line
    def LineSlope(self, Line, width, height):
        slope = 0.0
        idx2 = 10
        l = len(Line) - 1

            
        # if(LoC == 2):
        # # if the robot cannot find a line
        # # then it will give the value of LoC
        # # so that the robot will not give back an Error
        #     slope = 0.0
        #     return slope
        
        # if the line is a straight line or a curve line
        # then we need to find three different pairs of points
        # and their corresponding slope
        slope = self.CalSlope(Line, width, height, l, idx2)
        return slope


    # This function is used to calculate the slope of
    # a straight linebetween two points
    def CalSlope(self, Line, width, height, idx1, idx2):
        num0 = width  / 2
        num1 = height - 1
        Pnt1 = np.array([num0, num1], dtype = 'int') # height of Pnt1 should be lower as the one of Pnt2
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
        # avoid Error "divided by zero"
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
            BGR = [170, 170, 170]
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


    def DrawTangent(self, Line, img, slope):
        l = len(Line)
        idx1 = 10

        Pnt0 = Line[idx1]
        cv2.circle(img, (Pnt0[0], Pnt0[1]), 2, (255, 0, 255), 5)
        Pnt1 = [160, 319]
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
        i_bot = height - 1
        i_mid = height - 18

        # resize the image so that we can only focus on the right bottom of the image
        out_pnt = np.float32([[0,        0       ],
                              [0,        height-1],
                              [width-1,  height-1],
                              [width-1,  0       ]])
        inp_pnt = np.float32([[13,       i_mid],    # A         10 -> slope:0.02192631009880022
                                                    #           20 -> slope:0.035073330533225366
                              [0,        i_bot],    # B
                              [width-4,  i_bot],    # C
                              [width-23, i_mid]])   # D     wid-40 -> slope:0.03507333053322537
                                                    #       wid-20 -> slope:0.10969010348702159

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

        slope_n = self.LineSlope(Line, width, height)
        self.slope = slope_n
        
        #########################
        # Fix the slope problem #
        #########################

        self.DrawTangent(Line, img, self.slope)
        print("------------- slope_n = ", slope_n, "-------------")
        print("-------------slope = ", self.slope, " -------------")
        
        
        #####################################################################
        # if we calculate the slope through two point                       #
        # the upper Point is the last Point or the middle Point of the Line #
        # the lower Point is the middle Point of the Image                  #
        ##################################################################### 


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
        elif(self.slope <= 0.035 and self.slope >= 0.00):
        #call the class variable
        #since the publisher create the msg itself
        #you are not allow to use the msg in publisher
        #[ self.i < 20): ]
            msg = Twist()
            msg.linear.x = 0.015
            msg.angular.z = 0.00
            self.publisher_.publish(msg)
            print("Straight Driving!")
        elif(self.slope > 0.035 or self.slope < 0.00):
            msg = Twist()
            msg.linear.x = 0.015
            msg.angular.z = self.slope
            self.publisher_.publish(msg)
            print("Turning!")
        else: 
            msg = Twist()
            msg.linear.x = 0.00
            msg.angular.z = 0.00
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
