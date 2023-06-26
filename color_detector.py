import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image

#write a LoC = 3 WHEN the robot is on the right way but need to correct a little bit

def imgmsg_to_cv2(img_msg):
    if img_msg.encoding != "bgr8":
        rospy.logerr("This Coral detect node has been hardcoded to the 'bgr8' encoding. Come change the code if you are actually trying to implement a new camera")
    dtype = np.dtype("uint8")
    dtype = dtype.newbyteorder('>' if img_msg.is_bigendian else'<')
    image_opencv = np.ndarray(shape=(img_msg.height, img_msg.width, 3), # and three channels of data. Since OpenCV works with bgr natively, we dont need to reorder the channels
                              dtype = dtype, buffer = img_msg.data)
    if img_msg.is_bigendian == (sys.byteorder == 'little'):
        image_opencv = image_opencv.byteswap().newbyteorder()
    return image_opencv


def cv2_to_imgsmg(cv_image):
    img_msg = Image()
    img_msg.height = cv_image.shape[0]
    img_msg.width = cv_image.shape[1]
    img_msg.encoding = "bgr8"
    img_msg.is_bigendian = 0
    img_msg.data = cv_image.tobytes()
    img_msg.step = len(img_msg.data)
    return img_msg


#def img_read():
#img = cv2.imread("004.png")
#height, width, _ = img.shape
#    return img, height, width

def ColRec(img, i, j):
    color = "Undefined"
    pixel_center = []
    pixel_center = img[i, j]
    b_val = pixel_center[0]
    g_val = pixel_center[1]
    r_val = pixel_center[2]

    if b_val > 170 and g_val > 170 and r_val > 170:
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


def ColDet(img, height, width, i):
    Col = []
    for j in range(width):
        color = ColRec(img, i, j)
        if color == "White":
        #sucess to find the white color
            temp = [i, j]
            Col.append(temp)
    return Col


def LinePos(img, height, width, i, rr):
    Line = []
    length = 0
    Line = ColDet(img, height, width, i)
    #print(i)
    # for i in range(len(Line)):
    #     print(i, ':', Line[i])

    #sucess to finde the white color in one line
    length = len(Line) - 2
    if(Line != [])and(rr == 0):
        INDEX = FindLine(Line, length)
        #print('INDEX = ', INDEX)
        Line_N = DelRef(Line, INDEX)
        #print('Line_N = ', Line_N)
        return Line_N
    elif(Line != [])and(rr == 1):
        return Line


def FindLine(Line, length):
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
            idx_n = bubblesort(length, idx, Line)
            #print('idx_n = ', idx_n)
            if(idx == idx_n):
                INDEX.append(idx_n)


def DelRef(Line, INDEX):
    Line_n = []
    #print('INDEX = ', INDEX)
    for i in range(0, len(INDEX)-1):
        diff = abs(INDEX[i+1] - INDEX[i])
        idx = INDEX[i+1]
        #print('idx = ', idx)
        #print('i = ', i, '; diff = ', diff)
        while(diff >= 7 and diff < 28):
            #print('in WHILE')
            #print('len(INDEX) = ', len(INDEX))
            num = Line[idx]
            for k in range(len(Line)-1, -1, -1):
                #print("k = ", k)
                #if(num[1] < 320-k*5):
                Line_n = []
                for j in range(INDEX[i], INDEX[i+1]):
                    #print('INDEX[i] = ', INDEX[i])
                    #print('j = ', j+1, ';\tLine[j]', Line[j+1])
                    Line_n.append(Line[j+1])
                # print('Line_n = ', Line_n)
            break
    return Line_n


def bubblesort(length, idx, Line):
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


def LineForm(img, height, width, i_min, i_max):
    L = []
    Line_temp = []
    for i in range (i_min, i_max):
        #print('i = ', i, ', min = ', i_min, ', max = ', i_max)
        Line_temp = LinePos(img, height, width, i, 0)
        if(Line_temp == []):
            L = []
            return L
        elif(Line_temp != []):
            l = len(Line_temp)
            a = Line_temp[0]
            num1 = a[1]
            b = Line_temp[l-1]
            num2 = b[1]
            if(num1 != None)and(num2 != None):
                temp = [int(MidCal(num1, num2)), i]
                # print('[', MidCal(a, b), ',', i, ']')
                L.append(temp)
    return L





def RighorLeft(Line):
    R = 0
    L = 0
    for i in range(len(Line) - 2):
        if(Line[i] < Line[i+1]):
            L += 1
        elif(Line[i] > Line[i+1]):
            R += 1
    print('L = ', L)
    print('R = ', R)
    if(L < 8 or R > 0):
        RoL = 0
    elif(L >= 8):
        RoL = 1
    return RoL


def MidCal(a, b):
    num1 = (a - b)/2
    num2 = b + num1
    num2 = int(num2)
    return num2


#def aLine(Line):
    #for i in range(len(Line) - 2):
        #Pnt1 = Line[i]
        #Pnt2 = Line[i+1]
        #if Pnt1[0] <= Pnt2[0]:
            #var = 0
            #return var


def LineorCurve(Line):
    LoC = 0
    aLine = 1
    if(Line != []):
        Pnt1 = Line[0]
        Pnt2 = Line[len(Line) - 1]
        slope = (Pnt2[1] - Pnt1[1])/(Pnt2[0] - Pnt1[0])
        const = Pnt1[1] - (Pnt1[0] * slope)

        num1 = len(Line)/4
        num1 = int(num1)
        Pnt3 = Line[num1]
        LoC = LoCCal(Pnt3, slope, const)
        '''
        num2 = len(Line)/2
        num2 = int(num2)
        Pnt3 = Line[num2]
        LoC2 = LoCCal(Pnt3, slope, const)

        num3 = (len(Line)/4)*3
        num3 = int(num3)
        Pnt3 = Line[num3]
        LoC3 = LoCCal(Pnt3, slope, const)

        sum = LoC1 + LoC2 + LoC3
        LoC = sum/3
        '''
        return LoC
    else:
        LoC = 2
        return LoC


def LoCCal(Pnt3, slope, const):
    y1 = Pnt3[1]
    Pnt4 = Pnt3
    y2 = (slope * Pnt4[0]) + const
    diff = y1 - y2
    if(abs(diff) < 1.5 and aLine == 0):
            LoC = 0
    else:
        LoC = 1
    return LoC


def LineSlope(Line, LoC):
    slope = 0.0
    idx = 0
    if(LoC == 0):
        idx1 = (len(Line))/4
        idx1 = int(idx1)
        slope1 = CalSlope(Line, idx1)

        idx2 = (len(Line))/2
        idx2 = int(idx2)
        slope2 = CalSlope(Line, idx2)

        idx3 = (len(Line))/4
        idx3 = 3 * idx3
        idx3 = int(idx3)
        slope3 = CalSlope(Line, idx3)

        print('slope1 = ', slope1)
        print('slope2 = ', slope2)
        print('slope3 = ', slope3)

        slope = (slope1+slope2+slope3) / 3
    elif(LoC == 1):
        idx = (len(Line))/4
        idx = 3 * idx
        idx = int(idx)
        slope = CalSlope(Line, idx)
        print(slope)
    elif(LoC == 2):
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


def CalSlope(Line, idx):
    l = len(Line) - 1
    Pnt1 = Line[l]
    Pnt2 = Line[idx]
    print('we will calculate the tangent line with Point', Pnt1, 'and Point', Pnt2)
    slope = GetAngl(Pnt1, Pnt2)
    return slope


def GetAngl(Pnt1, Pnt2):
    angle = 0.0
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

