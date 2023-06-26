import sys
import cv2
import numpy as np
from sensor_msgs.msg import Image

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
'''
i = 229
j = 1
print(ColRec(img, i, j))
cv2.circle(img, (j, i), 3, (255, 0, 0), 3)
'''

def ColDet(img, height, width, i):
    Col = []
    # i = height - 50
    for j in range(width):
        color = ColRec(img, i, j)
        if color == "White":
        #sucess to find the white color
            temp = [i, j]
            Col.append(temp)
    return Col

# print(ColDet(img, height, width))


def LinePos(img, height, width, i):
    Line = []
    length = 0
    Line = ColDet(img, height, width, i)
    print(i)
    for i in range(len(Line)):
        print(i, ':', Line[i])

    #sucess to finde the white color in one line
    length = len(Line) - 2
    if Line != []:
        INDEX = FindRef(Line, length)
        #print('INDEX = ', INDEX)
        Line_N = DelRef(Line, INDEX)
        #print('Line_N = ', Line_N)
        return Line_N


def FindRef(Line, length):
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
    print('INDEX = ', INDEX)
    for i in range(0, len(INDEX)-1):
        diff = abs(INDEX[i+1] - INDEX[i])
        idx = INDEX[i+1]
        print('idx = ', idx)
        print('i = ', i, '; diff = ', diff)

        while (diff >= 5 and diff <= 13):
            num = Line[idx]
            if(len(INDEX) < 3) or (num[1] < 310):
                Line_n = []
                for j in range(INDEX[i], INDEX[i+1]):
                    print('INDEX[i] = ', INDEX[i])
                    print('j = ', j+1, ';\tLine[j]', Line[j+1])
                    Line_n.append(Line[j+1])
            i+=1
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


'''
def LinePos(img, height, width, i):
    Line = []
    Line = ColDet(img, height, width, i)
    print(Line)
    #sucess to finde the white color in one line
    length = len(Line)
    if Line != []:
        Line_max = Line[length - 1]
        cy_max = Line_max[1]
        FindMax(Line, length, cy_max)
        print('cy_max = ', cy_max)
        ##sucess to finde the right white point
        cy_min = FindMin(Line, length)
        print('cy_min = ', cy_min)
        #sucess to finde the left white point
        return cy_max, cy_min


def FindMin(Line, length):
    minimum = 0
    Line_max = Line[length - 1]
    maximum = Line_max[1]
    Line_min = Line[0]
    minimum = Line_min[1]
    for k in range(length-1):
        Line_temp = Line[k]
        tempmin = Line_temp[1]
        return minimum
        while abs(tempmin - maximum) <= 10:
            minimum = tempmin
            #print(minimum)
            Line_min = Line_temp
            return minimum


def FindMax(Line, length, maximum):
    maximum = 0
    for k in range(length-1):
        Line_temp = Line[k+1]
        if maximum > Line_temp[1]:
            maximum = maximum
        elif maximum < Line_temp[1]:
            maximum = Line_temp[1]
    return maximum
'''


def LineForm(img, height, width, i_min, i_max):
    L = []
    for i in range (i_min, i_max):
        # print('i = ', i, ', min = ', i_min, ', max = ', i_max)
        Line_temp = LinePos(img, height, width, i)
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


def MidCal(a, b):
    num1 = (a - b)/2
    num2 = b + num1
    num2 = int(num2)
    return num2


def LineorCurve(Line):
    LoC = 0
    Pnt1 = Line[0]
    print('Pnt1 = ', Pnt1)
    Pnt2 = Line[len(Line) - 1]
    print('Pnt2 = ', Pnt2)
    num = len(Line) - 15
    num = int(num)
    Pnt3 = Line[num]
    print('Pnt3 = ', Pnt3[1])
    y1 = Pnt3[1]
    Pnt4 = Pnt3
    slope = (Pnt2[1] - Pnt1[1])/(Pnt2[0] - Pnt1[0])
    print(slope)
    const = Pnt1[1] - (Pnt1[0] * slope)
    y2 = (slope * Pnt4[0]) + const
    print('Pnt4 = ', y2)
    diff = y1 - y2
    print('Diff = ', diff)
    if(abs(diff) < 1.5):
        LoC = 0
    else:
        LoC = 1
    return LoC



def LineSlope(Line):
    slope = 0.0
    Pnt1 = Line[0]
    Pnt2 = Line[len(Line) - 1]
    slope = GetAngl(Pnt1, Pnt2)
    if slope > 0.8015:
    # it should be 0.8015
    # the PERSPECTIVE angle 0.5
    #           plus
    # DIFFERENCE between pi and max radians of turtlebot Burger 0.3015
    # HOWEVER
    # we still need to concider about the extra slope
    # caused by the distance of the camera 0.2
        slope = slope - 0.8015
    elif (slope <= 0.8015) and (slope > 0):
        slope = 0.0
    elif (slope < -0.8015):
        slope = slope + 0.8015
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

