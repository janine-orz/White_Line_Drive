import sys
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image

#write a LoC = 3 WHEN the robot is on the right way but need to correct a little bit

# TO DO: write a programm so that the robot can follow one line wenn there is no green line on the left

def ColRec(img, i, j):
    color = "Undefined"
    pixel_center = []
    pixel_center = img[i, j]
    b_val = pixel_center[0]
    g_val = pixel_center[1]
    r_val = pixel_center[2]
    # print("\tB = ", b_val, "\tG = ", g_val, "\tR = ", r_val)

    if b_val > 180 and g_val > 180 and r_val > 180:
        color = "White"
    elif b_val < 90 and g_val < 110 and r_val < 90:
        color = "Black"
    else:
        # print("...............Searching in HSV_image...................")
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pixel_center = hsv_img[i, j]
        h_val = pixel_center[0]
        s_val = pixel_center[1]
        v_val = pixel_center[2]
        # print("H = ", h_val)
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



def ColDet(img, height, width, i, string):
    Col = []
    if(string == 'White'):
        for j in range(width - 120, width, 1):
            color = ColRec(img, i, j)
            # print('[', i, ',', j, '] = ', color)
            if color == string:
            #sucess to find the white color
                temp = [i, j]
                Col.append(temp)
        return Col
    elif(string == 'Green'):
        for j in range(0, width - 180, 1):
            color = ColRec(img, i, j)
            # print('[', i, ',', j, '] = ', color)
            if color == string:
            #sucess to find the white color
                temp = [i, j]
                Col.append(temp)
        return Col


def LineForm(img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
    L = []
    L1 = []
    L2 = []
    Line1_temp = []
    Line2_temp = []
    i_min = max(Wi_min, Gi_min)
    i_max = min(Wi_max, Gi_max)
    for i in range(i_min, i_max):
        Line1_temp = ColDet(img, height, width, i, string1)
        Line2_temp = ColDet(img, height, width, i, string2)
        L1 = Lineform(Line1_temp, L1, i, i, string1, img, height, width)
        L2 = Lineform(Line2_temp, L2, i, i, string2, img, height, width)
        print("L1 = ", L1, "\t\tL2 = ", L2)
        x_temp = MidCal(L1[0], L2[0])
        temp = [x_temp, i]
        L.append(temp)
    if(i_min == Gi_min): 
        # The lower bound of Whiteline is smaller then the Greenline
        for h in range (Wi_min, Gi_min):
            Line1_temp = ColDet(img, height, width, h, string1)
            L1 = Lineform(Line1_temp, L1, h, h, string1, img, height, width)
            L2 = 0
            x_temp = MidCal(L1[0], L2)
            temp = [x_temp, h]
            L.append(temp)
    elif(i_min == Wi_min):
        for h in range (Gi_min, Wi_min):
            Line2_temp = ColDet(img, height, width, h, string2)
            L2 = Lineform(Line2_temp, L2, h, h, string2, img, height, width)
            L1 = 319
            x_temp = MidCal(L1, L2[0])
            temp = [x_temp, h]
            L.append(temp)
    if(i_max == Gi_max): 
        # The Greenline is shorter then the Whiteline
        for k in range (Gi_max, Wi_max):
            Line1_temp = ColDet(img, height, width, k, string1)
            L1 = Lineform(Line1_temp, L1, k, k, string1, img, height, width)
            L2 = 0
            x_temp = MidCal(L1[0], L2)
            temp = [x_temp, k]
            L.append(temp)
    elif(i_max == Wi_max):
        for k in range (Wi_max, Gi_max):
            Line2_temp = ColDet(img, height, width, k, string2)
            L2 = Lineform(Line2_temp, L2, k, k, string2, img, height, width)
            L1 = 319
            x_temp = MidCal(L1, L2[0])
            temp = [x_temp, k]
            L.append(temp)
    return L


def Lineform(Line_temp, L, i, j, string, img, height, width):
    print('i = ', i, ', string = ', string)
    LL = []
    LL.append(L)
    if((Line_temp == [])or(Line_temp == None)):
        print(type(LL), "\tL = ", L, "\tLL = ", LL)
        pos = LL[-1]
        # print("type(pos) = ", type(pos), "\tpos = ", pos, "(type(pos) == 'list')", isinstance(pos, list))
        if isinstance(pos, list):
            # print("... pos is list!! ...")
            num3 = pos[0]
            temp = [num3, j]
        else:
            pos = LL[0]
            temp = [pos, j]
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


def MidCal(a, b):
    num1 = (a - b)/2
    if(a > b):
        num2 = b + num1
        num2 = int(num2)
        return num2
    elif(b > a):
        num2 = a + num1
        num2 = int(num2)
        return num2


def LineorCurve(Line):
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
        LoC1 = LoCCal(Pnt1, slope, const)
        # comparing the LoC of a point at 1/4 of the line
        
        num2 = len(Line)/2
        num2 = int(num2)
        Pnt2 = Line[num2]
        LoC2 = LoCCal(Pnt2, slope, const)
        # comparing the LoC of a point at 1/2 of the line

        num3 = (len(Line)/4)*3
        num3 = int(num3)
        Pnt3 = Line[num3]
        LoC3 = LoCCal(Pnt3, slope, const)
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


def LoCCal(Pnt3, slope, const):
    y1 = 0
    Pnt4 = Pnt3
    y1 = (Pnt4[1]*slope) + const
    y1 = int(y1)
    diff = Pnt4[0] - y1
    if(abs(diff) < 5):
        LoC = 0
    else:
        LoC = 1
    return LoC


# This function is used to calculate slope of a line
def LineSlope(Line, LoC):
    slope = 0.0
    idx = 0
    l = len(Line) - 1

    idx1 = (len(Line))/3
    idx1 = int(idx1)

    idx2 = (len(Line))/3
    idx2 = 2 * idx2
    idx2 = int(idx2)

    if(LoC == 0):
    # if the line is a straight line
    # then we need to find three different pairs of points
    # and their corresponding slope
        slope1 = CalSlope(Line, l, idx2)
        slope2 = CalSlope(Line, idx2, idx1)
        slope3 = CalSlope(Line, idx1, idx)
        slope = (slope1+slope2+slope3) / 3
        return slope

    elif(LoC == 1):
    # if the line is a curve
    # then we only need to calculate the slope of the pairs at bottom
        slope = CalSlope(Line, l, idx1)
        # the slope that the robot uses to test, how much should the robot turn
        return slope
    elif(LoC == 2):
    # if the robot cannot find a line
    # then it will give the value of LoC
    # so that the robot will not give back an Error
        return LoC

    # if slope > 0.1835:#[!!!]MIGHT NOT CORRECT
    # # it should be 0.8015
    # # the PERSPECTIVE angle 0.5
    # #           plus
    # # DIFFERENCE between pi and max radians of turtlebot Burger 0.3015
    # # HOWEVER
    # # we still need to concider about the extra slope
    # # caused by the distance of the camera 0.2
    #     slope = slope - 0.1565
    # elif (slope <= 0.1835) and (slope > 0.0615):
    #     slope = 0.0
    # elif (slope < 0.0615):
    #     slope = slope - 0.0615
    # return slope


def CalSlope(Line, idx1, idx2):
    Pnt1 = Line[idx1] # height of Pnt1 should be lower as the one of Pnt2
    Pnt2 = Line[idx2]
    # print('we will calculate the tangent line with Point', Pnt1, 'and Point', Pnt2)
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

