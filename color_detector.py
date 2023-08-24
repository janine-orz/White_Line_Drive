import sys
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image

#write a LoC = 3 WHEN the robot is on the right way but need to correct a little bit

# TO DO: write a programm so that the robot can follow one line wenn there is no green line on the left

def ColRec(img, i, j, val):
    color = "Undefined"
    pixel_center = []
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
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        pixel_center = hsv_img[i, j]
        h_val = pixel_center[0]
        s_val = pixel_center[1]
        v_val = pixel_center[2]
        # print("[", j, ", ", i, "]H = ", h_val, "S = ", s_val, "V = ", v_val)
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


# mit i = height, j = width
def ColDet(img, height, width, i, val, string):
    Col = []
    if(string == 'White'):
        temp = 0
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for j in range(width - 150, width, 1):
            color = ColRec(img, i, j, val)
            if color == string:
            #sucess to find the white color
                if (temp == 0):
                    temp = j
                elif (j > temp) and (abs(temp-j) <= 15) and (color_a == string): 
                    temp = j
                if(temp != 0):
                    temp_pos = [temp, i]
                    Col = temp_pos
            color_a = color
            j_temp = j
    elif(string == 'Green'):
        temp = 0
        for j in range(0, width - 170, 1):
            color = ColRec(img, i, j, val)
            if color == string:
            #sucess to find the white color
                # print("color = ", color, "\tposition = [", j, ",", i, "]", "\t bgr = ", img[i, j], "\t hsv = ", hsv_img[i, j])
                if (temp == 0):
                    temp = j
                elif (j > temp) and (abs(temp-j) <= 15) and (color_a == string): 
                    temp = j
                if(temp != 0):
                    temp_pos = [temp, i]
                    Col = temp_pos
            color_a = color
    return Col

def LineForm(img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
    L = []
    L1 = []
    L2 = []
    WL = []
    GL = []
    i_min = max(Wi_min, Gi_min)
    i_max = min(Wi_max, Gi_max)
    # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    for i in range(i_min, i_max):
        # print("==========white line==========")
        L1 = ColDet(img, height, width, i, 180, string1)
        L1 = IfLineBreak(WL, L1, i, i, string1, img, height, width, -1)
        WL.append(L1)
        # print("L1 = ", L1)
        
        # print("==========green line==========")
        L2 = ColDet(img, height, width, i, 180, string2)
        L2 = IfLineBreak(GL, L2, i, i, string2, img, height, width, -1)
        GL.append(L2)
        # print("L1 = ", L1, '\tL1 bgr_value: ', img[L1[1], L1[0]], "\nL2 = ", L2, '\tL2 hsv_value: ', hsv_img[L2[1], L2[0]])

        x_temp = MidCal(L1[0], L2[0])
        temp = [x_temp, i]
        
        L.append(temp)

    print("============ UpperLine() ============")
    L = UpperLine(img, height, width, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min)
    print("============ BottomLine() ============")
    L = BottomLine(img, height, width, L, WL, GL, string1, string2, i_max, Gi_max, Wi_max)
    # print("To fill the upper and bottom part of the line: ", end-start)

    return L


def IfLineBreak(L, Li, i, j, string, img, height, width, rr):
    if((Li == [])or(Li == None)):
        # start = time.time()
        # print("L = ", L)
        if(L == []):
            L.append([0, i])
        if(rr == 0):
            while (Li == []):
                i = i-1
                Li = L[0]
                # print("L = ", L)
            pos = Li[0]
            temp = [pos, j]
        elif(rr == -1):
            while (Li == []):
                i = i-1
                Li = L[-1]
            pos = Li[0]
            temp = [pos, j]
        # while (Li == []):
        #     i = i-1
        #     Li = ColDet(img, height, width, i, string)
        # pos = Li
        # if isinstance(pos, list):
        #     num3 = pos[0]
        #     temp = [num3, j]
        # elif isinstance(pos, int):
        #     temp = [pos, j]
        # end = time.time()
        # print("\tLineForm IfLineBreak(): ", end-start)
        return temp
    else:
        return Li


def UpperLine(img, height, width, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min):
    if(i_min == Gi_min): 
        # The lower bound of Whiteline is smaller then the Greenline
        for h in range (Gi_min, Wi_min, -1):
            L1 = ColDet(img, height, width, h, 165, string1)
            # print("L1 = ", L1)
            L1 = IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
            WL.insert(0, L1)
            L2 = L1[0] - 220
            x_temp = MidCal(L1[0], L2)
            temp = [x_temp, h]
            L.insert(0, temp)
    elif(i_min == Wi_min):
        for h in range (Wi_min, Gi_min, -1):
            L2 = ColDet(img, height, width, h, 165, string2)
            L2 = IfLineBreak(GL, L2, h, h, string2, img, height, width, 0)
            GL.insert(0, L2)
            L1 = L2[0] + 220
            x_temp = MidCal(L1, L2[0])
            temp = [x_temp, h]
            L.insert(0, temp)
    return L


def BottomLine(img, height, width, L, WL, GL, string1, string2, i_max, Gi_max, Wi_max):
    if(i_max == Gi_max): 
        # The lower bound of Whiteline is smaller then the Greenline
        for h in range (Gi_max, Wi_max):
            L1= ColDet(img, height, width, h, 165, string1)
            L1 = IfLineBreak(WL, L1, h, h, string1, img, height, width, -1)
            # print("L1 = ", L1)
            WL.append(L1)
            L2 = 0
            x_temp = MidCal(L1[0], L2)
            temp = [x_temp, h]
            L.append(temp)
    elif(i_max == Wi_max):
        for h in range (Wi_max, Gi_max):
            L2 = ColDet(img, height, width, h, 165, string2)
            L2 = IfLineBreak(GL, L2, h, h, string2, img, height, width, -1)
            GL.append(L2)
            L1 = 319
            x_temp = MidCal(L1, L2[0])
            temp = [x_temp, h]
            L.append(temp)
    return L


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
    l = len(Line)-1

    idx2 = (len(Line))/6
    idx2 = 5 * idx2
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
    slope = CalSlope(Line, l, idx2)
    return slope


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

