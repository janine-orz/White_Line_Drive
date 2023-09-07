import sys
import cv2
import time
import numpy as np
from sensor_msgs.msg import Image

#write a LoC = 3 WHEN the robot is on the right way but need to correct a little bit

# TO DO: write a programm so that the robot can follow one line wenn there is no green line on the left

def ColRec(img, i, j, BGR):
    color = "Undefined"
    pixel_center = np.array(img[i, j])
    b_val = pixel_center[0]
    g_val = pixel_center[1]
    r_val = pixel_center[2]
    # print("\tB = ", b_val, "\tG = ", g_val, "\tR = ", r_val)

    if b_val > BGR[0] and g_val > BGR[1] and r_val > BGR[2]:
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
            elif h_val < 80:
                color = "Green"
            elif h_val < 131:
                color = "Blue"
            elif h_val < 167:
                color = "Violet"
            else:
                color = "Red"
    return color


def Count(img, height, width, i, j, BGR, string):
    count = 0
    temp = j
    color = "Undefined"
    color_a = "Undefined"

    if(string == 'White'):
        
        for m in range(j, width - 1, 1):
            color = ColRec(img, i, m, BGR)
            if (color == string):
                temp = m
                count = count + 1
            else:
                return count


# mit i = height, j = width
def ColDet(img, height, width, i, BGR, string): 
    Col = np.array([-100, i])
    # initialize temp, color and color_a
    color = "Undefined"
    color_a = "Undefined"
    count = 0
    count_a = 0

    if(string == 'White'):
        temp = 0
        # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for j in range(width - 120, width - 1, 1):
            color = ColRec(img, i, j, BGR)
            if color == string:
            #sucess to find the white color
                count = Count(img, height, width, i, j, BGR, string)
                if (temp == 0) or ((count >= count_a) and abs(Col[0]-j) <= 5): 
                    temp = j
                elif (j > temp) and (abs(temp-j) <= 5) and (color_a == string): 
                    temp = j
                if(temp != 0):
                    Col[0] = temp
                if count > count_a:
                    count_a = count
            color_a = color

    elif(string == 'Green'):
        temp = 0
        # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        for j in range(0, 140, 1):
            color = ColRec(img, i, j, BGR)
            if color == string:
            #sucess to find the white color
                # print("color = ", color, "\tposition = [", j, ",", i, "]", "\t bgr = ", img[i, j], "\t hsv = ", hsv_img[i, j])
                if (temp == 0):
                    temp = j
                elif (j > temp) and (abs(temp-j) <= 5) and (color_a == string): 
                    temp = j
                if(temp != 0):
                    Col[0] = temp
            color_a = color
    
    return Col

def LineForm(img, height, width, string1, string2, Wi_min, Wi_max, Gi_min, Gi_max):
    L1 = np.zeros((2), dtype = 'int')
    L2 = np.zeros((2), dtype = 'int')
    i_min = max(Wi_min, Gi_min)
    i_max = min(Wi_max, Gi_max)
    # hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    L = np.array([[-100, i_min]], dtype = 'int')
    WL = L
    GL = L
    Ldiff = []
    diff_min = 105
    diff_max = 135

    for i in range(i_min, i_max):
        BGR = [170, 170, 160]
        # print("==========white line==========")
        L1 = ColDet(img, height, width, i, BGR, string1)
        L1 = IfLineBreak(WL, L1, i, i, string1, img, height, width, -1)
        # print("L1 = ", L1, '\tL1 bgr_value: ', img[L1[1], L1[0]])
        WL = np.vstack((WL, L1))            
        
        # print("==========green line==========")
        hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
        L2 = ColDet(img, height, width, i, BGR, string2)
        L2 = IfLineBreak(GL, L2, i, i, string2, img, height, width, -1)     
        # print("L2 = ", L2, '\tL2 hsv_value: ', hsv_img[L2[1], L2[0]])
        GL = np.vstack((GL, L2))

        diff_temp = abs(L1[0] - L2[0])
        # print(diff_temp)
        Ldiff.append(diff_temp)

        x_temp = MidCal(L1[0], L2[0])
        temp = [x_temp, i]

        if(i == i_min):
            WL = np.delete(WL, 0, 0)
            GL = np.delete(GL, 0, 0)
            L = np.delete(L, 0, 0)
            diff_min = L1[0] - x_temp
        elif(i == i_max-1):
            diff_max = L1[0] - x_temp
        L = np.vstack((L, temp))
        # print(L)

    BGR = [140, 140, 120]

    if (Ldiff != []) and (Gi_max > 200 and Wi_max > 200):
        Ldmin = max(Ldiff[0], Ldiff[1], Ldiff[2])
        Ldmax = min(Ldiff[-1], Ldiff[-2], Ldiff[-3])
        print("Ldmax-Ldmin", Ldmax-Ldmin)
        # if (Wi_max - Wi_min) < (Gi_max - Gi_min):
        #     string1 = "Green"
        # elif (Wi_max - Wi_min) >= (Gi_max - Gi_min):
        #     string1 = "White"
        if Ldmax-Ldmin < 30:
            L = Follow_White(img, height, width, BGR, L, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, string1, diff_min)
            return L
    
    L = UpperLine(img, height, width, BGR, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min, diff_min)
    L = BottomLine(img, height, width, BGR, L, WL, GL, string1, string2, i_min, i_max, Gi_max, Wi_max, diff_max)

    return L


def IfLineBreak(L, Li, i, j, string, img, height, width, rr):
    if((Li[0] == -100)):
        # start = time.time()
        # print("L = ", L)
        if(L[0][0] == -100):
            if(string == 'White'):
                L = np.array([width-150, j])
            elif(string == 'Green'):
                L = np.array([0, j])
            # print("L = ", L, "type(L) = ", type(L))
            return L
        if(rr == 0):
            # print("Li[0] = ", Li[0])
            while (Li[0] == -100):
                i = i-1
                Li = L[0]
                # print("L = ", L)
        elif(rr == -1):
            # print("Li[0] = ", Li[0], "L[-1] = ", L[-1])
            while (Li[0] == -100):
                i = i-1
                Li = L[-1]
                # print("Li = ", Li)
        pos = Li[0]
        temp = [pos, j]
        return temp
    else:
        return Li


def Follow_White(img, height, width, BGR, L, WL, GL, Wi_min, Wi_max, Gi_min, Gi_max, string1, diff):
    print("===========  Follow_White  ===========")
    i_min = min(Wi_min, Gi_min)
    i_max = max(Wi_max, Gi_max)
    L = np.array([[-100, i_min]], dtype = 'int')
    WL = L
    GL = L

    # The lower bound of Whiteline is smaller then the Greenline
    for h in range (i_min, i_max, 1):
        L1 = ColDet(img, height, width, h, BGR, string1)
        # print("L1 = ", L1)
        L1 = IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
        WL = np.vstack((L1, WL))
        # print("WL = ", WL)
        L2 = L1[0] - (2*diff)
        x_temp = MidCal(L1[0], L2)
        temp = [x_temp, h]
        if(h == i_min):
            WL = np.delete(WL, 0, 0)
            GL = np.delete(GL, 0, 0)
            L = np.delete(L, 0, 0)
        L = np.vstack((L, temp))
    return L


def UpperLine(img, height, width, BGR, L, WL, GL, string1, string2, i_min, Gi_min, Wi_min, diff):
    print("===========  UpperLine  ===========")
    if(i_min == Gi_min) and (Gi_min != 0): 
        # The lower bound of Whiteline is smaller then the Greenline
        for h in range (Gi_min, Wi_min, -1):
            L1 = ColDet(img, height, width, h, BGR, string1)
            # print("L1 = ", L1)
            L1 = IfLineBreak(WL, L1, h, h, string1, img, height, width, 0)
            WL = np.vstack((L1, WL))
            # print("WL = ", WL)
            L2 = L1[0] - (2*diff)
            x_temp = MidCal(L1[0], L2)
            temp = [x_temp, h]
            L = np.vstack((temp, L))
    elif(i_min == Wi_min) and (Wi_min != 0):
        for h in range (Wi_min, Gi_min, -1):
            L2 = ColDet(img, height, width, h, BGR, string2)
            L2 = IfLineBreak(GL, L2, h, h, string2, img, height, width, 0)
            GL = np.vstack((L2, GL))
            L1 = L2[0] + (2*diff)
            x_temp = MidCal(L1, L2[0])
            temp = [x_temp, h]
            L = np.vstack((temp, L))
    return L


def BottomLine(img, height, width, BGR, L, WL, GL, string1, string2, i_min, i_max, Gi_max, Wi_max, diff):
    print("===========  BottomLine  ===========")
    if(i_max == Gi_max) and (Gi_max != 239): 
        # The lower bound of Whiteline is smaller then the Greenline
        for h in range (Gi_max, Wi_max):
            L1= ColDet(img, height, width, h, BGR, string1)
            L1 = IfLineBreak(WL, L1, h, h, string1, img, height, width, -1)
            print("L1 = ", L1, '\tL1 bgr_value: ', img[L1[1], L1[0]])
            WL = np.vstack((WL, L1))
            L2 = L1[0] - (2*diff)
            x_temp = MidCal(L1[0], L2)
            temp = [x_temp, h]
            if(h == i_min):
                WL = np.delete(WL, 0, 0)
                GL = np.delete(GL, 0, 0)
                L = np.delete(L, 0, 0)
            L = np.vstack((L, temp))
    elif(i_max == Wi_max) and (Wi_max != 239):
        for h in range (Wi_max, Gi_max):
            L2 = ColDet(img, height, width, h, BGR, string2)
            L2 = IfLineBreak(GL, L2, h, h, string2, img, height, width, -1)
            GL = np.vstack((GL, L2))
            L1 = L2[0] + (2*diff)
            x_temp = MidCal(L1, L2[0])
            temp = [x_temp, h]
            if(h == i_min):
                WL = np.delete(WL, 0, 0)
                GL = np.delete(GL, 0, 0)
                L = np.delete(L, 0, 0)
            L = np.vstack((L, temp))
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


def LineorCurve(Line, img):
    slope = 0.0
    LoC = 2

    idx0 = len(Line)-1
    pos0 = Line[idx0]

    idx1 = 0
    pos1 = Line[idx1]
    while(pos1[0] == -100):
        idx1 = idx1 + 1
        pos1[0] = Line[0]
    
    slope1 = CalSlope(Line, idx0, idx1)
    const1_0 = pos0[0] - slope1*pos0[1]
    const1_1 = pos1[0] - slope1*pos1[1]
    const1 = (const1_0 + const1_1) / 2

    idx2 = (int)(len(Line)/5)
    pos2 = Line[idx2]

    idx3 = 4 * idx2
    pos3 = Line[idx3]
    
    slope2 = CalSlope(Line, idx3, idx2)
    const2_0 = pos2[0] - slope2*pos2[1]
    const2_1 = pos3[0] - slope2*pos3[1]
    const2 = (const2_0 + const2_1) / 2
    
    idx4 = (int)(len(Line)/2)
    pos4 = Line[idx4]
    print("pos5 : [", pos4[0], pos4[1], "]")
    cv2.circle(img, (pos4[0], pos4[1]), 1, (0, 0, 255), 5)
    
    pos5_1 = pos4[1]
    pos5_0 = (int)(slope1*pos4[1] + const1)
    print("pos5 : [", pos5_0, pos4[1], "]")
    cv2.circle(img, (pos5_0, pos4[1]), 1, (0, 0, 255), 5)
    
    pos6_1 = pos4[1]
    pos6_0 = (int)(slope2*pos4[1] + const2)
    print("pos6 : [", pos6_0, pos4[1], "]")
    cv2.circle(img, (pos6_0, pos4[1]), 1, (0, 0, 255), 5)

    if (abs(pos5_0-pos4[0]) < 5) and (abs(pos6_0-pos4[0]) < 5):
        LoC = 0
    elif (abs(pos5_0 - pos4[0]) >= 5) or (abs(pos6_0 - pos4[0]) >= 5):
        LoC = 1
    else:
        LoC = 2
    
    return LoC


def LoCCal(Pnt3, slope, const):
    y1 = 0
    Pnt4 = Pnt3
    y1 = (Pnt4[1]*slope) + const
    y1 = int(y1)
    diff = Pnt4[0] - y1
    if(abs(diff) < 2):
        LoC = 0
    else:
        LoC = 1
    return LoC


# This function is used to calculate slope of a line
def LineSlope(Line, LoC):
    slope = 0.0
    idx = 0
    l = len(Line)-1

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
    print("===========  LineSLope  ===========")
    slope = CalSlope(Line, l, idx2)
    return slope


def CalSlope(Line, idx1, idx2):
    Pnt1 = Line[idx1] # height of Pnt1 should be lower as the one of Pnt2
    Pnt2 = Line[idx2]
    # print('Point[', idx1, ']', Pnt1, 'and Point[', idx2, ']', Pnt2)
    slope = GetAngl(Pnt1, Pnt2)
    while slope == -100:
        idx2 = idx2 + 1
        Pnt2 = Line[idx2]
        slope = GetAngl(Pnt1, Pnt2)
    return slope


def GetAngl(Pnt1, Pnt2):
    angle = 0.0
    opp = Pnt2[0] - Pnt1[0]
    adj = Pnt2[1] - Pnt1[1]
    if adj == 0:
        return -100
    angle = np.arctan(opp / adj)

    return angle

