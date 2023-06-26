import sys
import cv2
import numpy as np
import color_detector_004 as cd
from sensor_msgs.msg import Image


def checkagain(img, Line, height, width):
    l = len(Line) - 1
    Line_corr = Line
    for j in range(0, l-14, 1): # l-15
    # j = 132
        print('j = ', j, "but j < ", l-1)
        Pnt0 = Line[j]
        num00 = Pnt0[0]
        num01 = Pnt0[1]
        print('Pnt0 = [(wid)', num00, ',(hei)', num01, ']')
        Line1 = []
        Line2 = []
        L = 0
        R = 0
        for i in range(j+1, j+16, 1): 
            # print('i = ', i, "but i < ", j+16)
            Pnt1 = Line[i]
            num10 = Pnt1[0]
            num11 = Pnt1[1]
            print('\tPnt1 = [(wid)', num10, ',(hei)', num11, ']')
            print('\tPnt0 = [(wid)', num00, ',(hei)', num01, ']')
            if(abs(num00-num10) > 5):
                Line1.append(Pnt1)
                R += 1
            else:
                Line2.append(Pnt1)
                L += 1
                num00 = num10
                num01 = num11
        print('L = ', L)
        print('Line2 = \n', '\t', Line2)
        print('R = ', R)
        print('Line1 = \n', '\t', Line1)
        if((L > R) and (R != 0)):
        #if the reflection exist on the right hand side
            Line = CorrLine(img, height, width, Line_corr, Line1, num00)
            # print('Line_corr = ', Line_corr)
        elif((R > L) and (L != 0)):
        #if the reflection exist onthe left hand side
            Line_corr = 1 # CorrLine(img, height, width, Line_corr, Line2, num10)
            # print(Line_corr)
    return Line_corr


def CorrLine(img, height, width, Line_corr, Line1, num00):
    #Line1: contains the points that need to be correct
    #num00: the comparison width
    l = len(Line1)
    for k in range(l):
        Line_temp = []
        Pnt = Line1[k]
        print('Pnt = ', Pnt)
        hei = Pnt[0]
        wid = Pnt[1]
        print('k = ', k, '; wid = ',wid, 'Pnt = ', Pnt)
        Line_temp = cd.LinePos(img, height, width, wid, 1)
        print(Line_temp)
        # we find out all the white points at width"wid"
        for i in range(len(Line_temp)):
            Pnt2 = Line_temp[i]
            num20 = Pnt2[0] # width of Pnt2
            num21 = Pnt2[1] # height of Pnt2
            # print('\tPnt2 = [(wid)', num20, ',(hei)', num21, ']')
            # print('num20-1 = ', num20-1)
            Pnt = Line_corr[num20-1] # the Point 1pixel above
            print('Pnt = ', Pnt)
            hei = Pnt[0] # height of the Point above
            wid = Pnt[1] # width of the Point above
            if(abs(num21-hei) < 1):
                startpnt = Pnt2
                print('startpnt', i, ' = ', startpnt)
                startidx = i
                Line_pos = ReFindLine(startpnt, startidx, Line_temp)
                Pnt5 = Line_pos[0] # first Point in Line_corr
                num50 = Pnt5[0] # height of first Point
                num51 = Pnt5[1] # width of first Point
                Pnt6 = Line_pos[-1]
                num61 = Pnt6[1]
                num70 = cd.MidCal(num51, num61) # width
                num71 = num50 # height
                Pnt7 = [num70, num71] # Middle Point
                print('\tPnt7 = [(wid)', num70, ',(hei)', num71, ']')
                Line_corr[num20] = Pnt7
                # print(Line_corr)
                break 
        return Line_corr


def print_line(Line):
    for k in range(len(Line)):
        temp1 = Line[k]
        print('k = ', k,'point = ', temp1, '\n')


def ReFindLine(startpnt, i, Line_temp):
    start_width = startpnt[0]
    start_height = startpnt[1]
    Line_for = []
    Line_back = []
    Line = []
    for k in range(i-1, -1, -1):
        # print('k  = ', k)
        Pnt3 = Line_temp[k]
        num30 = Pnt3[0] # height
        num31 = Pnt3[1] # width
        # print('\tPnt3 = [(wid)', num30, ',(hei)', num31, ']')
        diff = start_height - num31
        # print('diff = ', diff)
        if(abs(diff) < 2):
            Line_for.append(Pnt3)
            start_width = num30
            start_height = num31
    for h in range(0, i):
        temp = Line_for[i-1-h]
        Line.append(temp)
    # print_line(Line)
    start_width = startpnt[0]
    start_height = startpnt[1]
    for j in range(i+1, len(Line_temp)-1, 1):
        # print('j = ', j)
        Pnt4 = Line_temp[j]
        num40 = Pnt4[0] # height
        num41 = Pnt4[1] # width
        # print('\tPnt4 = [(wid)', num40, ',(hei)', num41, ']')
        diff = start_height - num41
        if(abs(diff) < 2):
            Line.append(Pnt4)
            start_width = num40
            start_height = num41
    # print_line(Line)
    return Line
        


# def CorrLine(img, height, width, Line_corr, Line1, num00):
#     for k in range(len(Line1)):
#         Line_temp = []
#         wid = Line1[k]
#         print('k = ', k, '; wid = ',wid, 'type of wid: ', type(wid))
#         Line_temp = cd.LinePos(img, height, width, wid, 1)
#         # we find out all the white points at width"wid"
#         print(Line_temp)
#         for i in range(len(Line_temp)-1):
#             # print("i = ", i)
#             Pnt2 = Line_temp[i]
#             num20 = Pnt2[1]
#             diff = num20 - num00
#             if(abs(diff) < 2):
#                 # print("abs(diff) < 2...")
#                 Line_cross = []
#                 for j in range(i+1, len(Line_temp)-1, 1):
#                     # print('j = ', j)
#                     Pnt3 = Line_temp[j]
#                     num30 = Pnt3[1]
#                     # print('num20 = ', num20)
#                     # print('num30 = ', num30)
#                     # print('diff = ', abs(num30-num20))
#                     if(abs(num30-num20) < 2):
#                         Line_cross.append(num30)
#                         num20 = num30
#                         Pnt2 = Pnt3
#                     print('\t', Line_cross)
#                 sum = Line_cross[0] + Line_cross[-1]
#                 num = sum/2
#                 # print(len(Line_cross))
#                 pnt = [int(num), wid]
#                 # print(pnt)
#                 Line_corr[wid] = pnt
#                 print(Line_corr)
#                 return Line_corr


# def CorrLine(img, height, width, Line_corr, Line1, num00):
#     for k in range(len(Line1)):
#         Line_temp = []
#         wid = Line1[k]
#         print('k = ', k, '; wid = ',wid, 'type of wid: ', type(wid))
#         Line_temp = cd.LinePos(img, height, width, wid, 1)
#         print(Line_temp)
        # for i in range(len(Line_temp)-1):
        #     Pnt2 = Line_temp[i]
        #     num20 = Pnt2[0]
        #     print('num20 = ', num20)
        #     diff = num20 - num00
        #     if(abs(diff) < 2):
        #         Line_cross = []
        #         for j in range(i, len(Line_temp)-1, 1):
        #             print('j = ', j)
        #             Pnt3 = Line_temp[j]
        #             num30 = Pnt3[0]
        #             print('num30 = ', num30)
        #             if(abs(num30-num20) < 5):
        #                 Line_cross.append(Pnt2)
        #                 num20 = num30
        #                 print('num20 = ', num20)


# def AddIn(Line, Line_corr, wid):
    

# def CorrLine(img, Line, height, width, diff1, j, i, wid, num1):
#     if(abs(diff1) > j+2):
#         Line_corr = []
#         Line_temp = cd.LinePos(img, height, width, wid, 1)
#         print("wid", wid)
#         for k in range(len(Line_temp)):
#             Pnt4 = Line_temp[k]
#             num4 = Pnt4[1]
#             if(abs(num4 - num1) < j+2):
                
#                 #to do:find out the all connected part of this width
#         print('Line_corr: ', Line_corr)
#         Pnt5 = Line_corr[0]
#         num5 = Pnt4[1]
#         Pnt6 = Line_corr[-1]
#         num6 = Pnt5[1]
#         temp = [int(cd.MidCal(num5, num6)), wid]
#         #print('temp = ', temp)
#         Line[i+j] = temp
#         #print('Line[', i+j, ']',Line[i])


# def ReFindLine(Line, num4):
#     length = len(Line) - 2
#     INDEX = cd.FindLine(Line, length)
#     for i in range(0, len(INDEX) - 1):
#         diff = abs(INDEX[i+1] - INDEX[i])
#         idx = INDEX[i+1]
#         for j in range(INDEX[i], idx, 1):
#             num7 = Line[j]
#             num8 = num7[1]
#             diff = num4 - num8
#             if(abs(diff) < 20):
#                 Line_n = []
#                 Line_n.append(num7)
#         return Line_n