import sys
import cv2
import numpy as np
import color_detector_004 as cd
from sensor_msgs.msg import Image

#cy_max_bot = cd.LinePos(img, height, width, i_bot)[0]
#cy_min_bot = cd.LinePos(img, height, width, i_bot)[1]
#SP1 = [cy_max_bot, cy_min_bot, i_bot]

#cy_max_mid = cd.LinePos(img, height, width, i_mid)[0]
#cy_min_mid = cd.LinePos(img, height, width, i_mid)[1]
#SP2 = [cy_max_mid, cy_min_mid, i_mid]

#cv2.circle(img, (SP1[0], SP1[2]), 3, (255, 0, 0), 3)
#cv2.circle(img, (SP1[1], SP1[2]), 3, (255, 0, 0), 3)
#cv2.circle(img, (SP2[0], SP2[2]), 3, (0, 255, 0), 3)
#cv2.circle(img, (SP2[1], SP2[2]), 3, (0, 255, 0), 3)

def print_line(Line, img):
    for k in range(len(Line)):
        temp1 = Line[k]
        #print('(', temp1[0], temp1[1], ')')
        a = int(temp1[0])
        cv2.circle(img, (a, temp1[1]), 2, (0, 0, 255), 2)

def checkagain(img, Line, height, width):
    for j in range(1, 5, 1):
        for i in range(0, len(Line)-(j+1), j):
            Pnt1 = Line[i]
            #print('i: ', i, '; Pnt1: ', Pnt1)
            num1 = Pnt1[0]
            Pnt2 = Line[i+j]
            #print('i: ', i+j, '; Pnt2: ', Pnt2)
            num2 = Pnt2[0]
            num3 = Pnt2[1]
            diff = num2 - num1
            if(abs(diff) > j+2):
                Line_corr = []
                Line_temp = cd.LinePos(img, height, width, num3, 1)
                for k in range(len(Line_temp)):
                    Pnt3 = Line_temp[k]
                    num4 = Pnt3[1]
                    if(abs(num4 - num1) < j+2):
                        ReFindLine(Line, num4)
                        Line_corr.append(Pnt3)
                        #to do:find out the all connected part of this width
                print('Line_corr: ', Line_corr)
                Pnt4 = Line_corr[0]
                num5 = Pnt4[1]
                Pnt5 = Line_corr[-1]
                num6 = Pnt5[1]
                temp = [int(cd.MidCal(num5, num6)), num3]
                print('temp = ', temp)
                Line[i+j] = temp
                print('Line[', i+j, ']',Line[i])


def ReFindLine(Line, num4):
    length = len(Line) - 2
    INDEX = cd.FindLine(Line, length)
    for i in range(0, len(INDEX) - 1):
        diff = abs(INDEX[i+1] - INDEX[i])
        idx = INDEX[i+1]
        for j in range(INDEX[i], idx, 1):
            num7 = Line[j]
            diff = num4 - num7
            if(abs(diff) < 20):
                Line_n = []
                Line_n.append(num7)
        return Line_n


def main():
    PNGList = ["001.png", "002.png", "003.png", "004.png", "005.png", "006.png", 
               "011.png", "012.png", "014.png", "015.png", "017.png", "019.png", 
               "020.png", "021.png", "026.png", "027.png", "028.png", "029.png", 
               "030.png", "033.png", "034.png", "035.png"]
    n = len(PNGList)
    print("the amount of PNGList: ", n)
    #for i in range(0, 10):
    img = cv2.imread(PNGList[6])
    print("imported image: ", PNGList[1])
    #img_msg = cd.cv2_to_imgs2mg(cv_image)
    #img = cd.imgmsg_to_cv2(img_msg)
    height, width, _ = img.shape
    print('height = ', height)
    print('width = ', width)

    i = 230 #225
    i_bot = height - 5
    i_mid = height - 40
    width_n = width - 180

    out_pnt = np.float32([[0, 0], [0, height-1], 
                            [width-1, height-1], [width-1, 0]])
    #print(out_pnt)
    inp_pnt = np.float32([[width_n - 58, i_mid], [width_n, i_bot],
                            [width-15, i_bot], [300, i_mid]])
    #print(inp_pnt)
    maxw = width - 1 - width_n
    maxh = height - 1 - i_mid
    M = cv2.getPerspectiveTransform(inp_pnt, out_pnt)
    img = cv2.warpPerspective(img, M, (width, height), flags = cv2.INTER_LINEAR)

    height, width, _ = img.shape
    print('height = ', height)
    print('width = ', width)

    minwid = 0
    maxwid = height

    #Line = cd.LinePos(img, height, width, 202, 0)
    '''
    if(Line != []):
        i = 0
        for i in range(len(Line)):
            Pnt = Line[i]
            print(Pnt)
            cv2.circle(img, (Pnt[1], Pnt[0]), 1, (255, 0, 0), 5)
    '''

    Line_n = cd.LineForm(img, height, width, minwid, maxwid)
    checkagain(img, Line_n, height, width)

    for i in range(len(Line_n)):
        Pnt = Line_n[i]
        #print(Pnt)
        cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 5)

    #print_line(Line, img)
    #LoC = cd.LineorCurve(Line_n)
    #print('LoC = ', LoC)
    #print('slope = ', cd.LineSlope(Line_n, LoC))
    # 004.png: 0.6514532274507854
    # 003.png: 0.9629943306809362
    # 001.png:

    cv2.imshow("IMG", img)
    cv2.waitKey(0)

    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
