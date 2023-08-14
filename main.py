import sys
import cv2
import time
import numpy as np
import color_detector as cd
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


def DeterHeight1(img, height, width, idx1, idx2, string1, string2):
    if(idx1 <= idx2):
        a = 1
    elif(idx1 > idx2):
        a = -1
    for i in range(idx1, idx2, a):
        # print("string2 = ", string2)
    # for idx1 < idx2: Check from upper to bottom, if there is "string" points
    # for idx1 > idx2: Check from bottom to upper, if there is "string" points
        J = (cd.ColDet(img, height, width, i, string1))
        # print('J = ', J)
        U = ((J == None)or(J == []))
        # print(string1, 'at', i, 'cd.ColDet(img,', height, ',', width, ',', i, ',', string1, '):', U)
        if((J == None)or(J == [])):
            idx1 = i
            # print('\t i = ', i, '\tidx1 == height-1:', idx1 == height-1)
            if((idx1 == height-1)and(string2 == 'min')):
                return i
            elif((idx1 == 0)and(string2 == 'max')):
                return i
        else:
            return i


def DeterHeight2(i_min, i_max):
    if (i_min == 239) and (i_max == 0):
        i_max = i_min
    return i_min, i_max


def DrawTangent(Line, img, string):
    l = len(Line) - 1
    idx1 = (len(Line))/6
    idx1 = 5 * idx1
    idx1 = int(idx1)
    idx2 = (len(Line))/3
    idx2 = 2 * idx2
    idx2 = int(idx2)
    if(string == 'bot'):
        Pnt0 = Line[l]
        cv2.circle(img, (Pnt0[0], Pnt0[1]), 1, (255, 0, 0), 2)
        Pnt1 = Line[idx2]
        cv2.circle(img, (Pnt1[0], Pnt1[1]), 1, (255, 0, 0), 4)
        slope = (Pnt1[0] - Pnt0[0])/(Pnt1[1] - Pnt0[1])
        print('slope = ', slope)
        const = Pnt0[0] - (Pnt0[1] * slope)
        print('const = ', const)
        for i in range(120):
            Pnt = Line[l-i]
            print(Pnt)
            y = Pnt[1]
            x = Pnt[1]*slope + const
            x = int(x)
            print('[', x, ',', y, ']')
            cv2.circle(img, (x, y), 1, (0, 0, 255), 2)
        return 0
    if(string == 'line'):
        Pnt0 = Line[idx2]
        cv2.circle(img, (Pnt0[0], Pnt0[1]), 1, (255, 0, 0), 2)
        Pnt1 = Line[idx1]
        cv2.circle(img, (Pnt1[0], Pnt1[1]), 1, (255, 0, 0), 4)
        slope = (Pnt1[0] - Pnt0[0])/(Pnt1[1] - Pnt0[1])
        print('slope = ', slope)
        const = Pnt0[0] - (Pnt0[1] * slope)
        print('const = ', const)
        for i in range(120):
            Pnt = Line[idx2-i]
            print(Pnt)
            y = Pnt[1]
            x = Pnt[1]*slope + const
            x = int(x)
            print('[', x, ',', y, ']')
            cv2.circle(img, (x, y), 1, (0, 175, 175), 2)


def main():

    PNGList = ["001.png", "002.png", "003.png", "004.png", ]
            #    "005.png", 
            #    "006.png", "007.png", "008.png", "009.png", "010.png", 
            #    "011.png", "012.png", "013.png", "014.png", "015.png", 
            #    "016.png", "017.png", "018.png", "019.png", "020.png", ]
            #   ["021.png", "022.png", "023.png", "024.png", "025.png", 
            #    "026.png", "027.png", "028.png", "029.png", "030.png", 
            #    "031.png", "032.png", "033.png", "034.png", "035.png", ]
    n = len(PNGList)
    # for i in range(0, n-1):
    start = time.time()
    i = 3 # 14
    print("the amount of PNGList: ", PNGList[i])
    img_cv = cv2.imread(PNGList[i])
    # print("\timported image", i, ": ", PNGList[i], '\n')
    #img_msg = cd.cv2_to_imgs2mg(cv_image)
    #img = cd.imgmsg_to_cv2(img_msg)
    height, width, _ = img_cv.shape
    print('height = ', height)
    print('width = ', width)

    i_bot = height - 5
    i_mid = height - 60
    # height = height-1
    # width = width-1
    width_n = width - 250
    # print('width_n = ', width_n)

    out_pnt = np.float32([[0, 0],
                        [0, height-1],
                        [width-1, height-1],
                        [width-1, 0]])
    inp_pnt = np.float32([[0, i_mid], # width_n + 58
                        [0, i_bot], # width_n + 8
                        [width-1, i_bot], # width-1
                        [width-1, i_mid]]) # 300

    M = cv2.getPerspectiveTransform(inp_pnt, out_pnt)
    img = cv2.warpPerspective(img_cv, M, (width, height), flags = cv2.INTER_LINEAR)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
    
    height, width, _ = img.shape
    Wminhigt = DeterHeight1(img, height, width, 0, height, "White", "min")
    Gminhigt = DeterHeight1(img, height, width, 0, height, "Green", "min")
    Wmaxhigt = DeterHeight1(img, height, width, height-1, -1, "White", "max")
    Gmaxhigt = DeterHeight1(img, height, width, height-1, -1, "Green", "max")
    Wminhigt, Wmaxhigt = DeterHeight2(Wminhigt, Wmaxhigt)
    Gminhigt, Gmaxhigt = DeterHeight2(Gminhigt, Gmaxhigt)
    print("Wminhigt = ", Wminhigt, "\t\tGminhigt = ", Gminhigt)
    print("Wmaxhigt = ", Wmaxhigt, "\t\tGmaxhigt = ", Gmaxhigt)

    # A = [24, 103]
    # print(cd.ColRec(img, A[1], A[0]))
    # cv2.circle(img, (A[0], A[1]), 5, (255, 0, 0), 1)

    # print(cd.ColDet(img, height, width, 191, "Green"))

    Line = cd.LineForm(img, height, width, "White", "Green", Wminhigt, Wmaxhigt, Gminhigt, Gmaxhigt)
    # GLine = cd.LineForm(img, height, width, "Green" or "Yellow", minhigt, maxhigt)
    # GLine = cd.checkagain(img, GLine, height, width, "Green")
    # GLine = cd.ReformLine(GLine, height)
    # Line = cd.ActionPath(img, height, width, minhigt, maxhigt, "White", "Green" or "Yellow")
    # Line = ReformLine(Line, height)
    # checkagain(img, WLine, height, width, Rstring)
    if((Line != 0)and(Line != 1)):
        for i in range(len(Line)):
            Pnt = Line[i]
            # Pnt1 = WLine[i]
            # Pnt2 = GLine[i]
            # print('i = ', i, '; Pos = ', Pnt)
            cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 3)
            # cv2.circle(img, (Pnt1[0], Pnt1[1]), 1, (125, 255, 0), 3)
            # cv2.circle(img, (Pnt2[0], Pnt2[1]), 1, (0, 175, 175), 3)

    # LoC = cd.LineorCurve(Line)
    # print('LoC = ' , LoC)
    # slope = cd.LineSlope(Line, LoC)
    # if(LoC == 0):
    #     # DrawTangent(WLine, img, "bot")
    #     # DrawTangent(WLine, img, "line")
    #     print('slope = ', slope)
    # elif(LoC == 1):
    #     # DrawTangent(WLine, img, "bot")
    #     # DrawTangent(WLine, img, "line")
    #     print('slope = ', slope)
    

    end = time.time()
    print(end-start)

    # cv2.imshow("IMG_CV", img_cv)
    cv2.imshow("IMG", img)
    # cv2.imshow("GIMG", Gimg)
    cv2.imshow("IMG_HSV", img_hsv)
    cv2.waitKey(0)
    

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
