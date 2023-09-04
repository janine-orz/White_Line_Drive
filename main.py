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
    # for idx1 < idx2: Check from upper to bottom, if there is "string" points
    # for idx1 > idx2: Check from bottom to upper, if there is "string" points
        
        BGR = [160, 160, 150]
        J = (cd.ColDet(img, height, width, i, BGR, string1))
        U = (J[0] == -100)
        # print('i = ', i, '\tJ = ', J, '\tU = ', U, '\tcolor: ', string1)
        if U == True:
            idx1 = i
            # print('\t i = ', i, '\tidx1 == height-1:', idx1 == height-1)
            if((idx1 == height-1)and(string2 == 'min')):
                return 0
            elif((idx1 == 0)and(string2 == 'max')):
                return i
        elif U == False:
            return i


def DrawTangent(Line, img, slope):
    
    l = len(Line)
    idx1 = (l)/5
    idx1 = 4 * idx1
    idx1 = int(idx1)

    Pnt0 = Line[idx1]
    cv2.circle(img, (Pnt0[0], Pnt0[1]), 2, (255, 0, 255), 5)
    Pnt1 = Line[l-1]
    cv2.circle(img, (Pnt1[0], Pnt1[1]), 2, (255, 0, 255), 5)
    const = Pnt1[0] - (Pnt1[1] * slope)
    for i in range(1, 60, 1):
        Pnt = Line[l-i]
        # print(Pnt)
        y = Pnt[1]
        x = Pnt[1]*slope + const
        x = int(x)
        # print('[', x, ',', y, ']')
        cv2.circle(img, (x, y), 1, (0, 175, 175), 2)


def main():

                #  0          1          2          3          4
    PNGList = ["001.png", "002.png", "003.png", "004.png", "005.png", 
                #  5          6          7          8          9
               "006.png", "007.png", "008.png", "009.png", "010.png", 
                #  10         11         12         13         14
               "011.png", "012.png", "013.png", "014.png", "015.png", 
                #  15         16         17         18         19
               "016.png", "017.png", "018.png", "019.png", "020.png", 
                #  20         21         22         23         24
               "021.png", "022.png", "023.png", "024.png", "025.png", 
                #  25         26         27         28         29
               "026.png", "027.png", "028.png", "029.png", "030.png", 
                #  30         31         32         33         34
               "031.png", "032.png", "033.png", "034.png", "035.png", 
                #  35         36         37         38         39
               "036.png", "037.png", "038.png", "039.png", "040.png", 
                #  40         41         42         43         44
               "041.png", "042.png", "043.png", "044.png", "045.png", 
                #  45         46         47         48         49
               "046.png", "047.png", "048.png", "049.png", "050.png", 
                #  50         51         52         53         54
               "051.png", "052.png", "053.png", "054.png", "055.png",
                #  55         56         57         58         59
               "056.png", "057.png", "058.png", "059.png", "060.png", 
                #  60
               "061.png", ]
    n = len(PNGList)
    # for i in range(0, n-1):
    
    # 013.png ~ 027.png : reflection between white line and green line
    # 028.png ~ 030.png : no reflection
    # 031.png ~ 042.png : reflection between white line and green line

    i = input('Please give the number of png: ')
    i = int(i)# 19
    print("the amount of PNGList: ", PNGList[i])
    img_cv = cv2.imread(PNGList[i])
    # print("\timported image", i, ": ", PNGList[i], '\n')
    #img_msg = cd.cv2_to_imgs2mg(cv_image)
    #img = cd.imgmsg_to_cv2(img_msg)
    height, width, _ = img_cv.shape
    print('height = ', height)
    print('width = ', width)

    i_bot = height - 5
    i_mid = height - 50
    width_n = width - 250

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

    # print(cd.ColDet(img, height, width, 0, 165, 'Green'))

    Wminhigt = DeterHeight1(img, height, width, 0, height, "White", "min")    
    Gminhigt = DeterHeight1(img, height, width, 0, height, "Green" or "Yellow", "min")
    Wmaxhigt = DeterHeight1(img, height, width, height-1, -1, "White", "max")
    Gmaxhigt = DeterHeight1(img, height, width, height-1, -1, "Green" or "Yellow", "max")

    if(Wminhigt == Wmaxhigt):
        Wminhigt = Gminhigt
        Wmaxhigt = Gminhigt
    elif(Gminhigt == Gmaxhigt):
        Gminhigt = Wminhigt
        Gmaxhigt = Wminhigt
    print("Wminhigt = ", Wminhigt, "\t\tGminhigt = ", Gminhigt)
    print("Wmaxhigt = ", Wmaxhigt, "\t\tGmaxhigt = ", Gmaxhigt)

    # Wminhigt, Wmaxhigt = DeterHeight2(Wminhigt, Wmaxhigt)
    # Gminhigt, Gmaxhigt = DeterHeight2(Gminhigt, Gmaxhigt)
    # print("Wminhigt = ", Wminhigt, "\t\tGminhigt = ", Gminhigt)
    # print("Wmaxhigt = ", Wmaxhigt, "\t\tGmaxhigt = ", Gmaxhigt)

    # start = time.time()
    # A = cd.ColDet(img, height, width, 116, 170, "White")
    # print('White Line : ', A)
    # B = cd.ColDet(img, height, width, 116, 170, "Green" or "Yellow")
    # print('Green Line : ', B)
    # end = time.time()
    # print("to calculate min, max heigt: ", end-start)

    # C = cd.ColDet(img, height, width, 103, "White")
    # print(C)
    # for i in range(len(C)):
    #     Pnt = C[i]
    #     cv2.circle(img, (Pnt[1], Pnt[0]), 1, (255, 0, 0), 3)

    start = time.time()

    Line = cd.LineForm(img, height, width, "White", "Green" or "Yellow", Wminhigt, Wmaxhigt, Gminhigt, Gmaxhigt)

    end = time.time()
    print(end-start)

    # print(Line, "\tlen(Line)", len(Line))

    for i in range (len(Line)):
        Pnt = Line[i]
        cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 3)
    # for i in range max(Wmaxhigt, Gmaxhigt):
    #     if(np.all(Line[i][0] == -100)):
    #     print("type of Line : ", type(Line), "len(Line) = ", len(Line))
    #     for i in range(len(Line)):
    #         Pnt = Line[i]
    #         # Pnt1 = WLine[i]
    #         # Pnt2 = GLine[i]
    #         print('i = ', i, '; Pos = ', Pnt)
    #         cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 3)
            # cv2.circle(img, (101, 238), 5, (255, 255, 0), 1)
            # cv2.circle(img, (99, 179), 5, (255, 255, 0), 1)
            # cv2.circle(img, (Pnt1[0], Pnt1[1]), 1, (125, 255, 0), 3)
            # cv2.circle(img, (Pnt2[0], Pnt2[1]), 1, (0, 175, 175), 3)

    LoC = cd.LineorCurve(Line)
    print("-------------- LoC = ", LoC, " --------------")
    slope = cd.LineSlope(Line, LoC)
    print("------------ slope = ", slope, " ------------")
    DrawTangent(Line, img, slope)

    # LoC = cd.LineorCurve(Line)
    # print('LoC = ' , LoC)
    # slope = cd.LineSlope(Line, LoC)
    # # if(LoC == 0):
    # #     # DrawTangent(WLine, img, "bot")
    # #     # DrawTangent(WLine, img, "line")
    # #     print('slope = ', slope)
    # # elif(LoC == 1):
    # #     # DrawTangent(WLine, img, "bot")
    # #     # DrawTangent(WLine, img, "line")
    # print('slope = ', slope)
    

    # cv2.imshow("IMG_CV", img_cv)
    cv2.imshow("IMG", img)
    # cv2.imshow("GIMG", Gimg)
    # cv2.imshow("IMG_HSV", img_hsv)
    cv2.waitKey(0)
    

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
