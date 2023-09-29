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


def DeterHeight1(img, height, width, idx1, idx2, string1, string2):
    if(idx1 <= idx2):
        a = 1
    elif(idx1 > idx2):
        a = -1
    for i in range(idx1, idx2, a):
    # for idx1 < idx2: Check from upper to bottom, if there is "string" points
    # for idx1 > idx2: Check from bottom to upper, if there is "string" points
        J = np.zeros((2), dtype = 'int')
        BGR = [170, 170, 160]
        HSV = [100, 100]
        J = (cd.ColDet(img, height, width, i, BGR, HSV, string1, J))
        # if(string1 == "White"):
        # U = (J[0] == 0)
        # elif(string1 == "Green"):
        U = (J[0] == 0)
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
    # print("len(Line) = ", len(Line))
    idx1 = 10

    Pnt0 = Line[idx1]
    cv2.circle(img, (Pnt0[0], Pnt0[1]), 2, (255, 0, 255), 5)
    Pnt1 = Line[l-1]
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


def PrintLine(Line, img):
    rtl = 0 # from bot to top : right to left
    ltr = 0 # from bot to top : left to right
    D_L = []
    D_R = []
    # 1. check the direction on the line 
    ltr, D_L, rtl, D_R = Chec_dir(Line, ltr, D_L, rtl, D_R)
    
    print("left_to_right = ", ltr, "\tright_to_left = ", rtl)
    # 2. show the point that is out of the direction
    if(ltr > rtl):
        direc = "LTR"
        diff = (sum(D_L)) / ltr
    elif(rtl >= ltr):
        direc = "RTL"
        diff = (sum(D_R)) / rtl
    
    print("direction : ", direc)
    # 3. correct these points
    if(direc == "RTL"):
        Line = Corr_Pnt(Line, rtl, ltr, diff, rtl)
    elif(direc == "LTR"):
        Line = Corr_Pnt(Line, rtl, ltr, diff, ltr)
    for i in range(len(Line)):
        Pnt = Line[i]
        cv2.circle(img, (Pnt[0], Pnt[1]), 1, (0, 125, 225), 3)

    return Line


def Chec_dir(Line, ltr, D_L, rtl, D_R):
    for i in range(len(Line)-1):
        Pnt1 = Line[i]
        Pnt2 = Line[i+1]
        if(Pnt2[0] <= Pnt1[0]):
            ltr = ltr + 1
            diff = (Pnt2[0]-Pnt1[0])
            D_L.append(diff)
        if(Pnt2[0] >= Pnt1[0]):
            rtl = rtl + 1
            diff = (Pnt2[0]-Pnt1[0])
            D_R.append(diff)
    return ltr, D_L, rtl, D_R


def Corr_Pnt(Line, rtl, ltr, diff, aa):
    while(aa < 239):
        for i in range(len(Line)-1, 1, -1):
            Pnt1 = Line[i]
            Pnt2 = Line[i-1]
            if(Pnt2[0] > Pnt1[0]):
                Pnt2[0] = Pnt1[0] + diff
                Pnt2[0] = (int)(Pnt2[0])
                rtl = rtl + 1
                aa = rtl
            elif(Pnt2[0] < Pnt1[0]):
                Pnt2[0] = Pnt1[0] - diff
                Pnt2[0] = (int)(Pnt2[0])
                ltr = ltr + 1
                aa = ltr
    return Line



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
                #  60         61         62         63         64
               "061.png", "062.png", "063.png", "064.png", "065.png",
                #  65         66         67         68         69
               "066.png", "067.png", "068.png", "069.png", "070.png",  
                #  70         71         72         73         74
               "071.png", "072.png", "073.png", "074.png", "075.png",
                #  75         76         77         78         79
               "076.png", "077.png", "078.png", "079.png", "080.png", 
                #  80         81         82         83         84
               "081.png", "082.png", "083.png", "084.png", "085.png",
                #  85         86         87         88         89
               "086.png", "087.png", "088.png", "089.png", "090.png", 
               ]
    n = len(PNGList)
    # for i in range(0, n-1):
    
    # 013.png ~ 027.png : reflection between white line and green line
    # 028.png ~ 030.png : no reflection
    # 031.png ~ 042.png : reflection between white line and green line

    i = input('Please give the number of png: ')
    i = (int(i)) - 1 # 19
    print("the amount of PNGList: ", PNGList[i])
    img_cv = cv2.imread(PNGList[i])
    # print("\timported image", i, ": ", PNGList[i], '\n')
    #img_msg = cd.cv2_to_imgs2mg(cv_image)
    #img = cd.imgmsg_to_cv2(img_msg)
    height, width, _ = img_cv.shape
    print('height = ', height)
    print('width = ', width)

    i_bot = height - 1
    i_mid = height - 20

    out_pnt = np.float32([[0,        0       ],
                          [0,        height-1],
                          [width-1,  height-1],
                          [width-1,  0       ]])
    inp_pnt = np.float32([[15,       i_mid],    # A         10 -> slope:0.02192631009880022
                                                #           20 -> slope:0.035073330533225366
                          [0,        i_bot],    # B
                          [width-4,  i_bot],    # C
                          [width-20, i_mid]])   # D     wid-40 -> slope:0.03507333053322537
                                                #       wid-20 -> slope:0.10969010348702159

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

    start = time.time()

    Line = cd.LineForm(img, height, width, "White", "Green" or "Yellow", Wminhigt, Wmaxhigt, Gminhigt, Gmaxhigt)

    end = time.time()
    print("Time for cd.LineForm() : ", end-start)

    # print(Line, "\tlen(Line)", len(Line))

    # Line = PrintLine(Line, img)
    
    # for i in range max(Wmaxhigt, Gmaxhigt):
        # if(np.all(Line[i][0] == -100)):
        # print("type of Line : ", type(Line), "len(Line) = ", len(Line))
    for i in range(len(Line)):
        Pnt = Line[i]
        # Pnt1 = WLine[i]
        # Pnt2 = GLine[i]
        print(Pnt)
        cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 3)
            # cv2.circle(img, (101, 238), 5, (255, 255, 0), 1)
            # cv2.circle(img, (99, 179), 5, (255, 255, 0), 1)
            # cv2.circle(img, (Pnt1[0], Pnt1[1]), 1, (125, 255, 0), 3)
            # cv2.circle(img, (Pnt2[0], Pnt2[1]), 1, (0, 175, 175), 3)

    LoC = cd.LineorCurve(Line, width, img)
    print("-------------- LoC = ", LoC, " --------------")
    
    slope = 0.00
    slope_n = cd.LineSlope(Line, width, LoC)
    slope_do = slope_n
    
    #########################
    # Fix the slope problem #
    #########################

    # if(slope_n > 0.03):
    #     slope = slope_n - 0.0078
    # elif(slope_n < -0.01):
    #     slope = slope_n + 0.0052
    DrawTangent(Line, img, slope)
    print("-------------  slope_n = %.2f", slope_n, "  -------------")
    print("------------- slope_do = %.2f", slope_do, "  -------------")
    print("-------------    slope = %.2f", slope, "  -------------")

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
