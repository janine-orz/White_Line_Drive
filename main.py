import sys
import cv2
import numpy as np
import color_detector_004 as cd
import color_det_part1 as cd1
import color_det_hsv_red as cd2
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


def main():
    PNGList = ["001.png", "002.png", "003.png", "004.png", "005.png", "006.png", 
               "011.png", "012.png", "014.png", "015.png", "017.png", "019.png", 
               "020.png", "021.png", "026.png", "027.png", "028.png", "029.png", 
               "030.png", "033.png", "034.png", "035.png"]
    n = len(PNGList)
    print("the amount of PNGList: ", n)
    # for i in range(0, n-1):
    i = 18
    img_cv = cv2.imread(PNGList[i])
    print("\timported image", i, ": ", PNGList[i], '\n')
    #img_msg = cd.cv2_to_imgs2mg(cv_image)
    #img = cd.imgmsg_to_cv2(img_msg)
    height, width, _ = img_cv.shape
    print('height = ', height)
    print('width = ', width)

    i_bot = height - 5
    i_mid = height - 40
    width_n = width - 260
    i = width_n

    out_pnt = np.float32([[0, 0],
                        [0, height-1],
                        [width-1, height-1],
                        [width-1, 0]])
    #print(out_pnt)
    inp_pnt = np.float32([[width_n + 58, i_mid],
                        [width_n + 8, i_bot],
                        [width-1, i_bot],
                        [300, i_mid]])
    #print(inp_pnt)
    maxw = width - 1 - width_n
    maxh = height - 1 - i_mid
    M = cv2.getPerspectiveTransform(inp_pnt, out_pnt)
    img = cv2.warpPerspective(img_cv, M, (width, height), flags = cv2.INTER_LINEAR)
    img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

    #height, width, _ = img.shape
    #print('height = ', height)
    #print('width = ', width)

    minwid = 0
    maxwid = height

    # Line = cd.LinePos(img, height, width, 150, 0)
    # print(len(Line))

    # if(Line != []):
    #     i = 0
    #     for i in range(len(Line)):
    #         Pnt = Line[i]
    #         # print(Pnt)
    #         cv2.circle(img, (Pnt[1], Pnt[0]), 1, (0, 255, 0), 3)

    # Liner = cd2.HSV_Red_Det(img_hsv, height, width, 0, 0)

    # if(Liner != []):
    #     i = 0
    #     for i in range(len(Liner)):
    #         Pntr = Liner[i]
    #         # print(Pntr)
    #         cv2.circle(img_hsv, (Pntr[1], Pntr[0]), 1, (255, 0, 0), 3)

    Line_n = cd.LineForm(img, height, width, minwid, maxwid)
    cd1.checkagain(img, Line_n, height, width)

    for i in range(len(Line_n)):
        Pnt = Line_n[i]
        print('i = ', i, '; Pos = ', Pnt)
        cv2.circle(img, (Pnt[0], Pnt[1]), 1, (255, 0, 0), 5)

    # # Liner_n = cd2.LinerForm(img_hsv, height, width, minwid, maxwid)
    # cd1.checkagain(img, Line_n, height, width)

    # for i in range(len(Liner_n)):
    #     Pnt = Line_n[i]
    #     # print('i = ', i, '; pos: ', Pnt)
    #     cv2.circle(img_hsv, (Pnt[0], Pnt[1]), 1, (0, 0, 255), 5)

    #print_line(Line, img)
    #LoC = cd.LineorCurve(Line_n)
    #print('LoC = ', LoC)
    #print('slope = ', cd.LineSlope(Line_n, LoC))
    # 004.png: 0.6514532274507854
    # 003.png: 0.9629943306809362
    # 001.png:

    # cv2.imshow("IMG_CV", img_cv)
    cv2.imshow("IMG", img)
    # cv2.imshow("IMG_HSV", img_hsv)
    cv2.waitKey(0)

    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
