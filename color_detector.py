import cv2
import numpy as np

#def img_read():
img = cv2.imread("004.png")
height, width, _ = img.shape
    #hsv_img = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)
#    return img, height, width

def ColDet(img, height, width):
    color = "Undefined"
    white = []
    i = height - 5
    for j in range(width):
        pixel_center = img[i, j]
        b_val = pixel_center[0]
        g_val = pixel_center[1]
        r_val = pixel_center[2]
        if b_val > 180 and g_val > 180 and r_val > 180:
        #sucess to find the white color
            temp = [i, j]
            white.append(temp)
    print(white)
    #sucess to finde the white color in one line
    length = len(white)
    if white != []:
        white_min = white[0]
        cy_min = white_min[1]
        for k in range(length-1):
            white_temp1 = white[k+1]
            if cy_min < white_temp1[1]:
                cy_min = cy_min
            elif cy_min > white_temp1[1]:
                cy_min = white_temp1[1]
        print(cy_min)
    #sucess to finde the left white point
        white_max = white[length - 1]
        cy_max = white_max[1]
        for k in range(length-1):
            white_temp1 = white[k+1]
            if cy_max > white_temp1[1]:
                cy_max = cy_max
            elif cy_max < white_temp1[1]:
                cy_max = white_temp1[1]
        print(cy_max)
    #sucess to finde the right white point

    #print (cx_max, cy_max)
    #cx = int((cx_max - cx_min)/2)
    #cy = int((cy_max - cy_min)/2)
    #pixel_center = img[:,i]
    return cy_max, cy_min, i
''''''
cy_max = cx = ColDet(img, height, width)[0]
cy_min = cx = ColDet(img, height, width)[1]
cx = ColDet(img, height, width)[2]
cv2.circle(img, (cy_min, cx), 3, (255, 0, 0), 3)
cv2.circle(img, (cy_max, cx), 3, (255, 0, 0), 3)
''''''
width = width - 1
cv2.imshow("003", img)
cv2.waitKey(0)

cv2.destroyAllWindows()
