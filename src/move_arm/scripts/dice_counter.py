#!/usr/bin/env python
import os
import numpy as np
import cv2
import matplotlib.pyplot as plt

IMG_DIR = r'/home/cc/ee106a/fa22/class/ee106a-acn/ros_workspaces/final_project/src/move_arm/img'

def dice_counter():
    image_location = IMG_DIR + '/dice_image.jpg'
    image = cv2.imread(image_location)
    image_blurred = cv2.blur(image, (3,3))
    img_gray = cv2.cvtColor(image_blurred, cv2.COLOR_BGR2GRAY)
    ret, thresh = cv2.threshold(img_gray, 130, 255, cv2.THRESH_BINARY)
    contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
                                        
    # draw contours on the original image
    image_copy = image.copy()
    #cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)


    min_dice_area = 1000
    max_dice_area = 7000
    dice_locations = []
    cnts = contours
    for i in range(len(contours)):
        rect = cv2.minAreaRect(cnts[i])
        box = np.int0(cv2.boxPoints(rect))
        #print(rect)
        #print(rect[1][0]*rect[1][1])
        '''if min_dot_area < rect[1][0]*rect[1][1] < max_dot_area:
            #print("dot", i, rect[1][0]*rect[1][1])
            cv2.drawContours(image_copy, [box], 0, (36,255,12), 3)
            dot_total += 1'''

        if min_dice_area < rect[1][0]*rect[1][1] < max_dice_area:
            dice_locations.append(rect)
        #cv2.drawContours(image_copy, [box], 0, (36,255,12), 3)

    # cv2.polylines(image, [box], True, (36,255,12), 3)
    print(dice_locations)
    dot_total = 0 

    for i in dice_locations:
        rect = i

        rotation = cv2.getRotationMatrix2D(rect[0], rect[2], 1.0)
        new_image = cv2.warpAffine(src=img_gray, M=rotation, dsize=np.shape(img_gray))

        smaller_img = cv2.getRectSubPix(new_image, (int(rect[1][0]), int(rect[1][1])), rect[0])
        image_blurred = cv2.blur(smaller_img, (5,5))

        ret, thresh = cv2.threshold(image_blurred, 210, 255, cv2.THRESH_BINARY)
        # detect the contours on the binary image using cv2.CHAIN_APPROX_NONE
        contours, hierarchy = cv2.findContours(image=thresh, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
                                            
        # draw contours on the original image
        image_copy = smaller_img.copy()
        #cv2.drawContours(image=image_copy, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2, lineType=cv2.LINE_AA)

        min_dot_area = 20
        max_dot_area = 121
        dice_locations = []
        cnts = contours
        for i in range(len(contours)):
            rect = cv2.minAreaRect(cnts[i])
            box = np.int0(cv2.boxPoints(rect))
            print(rect)
            #print(rect[1][0]*rect[1][1])
            if min_dot_area < rect[1][0]*rect[1][1] < max_dot_area:
                #print("dot", i, rect[1][0]*rect[1][1])
                cv2.drawContours(image_copy, [box], 0, (36,255,12), 3)
                dot_total += 1


            #cv2.drawContours(image_copy, [box], 0, (36,255,12), 3)
    print(dot_total)
if  __name__ == '__main__':
    dice_counter()
