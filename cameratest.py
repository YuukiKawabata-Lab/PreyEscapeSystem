# -*- coding: utf-8 -*-
"""
Created on Mon Aug 10 14:17:37 2020

@author: earth
"""


import tisgrabber as IC
import cv2

def uvcRead(Camera):
    Camera.SnapImage()
    image:list = Camera.GetImage()
    image:list = cv2.flip(image,0)
    
    return image

Camera = IC.TIS_CAM()

Camera.open("DMK 33UX287 25020344")
# Set a video format
Camera.SetVideoFormat("Y800 (640x480)")
#Set a frame rate of 30 frames per second
Camera.SetFrameRate(500)


#Camera.ShowDeviceSelectionDialog()

Camera.StartLive(0)  
Camera.SnapImage()
image = Camera.GetImage()


while(1):
    # Camera.SnapImage()

    # image = Camera.GetImage()
    # image = cv2.flip(image,0)
    image=uvcRead(Camera)
    image=cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    info=tuple(Camera.GetImageDescription())
    print(info[1])
    cv2.imshow("Frame",image)
    key=cv2.waitKey(1)
    if(key==ord("q")):
        break

cv2.destroyAllWindows()
Camera.StopLive()
