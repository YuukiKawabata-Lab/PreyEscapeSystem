import cv2
import numpy as np
import time
import serial
import tisgrabber as IC

#=== Program settings ===

# True: Using a high speed camera, False: Using a normal USB camera
HighSpeedCam:bool=True

# *** Settings of DMK33UX287 high speed camera (The Imaging Source Co., Ltd.) ***
# Provide a serial number of high speed camera
# UVC:str="Serial number"
# Set a video format of high speed camera
videoFormat:str="Y800 (640x480)"
# *** End of high speed camera settings ***

# *** Normal USB camera settings ***
# Set a camera number
camNum:int = 0
# *** End of Normal USB camera settings *** 

# *** Image processing settings ***
# Set a maximum target frame rate 
camFPS:int = 500 
# True: Show a real time frame rate 
printDetail:bool = False
# Center points of the search circle and reaction circle 
roiX:int = 285
roiY:int = 250
# Diameter of the search circle
roiRad:int = 165
# Diameter of the initial pray position
roiRadInner:int=7
# Diameter of the reaction circle (reaction distance)
disThresh:int = 43
# Binary thresholding parameter 
binalyThresh:int = 65
# *** End of image processing settings ***

# *** Arduino settings ***
# Select a COM board connecting Arduino
board:str = 'COM5'
# Serial communication speed
serialSpeed:int = 115200

#=== End of program settings ===


def main():
    # Capture camera frame
    if(HighSpeedCam):
        # High speed camera
        Camera = IC.TIS_CAM()

        Camera.open(UVC)
        # Set a video format
        Camera.SetVideoFormat(videoFormat)
        # Set a frame rate of 500 frames per second
        Camera.SetFrameRate(camFPS)
        Camera.StartLive(0)
    else:
        video1 = cv2.VideoCapture(camNum)
        
        # Normal USB camera
        video1.set(cv2.CAP_PROP_FPS, camFPS)
        video1.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
        video1.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        
    # import class
    Processing = autoTrigger(Camera,5)
    SerialCom = serialtrigger(board, serialSpeed)
    # Capture camera frame
    if(HighSpeedCam):
        frame = uvcRead(Camera)
    else:
        retval, frame = video1.read()
    
    if(True):
        loop=[0]
        append=loop.append
        waitKey=cv2.waitKey
        for i in loop:
            append(i+1)
        # while(True):
            timeStart:float = time.time() 
            
            # Process image
            ProcessedFrame=Processing.multiChannel2binary(frame, binalyThresh)
            ProcessedFrame=Processing.openingOrClosing(ProcessedFrame,False)
            ProcessedFrame=cv2.bitwise_not(ProcessedFrame)
            
            ProcessedFrame=Processing.roiMask(ProcessedFrame,roiX,roiY,roiRad,roiRadInner)
            
            posxy,frame=Processing.contoursFromBinaly(ProcessedFrame, frame)
            
            #ProcessedFrame=SerialCom.distanceTrigger(roiX, roiY, disThresh, posxy, frame)
            SerialCom.distanceTrigger(roiX, roiY, disThresh, posxy, frame)
            
            # Draw ROI
            frame = cv2.circle(frame, (roiX,roiY), roiRad, (0,255,255),3)
            frame = cv2.circle(frame, (roiX,roiY), roiRadInner, (0,255,255),-1)
            
            cv2.namedWindow("Processed",cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Processed",640,480)
            #cv2.resizeWindow("Processed",960,540)
            cv2.imshow("Processed",ProcessedFrame)
            
            cv2.namedWindow("Native",cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Native",640,480)
            #cv2.resizeWindow("Native",960,540)
            cv2.imshow("Native",frame)

            # key input
            key = waitKey(1)
            
            SerialCom.switchTrigger(key)
    
            # Stop update frame
            if(key==ord('q')):
                break
            #Processing.printData(timeStart,False)
    
            # Update frame
            if(HighSpeedCam):
                frame=uvcRead(Camera)
            else:
                retval,frame = video1.read()
                if(retval==False):
                    break

            # Display frame rate info
            Processing.printData(timeStart, printDetail)

    cv2.destroyAllWindows()
    if(HighSpeedCam):
        Camera.StopLive()
    else:
        video1.release()
    
    


def imageProcessing(frame :list):
    if(len(frame[1,1,:])==3):
        imageB,imageG,imageR = cv2.split(frame)
        imageGray:list = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    
        windowName:list = ["Blue","Green","Red","Gray"] 
    
        for i in windowName:
            cv2.namedWindow(i,cv2.WINDOW_NORMAL)
            cv2.resizeWindow(i,640,480)
                
        cv2.imshow("Blue",imageB)
        cv2.imshow("Green",imageG)
        cv2.imshow("Red",imageR)
        cv2.imshow("Gray",imageGray)
                
        return imageB, imageG, imageR, imageGray
    
    else:
        cv2.imshow("Gray?",frame)
        return frame
    
    
        
class autoTrigger:
    def __init__(self, camera, kernelSize :int):
        self.camera=camera
        self.uvcInfo:tuple=tuple(camera.GetImageDescription())
        # Average Image
        self.accum_frame :list = np.zeros((self.uvcInfo[1],self.uvcInfo[0]),np.float32)
        self.maskInitial = self.accum_frame.copy()
        self.maskInitial2 = self.accum_frame.copy()
        self.accumulateWeighted = cv2.accumulateWeighted
        self.float32 = np.float32
        self.kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(kernelSize,kernelSize))
                        
    def accum(self, frame :list, wight :float = 0.2, colorScale :bool = False):
        if(len(frame[1,1,:])==3):
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        f_frame :list= frame.astype(self.float32)
        self.accumulateWeighted(f_frame, self.accum_frame, wight)
        diff_frame :list= cv2.absdiff(f_frame, self.accum_frame)
        diff_frame:list = diff_frame.astype(np.uint8)
        return diff_frame

    def printData(self,time_now :float,movieVal:bool=True):
        timeProcess :float = time.time() - time_now
        if(movieVal):
            if(timeProcess==0):
                print("Est: NA fps, Processing: {:.3f} s, {}×{}".format(timeProcess,self.uvcInfo[0],self.uvcInfo[1]))
            else:
                print("Est: {:.1f} fps, Processing: {:.3f} s, {}×{}".format(1/timeProcess,timeProcess,self.uvcInfo[0],self.uvcInfo[1]))
        else:
            if(timeProcess==0):
                print("Est: NA fps")
            else:
                print("Est: {:.1f} fps".format(1/timeProcess))
            
    def multiChannel2binary(self,frame :list,val :int):
        if(len(frame[1,1,:])==3):
            imageGray:list = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        ret,binalyImage = cv2.threshold(imageGray,val,255,cv2.THRESH_BINARY)
        return binalyImage

    def openingOrClosing(self,binalyImage :list,opening :bool = True, rep :int = 1):
        if(opening):
            for i in range(rep):
                binalyImage:list = cv2.morphologyEx(binalyImage, cv2.MORPH_OPEN, self.kernel)
        else:
            for i in range(rep):
                binalyImage:list = cv2.morphologyEx(binalyImage, cv2.MORPH_CLOSE, self.kernel)
        return binalyImage
    
    def roiMask(self,Processedframe:list,x:int,y:int,radius:int,radius2:int):
        mask:list = self.maskInitial
        mask2:list = self.maskInitial2
        cv2.circle(mask, center=(x,y), radius=radius, color=255, thickness=-1)
        cv2.circle(mask2, center=(x,y), radius=radius2, color=255, thickness=-1)
        #maskedFrame = Processedframe.astype(np.float32)
        maskedFrame = Processedframe.astype(np.uint8)
        maskedFrame[mask==0]=[0]
        maskedFrame[mask2==255]=[0]
        
        return maskedFrame    
    def contoursFromBinaly(self,frameBinaly:list,frameNative:list,drawContour:bool=False):
        _, contours, hierarchy = cv2.findContours(frameBinaly,cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
        maxArea:int = 0
        cntMaxarea:int = 0
        for i in range(len(contours)):
            area = cv2.contourArea(contours[i])
            if(maxArea<area):
                maxArea=area
                cntMaxarea = i
                
        #e=cv2.fitEllipse(cnt)
        if(len(contours)>0):
            cnt:list = contours[cntMaxarea]
            topmost:tuple = tuple(cnt[cnt[:,:,1].argmin()][0])
        else:
            topmost:tuple = None

        # if(drawContour):
        #     frameNative = cv2.drawContours(frameNative, contours, cntMaxarea, (255,0,0), 3)
        frameNative = cv2.circle(frameNative, topmost, 5, (0,255,0),-1)
        
        return topmost,frameNative
        
    
class serialtrigger:
    def __init__(self, board :str, speed :int=11520):
        self.com = serial.Serial(board,speed)
        #self.com.rts = False
        self.trigger_flag = False
        self.ex_flag = False
        #self.serbo_flag = False
        
    def distanceTrigger(self,targetX:int,targetY:int,distanceThresh:int,trakingPoint:tuple,frame:list):
        
        if self.ex_flag and not(self.trigger_flag):
                frame = cv2.circle(frame, (targetX,targetY), distanceThresh, (255,0,0),3)
        
        if trakingPoint != None:
            if np.linalg.norm(np.array(trakingPoint)-np.array([targetX,targetY]))<distanceThresh:
                self.trigger_flag = True
                        
        if self.ex_flag and self.trigger_flag:
            self.com.write(str.encode('a'))                
            #print("Triggered")
            frame = cv2.circle(frame, (targetX,targetY), distanceThresh, (0,0,255),3)
                
        return frame

    def switchTrigger(self,key:int):
        if(key==ord('e')): # Start Experiment
            self.ex_flag = True
            self.trigger_flag = False
            
        if(key==ord('n')): # Trigger off
            self.ex_flag = False
            self.trigger_flag = False
            self.com.write(str.encode('b'))
        if(key==ord('q')): # Stop Program
             self.com.write(str.encode('b')) 
             self.com.close()

def uvcRead(Camera):
    Camera.SnapImage()
    image:list = Camera.GetImage()
    image:list = cv2.flip(image,0)
    return image

if __name__ == '__main__':
    main()

