# -*- coding: utf-8 -*-
"""
Created on Fri Jan  7 12:29:17 2022

@author: pauls
Outputs a 3d Array (step,x,y) Matte with interpolated outlines between two Matte Arrays of same Size. 
Input (steps, Matte1st, Matte2nd)

"""
import cv2
import numpy as np
import mp4_to_np

def main():
    matte1=np.zeros((100,100))
    matte1[40:60,40:60]=np.ones((20,20))
    
    matte2=np.zeros((100,100))
    matte2[20:80,20:80]=np.ones((60,60))
    shape=np.shape(matte1)
    
    
    showImgAll(np.hstack((matte1,np.ones((shape[0],1)),matte2)))
        
    outline1=getOutlinePoints(matte1)
    outline2=getOutlinePoints(matte2)
    outline1XY=getXYCoordsFromPixels(outline1,0)
    outline2XY=getXYCoordsFromPixels(outline2,0)
    
    showImgAll(outline1)
    showImgAll(outline2)

    
    steps=2

   
    if len(outline1XY)>= len(outline2XY):
        interpolatedOutline, intXY=shapeInt(steps,outline1XY,outline2XY,shape)
    else:
        interpolatedOutline, intXY =shapeInt(steps,outline2XY,outline1XY,shape)
        intXY=np.flipud(intXY)
        interpolatedOutline=np.flipud(interpolatedOutline)

    for k in range(0,steps):
        showImgAll(interpolatedOutline[k,:,:])
    
            
    showImgAll(fillOutline(intXY[1],shape))
    
    return outline1XY

def fillOutline(outlineXY,shape):
    filledOutline=np.zeros(shape)
    filledOutline[outlineXY[:,0],outlineXY[:,1]]=1

    outlineXY=outlineXY[np.argsort(outlineXY[:, 1])]
    filledMatte=np.zeros(shape)
    for x in range (np.min(outlineXY[:,1]),np.max(outlineXY[:,1])):
        ysort=np.sort(outlineXY[:,0][outlineXY[:,1]==x])
        ylow=ysort[0]
        y1=ylow
        fill=True
        first=True
        concurrent=False

        for t,y in enumerate(ysort): 
            last=not t+1<len(ysort)
            
            if last or ysort[t+1]-1>y:
                if concurrent:
                    concurrent=False
                    first=True
                    if fill:
                        filledMatte[ylow+1:y,x]=1
                        fill=False
                    else:
                        fill=True
                
                
                if fill:
                    filledMatte[y+1:ysort[t+1],x]=1
                    fill=False
                else:
                    fill=True
            else:
                if first:
                    ylow=y
                    concurrent=True
                    first=False
                
    filledMatte[filledOutline==1]=0
    return filledMatte
               

def shapeInt(steps,matte1YX,matte2YX,shape):
        nearestPoints=np.empty((steps,np.shape(matte1YX)[0],np.shape(matte1YX)[1]))
        nearestPoints[0,:,:]=matte1YX
        outputMattes=np.zeros((steps,shape[0],shape[1]))
        #Find nearestPoints to Points and add to list of same size as Points
        for t,coords in enumerate(matte1YX):
            y=coords[0]
            x=coords[1]
            distance=(matte2YX[:,0]-y)*(matte2YX[:,0]-y)+(matte2YX[:,1]-x)*(matte2YX[:,1]-x)
            idx = np.argmin(distance)  
            nearestPoints[-1,t,:]=matte2YX[idx]
        
        
        #Interpolate Coords and filld outPoutMattes (outlines)
        for step in range(0,steps):
            yint=np.round((nearestPoints[-1,:,0]-matte1YX[:,0])/steps*step+matte1YX[:,0],0).astype(int)
            xint=np.round((nearestPoints[-1,:,1]-matte1YX[:,1])/steps*step+matte1YX[:,1],0).astype(int)
            nearestPoints[step,:,:]=np.stack((yint,xint),axis=1)
            outputMattes[step,yint[:],xint[:]]=1
        
        return outputMattes, nearestPoints.astype(int)
            
def getXYCoordsFromPixels(image, schwelle):
      result=np.where(image>schwelle)
      listOfCoordinates= list(zip(result[0], result[1]))
      return np.asarray(listOfCoordinates)   
    
    
    

def getOutlinePoints(image):
    imageBG=np.zeros((np.shape(image)[0]+2,np.shape(image)[0]+2))
    imageBG[1:-1,1:-1]=image
    stackDiff=np.stack((rollDif(imageBG,0,1),rollDif(imageBG,0,-1),rollDif(imageBG,1,1),rollDif(imageBG,1,-1)),axis=0)
    stackDiff= np.squeeze(np.max(stackDiff[:,1:-1,1:-1],axis=0))
    return stackDiff


def rollDif(image,ax,shift):
    difImg=np.abs(np.subtract(image,np.roll(image,shift,axis=ax),))
    difImg=np.roll(difImg,-shift,axis=ax)
    difImg[image>0]=0
    return difImg
    

def showImgAll(image):
           image=image*255/np.max(image)
           cv2.imshow('frame ', image.astype(np.uint8))
           c=cv2.waitKey(0)
           cv2.destroyAllWindows()

if __name__ == "__main__":
    # fillOutline(outlineXY,shape)
    OutlineXY=main()
    1+1




           
