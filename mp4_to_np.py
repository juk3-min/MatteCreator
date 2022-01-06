import numpy as np
import cv2
import concaveHull
from skimage.transform import resize
from skimage.transform import rotate
from skimage import util
from skimage.color import rgb2gray
import image_functions
from functools import partial
from scipy.ndimage import gaussian_filter as gauss
import scipy.ndimage as ndimage

# Starts searching for an outline from the defined seed as y,x Pixel coordinate. An outline (1=perfect´white)
# must have at least mitOutlineSize as size. Size measurement depends on the chosen shape default is cube.
# Size->width, other shapes could be circle. Size->Diameter. The searchpattern describes how the outline is
# searched for from the seed. Default is anti-clockwork spiral outwards("acso"). maxGap describes the gap in pixels
# between parts of the outline which are still considered continous. First the direction of the previous
# outline is continued. If no outline is found the search adds an increasing angle up to maxAngle until an outline
# or none are foudn. Imageborder is considered an outline
# the outline is filled with 1s afterwards

def test_func(values):
     return values.sum()/100
   

def getXYCoordsFromPixels(image, schwelle):
      result=np.where(image>=schwelle)
      listOfCoordinates= list(zip(result[0], result[1]))
      return np.asarray(listOfCoordinates)
      
def main():
    #Makes numpy Arrays from video
    buf=giveImgsFromVid('small.mp4')
    
    buf_gray=rgb2gray(buf)
    
    #Pictures to DxDy with gray scaling included
    buf_dx=(255*giveDxDxDiff(buf,5,5)).astype(np.uint8)
        
    #buf gray to schwellwert
    buf_dx=np.array(image_functions.schwellwert(buf_dx,10), dtype=np.uint8)
    
    #Loop to check noise Schwelle
    for i in range(4,5):
        noiseSchwelle=i*1
        #Intitialize the dt array with temporal difference between pictures
        dtPic=np.abs(np.diff((buf_gray[50],buf_gray[51]),axis=0))
        dtPic=np.squeeze((dtPic*255).astype(np.uint8))
        
        #Loop to regulate how dt Differences interact
        #Idea, minimal differences lead to 0 over the whole searched time --> Noise has to be canceled with NoiseMask
        noiseMask=np.zeros(np.shape(dtPic)).astype(bool)
        fillMask=np.zeros(np.shape(dtPic)).astype(bool)
        for i in range(49,53):
            dtPic2=np.abs(np.diff((buf_gray[i],buf_gray[i+1]),axis=0))
            #Float to uint8
            dtPic2=np.squeeze((dtPic2*255).astype(np.uint8))
            # showImg(dtPic2)
            noiseMask[dtPic2<noiseSchwelle]=True
            
            # dtPic=np.max((dtPic,dtPic2),axis=0).astype(np.uint8)
            #
            dtPic=np.sum((dtPic,dtPic2*255),axis=0).astype(np.uint8)
            # showImg(dtPic)

        dtPic[noiseMask]=0
        showImg(dtPic)
        
        #~~~~~~~21.12.2021
        arr=getXYCoordsFromPixels(dtPic,1)
        conc=concaveHull.concHull(arr,300)
        concaveHull.plotPointsAndLines(arr,conc)
        
        return dtPic
        
        
        
        # fillMask=similarGreyVal(dtPic, buf_gray[i])
        
        # #Buffer dt mask with boarder as search start
        # # pic=np.random.random_integers(255,size=(10,10))
        # # dtPic=np.zeros((10,10))
        # # dtPic[4,5]=255
        
        # #buffer and search for same colors
        # sB=3
        
        # dtpicBuf=np.zeros((np.shape(dtPic)[0]+sB*2,(np.shape(dtPic)[1]+sB*2)))
        # dtpicBuf[sB:np.shape(dtPic)[0]+sB,sB:np.shape(dtPic)[1]+sB]=dtPic
        
        # #Buffer Picture with boarder
        # # pic[3:5,3:7]=5
        # picBuf=np.zeros(np.shape(dtpicBuf))
        
        # #!!!!!!!!
        # picBuf[sB:np.shape(dtPic)[0]+sB,sB:np.shape(dtPic)[1]+sB]=buf_gray[52]
        
        # #Searched Pic to show which pixel has been searched already
        # searchedPic=np.zeros(np.shape(picBuf)).astype(bool)
        
        



        # # x = dtpicBuf

        # # footprint = np.ones((50,50))
        # # showImg(dtpicBuf.astype(np.uint8))
        # # # dtpicBuf = ndimage.generic_filter(x, test_func, footprint=footprint)
        
        # # dtpicBuf=
        # showImg(dtpicBuf.astype(np.uint8))
        
        # dtpicBuf=gauss(dtpicBuf,sigma=4)
        # showImg(dtpicBuf.astype(np.uint8))
        # #Go through all pixels
        # y=1
        # schwelle=0.05
        # while y<np.shape(picBuf)[0]-1:
        #     x=1
        #     print(y)
        #     while x<np.shape(picBuf)[1]-1:
        #        pixel=dtpicBuf[y,x]
        #        if pixel>=50:
        #             #If Neighbouring Pixel is same as center pixel, set dtPic to 255, set this pixel to searchedPic=True. only search pixels if searchedPic=false
        #             if not searchedPic[y,x]:
        #                 searchedPic[y,x]=True
                        
        #                 block=picBuf[y-sB:y+1+sB,x-sB:x+1+sB]
        #                 # print(block)
                        
        #                 mask=np.logical_or(block<block[sB,sB]-schwelle,block>schwelle+block[sB,sB])
        #                 fill=np.ones((1+2*sB,1+2*sB))*pixel
                        
        #                 fill[mask]=0
        #                 # print(fill)
                        
        #                 picBuf[y-sB:y+1+sB,x-sB:x+1+sB][np.logical_not(mask)]=picBuf[y,x]
                        
        #                 dtpicBuf[y-sB:y+1+sB,x-sB:x+1+sB]=np.max((fill,dtpicBuf[y-sB:y+1+sB,x-sB:x+1+sB]),axis=0)
                        
        #                 # showImg(np.min((dtpicBuf[1:np.shape(dtPic)[0]+1,1:np.shape(dtPic)[1]+1],buf_gray[51]*255),axis=0).astype(np.uint8))
        #                 # searchedPic[y-1:y+2,x-1:x+2]=fill.astype(bool)
        #                 #to make sure thate pixels to the top left are also searched if set to 255 through this loop, go back 1 row and 1 column
        #                 #x will be incremented right after this conditional --> -2
        #                 # x-=2
        #                 # y-=1
        #        x+=1         
        #     y+=1            
                
        
        # dtPic2=dtpicBuf[1:361,1:641]
        
        # # dtPic=np.array(image_functions.schwellwert(dtPic,10), dtype=np.uint8)
        
        # dxdyMatte=matteFromDxDySchwellert(np.squeeze(buf_dx[51]))
        # dxdyMatte=np.array(image_functions.schwellwert(gauss(dxdyMatte,sigma=5),200), dtype=np.uint8)
        
        
        # print("Picture without noise")
        # MatteComb=np.max((0*dxdyMatte,dtPic),axis=0).astype(np.uint8)
        # images=np.min((MatteComb,buf_gray[51]*255),axis=0).astype(np.uint8)
        # showImg(images)
        
        
        # print("Picture without noise and search for similar colors")        
        # MatteComb=np.max((0*dxdyMatte,dtPic2),axis=0).astype(np.uint8)
        # images2=np.min((MatteComb,buf_gray[51]*255),axis=0).astype(np.uint8)
        # showImg(images2)
       
        
    # print("stop")
    # return dxdyMatte
    # showUsedMatteFromDxDy(buf)
    
    

            
    
    
def showUsedMatteFromDxDy(buf):
    
        #Pictures to DxDy with gray scaling included
        buf_dx=(255*giveDxDxDiff(buf,5,5)).astype(np.uint8)
        
        #buf gray to schwellwert
        images=np.array(image_functions.schwellwert(buf_dx,10), dtype=np.uint8)
        playNpArray(images)
        
        images=images[52:73,:,:]
        
        
        buf_gray=rgb2gray(buf)
        buf_gray=buf_gray[52:73,:,:]

        images=list(images)

        results=list(map(matteFromDxDySchwellert,images))
       
        for n in range(0,len(images)):
             #Makeing Matte Bigger
             results[n]=np.array(image_functions.schwellwert(gauss(results[n],sigma=5),200), dtype=np.uint8)
             
             images[n]=np.min((gauss(results[n],sigma=2),buf_gray[n]*255),axis=0).astype(np.uint8)
             showImg(images[n])
             
        playNpArray(np.asarray(images))
        
        
        
def matteFromDxDySchwellert(image):
    imageChecked=np.zeros(np.shape(image), dtype=bool)
    
    #Seed for searching
    y=100
    x=450
    
    #Must be odd!
    minSize=3
    maxSize=0
    counter=0
    schwelle=0.70
    imageMeshSearched=False
    
    while ~imageMeshSearched:
        searchOutline=True
        x=x-5
        
        while searchOutline:
            print("Outline Search Start bei x = " + str(x))
            y,x=leftSearch(image,y,x-1,imageChecked)
            searchOutline,y,x,maxSize, schwelleReached=checkCubeSize(image,y,x,minSize,schwelle, 1)
            counter +=1
            
            imageMeshSearched=x<=0
                
            # print("Search Outline Tries: " + str(counter))
            # print("Erreichter Fillgrad :" + str(schwelleReached))
            if counter >40 or imageMeshSearched:
                imageMeshSearched=True
                break
           
                
        print("Walk along Search Start bei x = " + str(x))
        
        #Abbruch wenn x bei null Angekommen ist
        if imageMeshSearched:
            break
                
                
        dist2Center=minSize
        
        # xyCoords=walkAlongOutline(y,x,minSize,schwelle,dist2Center)
        
    
        
        
    
        lengthCheck=100
        
        #Find Start angle by trying to find what is outside(empty) and inside(full)
        #Right =0, down=90,up=-90
        startAngle=findStartAngle(image,y,x,lengthCheck)
        try:
            xyCoordsList
        except NameError:
            xyCoords=walkAlongOutlineBiased(image,startAngle,y,x,maxSize,schwelle/3,3*dist2Center, True)
            xyCoordsList=[xyCoords]
        else:
            xyCoordsNew=walkAlongOutlineBiased(image, startAngle,y,x,maxSize,schwelle/3,3*dist2Center, True)
            xyCoordsList.append(xyCoordsNew)
            

        #Start in the opposite Direction to last time
        buffer=20
        if xyCoords[1,2]+buffer<0:
            counterAngle=xyCoords[1,2]+buffer+180
        else:
            counterAngle=-180+((xyCoords[1,2]+buffer+180)%180)

        xyCoordsRev=walkAlongOutlineBiased(image, counterAngle,y,x,maxSize,schwelle/3,3*dist2Center, False)
        
        xyCoordsRev=np.flipud(xyCoordsRev)
        
        xyCoordsList[-1]=np.append(xyCoordsList[-1],xyCoordsRev,axis=0)
        
        #Since only the first Element is lead to a matte other can be ignored. Should be done for outline search in similar manner
        break
        


        
    for i, xyCoords in enumerate(xyCoordsList):
        xyCoordsList[i]=xyCoordsInterp(xyCoords)
        #Since only the first Element is lead to a matte other can be ignored. Should be done for outline search in similar manner
        break
    
    
        
    
    #Returns xyCoords Background Array with matte frome the FIRST outline in the xyCoordsList
    return xyCoordsToMatte(image,xyCoordsList[0]).astype(np.uint8)

    
def xyCoordsToMatte(image,xyCoordsInt):
    matte=np.zeros(np.shape(image))
    #Filter xyCoords das "Sattelpunkte" entfernt werden. x(n-1)==x(n+1) --> roll
    xyCoordsIntRoll=np.roll(xyCoordsInt[:,1],2)
    mask=np.roll(np.diff((xyCoordsInt[:,1],xyCoordsIntRoll),axis=0),-1)
    xyCoordsIntFilt=xyCoordsInt[np.squeeze(mask)!=0]
    xUnique=np.unique(xyCoordsIntFilt[:,1]).astype(int)
    for x in xUnique:
        #Last index is biggest
        n=-1
        xUniqueXYSorted=np.sort(xyCoordsIntFilt[xyCoordsIntFilt[:,1]==x],axis=0).astype(int)
        # matte[xUniqueXYSorted[n,0],xUniqueXYSorted[n,1]]=255
        switch=True
        size=xUniqueXYSorted[:,0].size+1
        # print('size {} and x value {}'.format(size, xUniqueXYSorted[n,1]))
        for n in range (-2,-size,-1):
            # print('y:{}:{} and x value {} ist {}'.format(xUniqueXYSorted[n,0],xUniqueXYSorted[n+1,0], xUniqueXYSorted[n,1],switch))
            matte[xUniqueXYSorted[n,0]:xUniqueXYSorted[n+1,0],xUniqueXYSorted[n,1]]= switch * 255
            switch = not switch
    return matte

def xyCoordsInterp(xyCoords):
    #Filter duplicate, linked x values. Starting from top to allow deleting
    x=xyCoords[-1,1]
    mask_array=np.ones(np.shape(xyCoords)[0]).astype(bool)
    i=np.shape(xyCoords)[0]-1
    counter1=0
    counter2=0
    while i >=0:
        found=False
        # counter1 +=1
        # if counter1>100:
        #     1+1
            
        ii=i
        while xyCoords[i,1]==x and ii-i<xyCoords[:,1].size:
            found=True
            # counter2 +=1
            # if counter2>100:
            #     1+1
            
            i -=1
        
        i += found * 1
        if i<ii:
            if i<0:
                xyCoords=np.roll(xyCoords,-i,axis=0)
                mask_array=np.roll(mask_array,-i,axis=0)
                
                yMax=int(np.argmax(xyCoords[0:-i+ii+1,0]))
                mask_array[0:-i+ii+1]=False
                mask_array[yMax]=True
                
                xyCoords=np.roll(xyCoords,+i,axis=0)
                mask_array=np.roll(mask_array,+i,axis=0)
            else:
                yMax=int(np.argmax(xyCoords[i:ii+1,0]))
                mask_array[i:ii+1]=False
                mask_array[i+yMax]=True

        
        i = i-1
        x=xyCoords[i,1]
    
    xyCoords=xyCoords[mask_array]
    
    #Interpolate missing x values
    i=np.shape(xyCoords)[0]-1
    neededSize=1
    if i>1:
        neededSize=np.sum(np.abs((np.diff((xyCoords[:,1],np.roll(xyCoords[:,1],-1,axis=0)),axis=0))))
        # while i >0:
        #     neededSize=neededSize + abs(xyCoords[i,1]-xyCoords[i-1,1])
        #     i = i-1
            
    xyCoordsInt=np.empty((int(neededSize),3))
    
       
        
    rowInt=0
    for row in range(0,np.shape(xyCoords)[0],1):
        row2= row+1
        if row==np.shape(xyCoords)[0]-1:
            row2=0

        xyCoordsInt[rowInt,:]=xyCoords[row,:]
        delta=xyCoords[row2,1]-xyCoords[row,1]
        if abs(delta)>1:
            vorZeichen=(delta>0)*1+(delta<=0)*-1
            xValues=np.arange(xyCoords[row,1]+vorZeichen,xyCoords[row2,1],vorZeichen)
            
            deltaY=xyCoords[row2,0]-xyCoords[row,0]
            vorZeichen=(deltaY>0)*1+(deltaY<=0)*-1
            step=deltaY/abs(delta)
            if step==0:
                yValues=np.ones(int(abs(delta)-1))*xyCoords[row,0]
            else:
                yValues=np.arange(xyCoords[row,0]+step,xyCoords[row2,0]-step/2,step)
                
            yValues=np.round(yValues).astype(int)
            
            angles=np.ones(int(abs(delta)-1))*xyCoords[row,2]
            
            data=np.stack((yValues,xValues,angles),axis=1)
            xyCoordsInt[rowInt+1:int(rowInt+abs(delta)),:]=data
            
        rowInt =int(rowInt+abs(delta))
    
    # xyCoordsInt[-1,:]=xyCoords[-1,:]
    
    return xyCoordsInt
            
            

def showXYCoords(image,xyList,size):
        bgAr=np.zeros(np.shape(image))
        for xyCoords in xyList:
             for p in xyCoords:
                 bgAr=np.max((bgAr,printRedCube(image,np.array([None,p[0],p[1],size,1]))),axis=0)
            
        bgAr=bgAr.astype(np.uint8)
        showImg(util.img_as_ubyte(resize(np.stack((np.zeros(np.shape(image),np.uint8),image,bgAr),axis=2),(600,600))))

#Search while loop
def walkAlongOutlineBiased(image,startAngle,y,x,minSize,schwelle,dist2Center,leftTurn):
    #2nd point anywhere around the 1 with 360°
    #Possible Outline found. Check CIrcumfrence for similar Cubes
    cubeNotFoundAtAll=False
    if leftTurn:
        searchAngleWidth=270
    else:
        searchAngleWidth=100
    searchAngle=startAngle
    # if leftTurn:
    #     if startAngle+searchAngleWidth/2<180:
    #         searchAngle=startAngle+searchAngleWidth/2
    #     else:
    #         searchAngle=-180+(startAngle+searchAngleWidth/2) % 180
    # else:
    #     if startAngle-searchAngleWidth/2>-180:
    #         searchAngle=startAngle-searchAngleWidth/2
    #     else:
    #         searchAngle=(startAngle-searchAngleWidth/2) % 180
            
            
    maxDist=3*dist2Center
    xyCoordsFound=np.array([y,x,searchAngle],dtype=object)
    xyCoordsFound=np.expand_dims(xyCoordsFound, axis=0)     
    circleFinished=False
    
    #Schleife während auch bei max Distanz kein punkt gefunden wird und kein Punkt zwei mal erreicht wird.
    while not cubeNotFoundAtAll and not circleFinished:
        cubeNotFound=True
        outlineCord=createSearchCoordSorted(image,y,x, dist2Center, searchAngle,searchAngleWidth)
        # print(str(x) + "x     :y " + str(y))

        outLineAr=np.empty((0,5),np.uint8)
        
        
        #Searches on right side first 
        if not leftTurn:
            outlineCord=np.flipud(outlineCord)
            

        for p in outlineCord:     
            data=np.array([(checkCubeSize(image,p[0],p[1],minSize,schwelle,2))])
            outLineAr=np.append(outLineAr, data,axis=0)
            if data[0,4]>schwelle:
                cubeNotFound, y2,x2,sizeFound,areaFill2 =outLineAr[-1,:]
                break
    
        # outLineAr=outLineAr[outLineAr[:,4]>=np.max(outLineAr[:,4])]
        # cubeNotFound, y2,x2,sizeFound,areaFill2 =outLineAr[0,:]
        # print(xyCoordsFound)
        # print(str(y2)+":"+ str(x2))
        
        if cubeNotFound:
            dist2Center +=dist2Center
            if dist2Center>maxDist:
                cubeNotFoundAtAll=True
                # print("auch bei maximaler weite kein punkt gefunden")
            
        
        if not cubeNotFound and any(np.equal(xyCoordsFound[:,0:2],[y2,x2]).all(1)):
            print("circle finished")
            circleFinished=True
        
        

        #Only print x and y into found xy, if they where in bound and reached the schwellwert
        if cubeNotFound:
            1+1
            # print("cube out of bounds oder kein schwellwert erreicht")
        else:
            angle12=np.arctan2(([y2-y]),([x2-x]))*180/np.pi
            angle12=angle12
            xyCoordsFound=np.vstack((xyCoordsFound,(np.transpose([y2,x2,angle12]))))
            searchAngleWidth=180
            searchAngle=angle12
            y=y2
            x=x2
            

        
        # print(str(x2) + "x     :y " + str(y2)  +"  with angle " + str(angle12))
        

        

        
    return xyCoordsFound


 #Search while loop
def walkAlongOutline(image,y,x,minSize,schwelle,dist2Center):
    #2nd point anywhere around the 1 with 360°
    #Possible Outline found. Check CIrcumfrence for similar Cubes
    cubeNotFoundAtAll=False
    searchAngleWidth=360
    searchAngle=0
    maxDist=3*dist2Center
    xyCoordsFound=np.array([y,x])
    xyCoordsFound=np.expand_dims(xyCoordsFound, axis=0)     
    circleFinished=False
    
    #Schleife während auch bei max Distanz kein punkt gefunden wird und kein Punkt zwei mal erreicht wird.
    while not cubeNotFoundAtAll and not circleFinished:
        outlineCord=createSearchCoord(y,x, dist2Center, searchAngle,searchAngleWidth)
        # print(str(x) + "x     :y " + str(y))

        outLineAr=np.empty((0,5),np.uint8)
        for p in outlineCord:         
            outLineAr=np.append(outLineAr, np.array([(checkCubeSize(image,p[0],p[1],minSize,schwelle,2))]),axis=0)
    
        outLineAr=outLineAr[outLineAr[:,4]>=np.max(outLineAr[:,4])]
        cubeNotFound, y2,x2,sizeFound,areaFill2 =outLineAr[0,:]
        # print(xyCoordsFound)
        # print(str(y2)+":"+ str(x2))
        
        if cubeNotFound:
            dist2Center +=dist2Center
            if dist2Center>maxDist:
                cubeNotFoundAtAll=True
                print("auch bei maximaler weite kein punkt gefunden")
            
        
        if any(np.equal(xyCoordsFound,[y2,x2]).all(1)):
            print("circle finished")
            circleFinished=True
        
        #Only print x and y into found xy, if they where in bound and reached the schwellwert
        if cubeNotFound:
            print("cube out of bounds oder kein schwellwert erreicht")
        else:
            xyCoordsFound=np.vstack((xyCoordsFound,([y2,x2])))
        angle12=np.arctan2(([y2-y]),([x2-x]))*180/np.pi
        
        # print(str(x2) + "x     :y " + str(y2)  +"  with angle " + str(angle12))
        
        searchAngleWidth=92
        searchAngle=angle12
        y=y2
        x=x2
        

        
    return xyCoordsFound

def printRedCube(image,arr):
        bol,y,x,cubeLength, schwelleReached=arr
        y=int(y)
        x=int(x)
        # print(str(y) + ":" + str(x))
        length=int(cubeLength+8)
        l2=(length/2)
        bgAr= np.zeros(np.shape(image),np.uint8)
        cubeFillAr=np.ones((length,length),np.uint8)*255
        cubeClearAr=np.zeros((length-4,length-4),np.uint8)*255
        cubeFillAr[2:length-2,2:length-2]=cubeClearAr
        # if np.shape(bgAr[int(y-l2):int(y+l2),int(x-l2):int(x+l2)])!=(11,11):
        #     Print("achtung")
        
        if np.shape(bgAr[int(y-l2):int(y+l2),int(x-l2):int(x+l2)])==np.shape(cubeFillAr)      :   
            bgAr[int(y-l2):int(y+l2),int(x-l2):int(x+l2)]=cubeFillAr
        return bgAr
        
# def createSearchCoord(image,y,x, dist2Center, searchAngle,searchAngleWidth):

    
#     #Creates coordintes around found x,y cube
#     yTop=min(y+dist2Center,np.shape(image)[0]-1)
#     yBot=max(y-dist2Center,0)
#     xLeft=max(x-dist2Center,0)
#     xRight=min(x+dist2Center,np.shape(image)[1]-1)
    
#     centerXTop=np.transpose(np.vstack((np.ones((dist2Center*2)+1)*yTop,np.arange(xLeft,xRight+1))))
#     centerXBot=np.transpose(np.vstack((np.ones((dist2Center*2)+1)*yBot,np.arange(xLeft,xRight+1))))
#     centerYLeft= np.transpose(np.vstack((np.arange(yBot+dist2Center-(dist2Center-1)/2-1,yTop-dist2Center+(dist2Center-1)/2+2), np.ones((dist2Center)+2)*(xLeft))))
#     centerYRight= np.transpose(np.vstack((np.arange(y-(dist2Center-1)/2-1,yTop-dist2Center+(dist2Center-1)/2+2), np.ones((dist2Center)+2)*(xRight))))
#     outlineCord=np.vstack(((centerXTop, centerXBot , centerYLeft , centerYRight)))
    
    
#     #Creates angles to filter for. 0° is straight right 180 straight left, 90 up, -90 down
#     outlineCordsOrigin=np.transpose(np.vstack((outlineCord[:,0]-y,outlineCord[:,1]-x)))
#     outlineAngles=np.empty(np.shape(outlineCord)[0])
    
    
#     #Creates mask according to allowe angles
#     outlineAngles=np.arctan2(outlineCordsOrigin[:,0],outlineCordsOrigin[:,1])/(np.pi)*180
#     if searchAngleWidth==360:
#         outlineCord=outlineCord
#     elif searchAngle+searchAngleWidth/2>180:
#         angleLow=-180-((searchAngle+searchAngleWidth/2) % 180)
#         angleHigh=searchAngle-searchAngleWidth/2
#         mask=np.max((np.less(outlineAngles,angleLow),np.greater(outlineAngles,angleHigh)),axis=0)
#         outlineCord=outlineCord[mask]
#     elif (searchAngle-searchAngleWidth/2)<-180:
#         angleHigh=180-((searchAngle-searchAngleWidth/2) % 180)
#         angleLow=searchAngle-searchAngleWidth/2        
#         mask=np.max((np.less(outlineAngles,angleLow),np.greater(outlineAngles,angleHigh)),axis=0)
#         outlineCord=outlineCord[mask]
#     else:
#         angleLow=searchAngle-searchAngleWidth/2
#         angleHigh=searchAngle+searchAngleWidth/2
#         mask=np.min((np.greater(outlineAngles,angleLow),np.less(outlineAngles,angleHigh)),axis=0)
#         outlineCord=outlineCord[mask]
        
#     return outlineCord
    
def createSearchCoordSorted(image,y,x, dist2Center, searchAngle,searchAngleWidth):

    
    
    #Creates coordintes around found x,y cube
    yTop=int(min(y+dist2Center,np.shape(image)[0]-1))
    yBot=int(max(y-dist2Center,0))
    xLeft=int(max(x-dist2Center,0))
    xRight=int(min(x+dist2Center,np.shape(image)[1]-1))
    
    centerXTop=np.transpose(np.vstack((np.ones(xRight-xLeft+1)*yTop,np.arange(xLeft,xRight+1))))
    centerXBot=np.transpose(np.vstack((np.ones(xRight-xLeft+1)*yBot,np.arange(xLeft,xRight+1))))
    yArange=np.arange(yBot+dist2Center-(dist2Center-1)/2-1,yTop-dist2Center+(dist2Center-1)/2+2)
    centerYLeft= np.transpose(np.vstack((yArange, np.ones(yArange.size)*(xLeft))))
    centerYRight= np.transpose(np.vstack((yArange, np.ones(yArange.size)*(xRight))))
    outlineCord=np.vstack(((centerXTop, centerXBot , centerYLeft , centerYRight)))
    
    
    #Creates angles to filter for. 0° is straight right 180 straight left, 90 up, -90 down
    outlineCordsOrigin=np.transpose(np.vstack((outlineCord[:,0]-y,outlineCord[:,1]-x)))
    outlineAngles=np.empty(np.shape(outlineCord)[0])

    
    #Creates existing angles for coords
    outlineAngles=np.arctan2(outlineCordsOrigin[:,0],outlineCordsOrigin[:,1])/(np.pi)*180
    
    #Stack angles and coords together and sort by angles
    outlineCoordsAndAngles=np.hstack((outlineCord,np.expand_dims(outlineAngles,axis=1)))
    outlineCoordsAndAnglesSorted =outlineCoordsAndAngles[np.argsort(outlineCoordsAndAngles[:, 2])]
    
    #Used sorted angles as mask on sorted coordinates
    outlineAngles=outlineCoordsAndAnglesSorted[:,2:3]
    outlineCord=outlineCoordsAndAnglesSorted[:,0:2]
    mask=np.ones(np.shape(outlineAngles)).astype(bool)
    
    if searchAngleWidth==360:
        outlineCord=outlineCord
    elif searchAngle+searchAngleWidth/2>180:
        angleLow=-180-((searchAngle+searchAngleWidth/2) % 180)
        angleHigh=searchAngle-searchAngleWidth/2
        mask=np.max((np.less(outlineAngles,angleLow),np.greater(outlineAngles,angleHigh)),axis=0)
        # outlineCord=outlineCord[mask]
    elif (searchAngle-searchAngleWidth/2)<-180:
        angleHigh=((searchAngle-searchAngleWidth/2) % 180)
        angleLow=searchAngle+searchAngleWidth/2        
        mask=np.max((np.less(outlineAngles,angleLow),np.greater(outlineAngles,angleHigh)),axis=0)
        # outlineCord=outlineCord[mask]
    else:
        angleLow=searchAngle-searchAngleWidth/2
        angleHigh=searchAngle+searchAngleWidth/2
        mask=np.min((np.greater(outlineAngles,angleLow),np.less(outlineAngles,angleHigh)),axis=0)
        # outlineCordtest = outlineCord[np.squeeze(mask),:]
        # outlineCord=outlineCord[np.squeeze(mask),:]
        
    #Mask Coordinates
    
    # outlineCord=outlineCord[np.squeeze(mask),:]
    outlineCord=outlineCoordsAndAnglesSorted[np.squeeze(mask),:].astype(int)
    
    # Mask Angles as well
    outlineAngles=outlineAngles[mask]
    outlineAngles=np.expand_dims(outlineAngles,axis=1)
    
    #Center the array around the search angle, so that the first element is the most left of the
    #desired search angle and the last the most right, even if 180 degrees is overtaken/skipped
    #eg search angle 172+-20 (rounding) --> 150,160,170,180,-170-->sorted -170,150,160,170,180
    if searchAngle-searchAngleWidth/2>-180:
        firstElement=searchAngle-searchAngleWidth/2
    else:
        firstElement=(searchAngle-searchAngleWidth/2) %180
    index=np.searchsorted(outlineAngles[:,0],firstElement, side="left")
    outlineCord=np.roll(outlineCord,-index,axis=0)
    
    return outlineCord
    
def find_nearest(array,value):
    idx = np.searchsorted(array, value, side="left")
    if idx > 0 and (idx == len(array) or math.fabs(value - array[idx-1]) < math.fabs(value - array[idx])):
        return array[idx-1]
    else:
        return array[idx]



#Checks for maximum cube size with
# y,x being the bottom right COrner of the Cube, for corner=1
#top right=corner 2, top left =corner 3, bottom left corner 4
def checkCubeSize(image,y,x,minSize,schwelle, corner):
    cubeNotFound=True
    # print("x=" + str(x) + " y=" + str(y))
    

        
    if y-(minSize-1)/2<1 or y-(minSize-1)/2<1 or y+(minSize-1)/2>np.shape(image)[0]-1 or x+minSize/2>np.shape(image)[1]-1:
        # print("Search for outline left bounds in checkCubeSize")
        # print("x=" + str(x) + " y=" + str(y))
        return False,y,x,0,0
    
    
    # while y-(minSize-1)/2>=0 and y-(minSize-1)/2=>0 and y+(minSize-1)/2>=np.shape(image)[0]-1 and x+minSize-/2>=np.shape(image)[1]-1:
    #     1+1
    
    #Schwelle von prozent zu Summe
    schwelleSum=minSize*minSize*schwelle*255
    #Note: y=n is only propererly sliced/included by (n:n+1) n+1 is exluded from the slice, n is included
    ylow=int(y-(minSize-1)/2)
    yup=int(y+(minSize-1)/2+1)
    xleft=int(x-(minSize-1)/2)
    xright=int(x+(minSize-1)/2+1)
    
    fillSum=np.sum(image[ylow:yup,xleft:xright])
    
    while fillSum>schwelleSum:
        cubeNotFound=False
        minSize +=+2
        schwelleSum=minSize*minSize*schwelle*255
        fillSum=np.sum(image[ylow:yup,xleft:xright])



    if not cubeNotFound:
        minSize -=2
        # print("Cube search found a big enough cube")
        # print("found point with cubesearch  x:y  --> " + str(x) +" : " +  str(y))
        # print("Size of Cube is " +str(minSize) )

    fillSum=fillSum/255/minSize/minSize
    # print(str(cubeNotFound) + "," + str(y) + "," + str(x)+ "," + str(minSize)+ "," + str(fillSum))
    
    return cubeNotFound,y,x,minSize,fillSum


    

def leftSearch(image,y,x, imageChecked):
    if y<0 or x<0 or y>np.shape(image)[0]-1 or x>np.shape(image)[1]-1:
        # print("Search for outline left bounds")
        # showImg(np.max((image,(imageChecked.astype(np.uint8)*255)),axis=0))
        return -1,-1
    imageChecked[y,x]=True
    if image[y,x]>125:
        # print("found point with leftSearch  x:y  --> " + str(x) +" : " +  str(y))
        # showImg(np.max((image,(imageChecked.astype(np.uint8)*255)),axis=0))
        return (y,x)
    return leftSearch(image,y,x-1, imageChecked)

    
    

def acsoSearch(image,y,x, imageChecked):
    imageChecked[y,x]=True
    if image[y,x]>125:
        showImg(np.max((image,(imageChecked.astype(np.uint8)*255)),axis=0))
        return y,x
    #Was pixel abouve unchecked and to left checked? If yes go up
    elif not imageChecked[y-1,x] and imageChecked[y,x-1]:
        return acsoSearch(image,y-1,x, imageChecked)
    
    #Was pixel below checked and to left unchecked? Go left
    elif imageChecked[y+1,x] and not imageChecked[y,x-1]:
        return acsoSearch(image,y,x-1, imageChecked)
    
    #Was pixel below unchecked and to right checked? Go down
    elif not imageChecked[y+1,x] and imageChecked[y,x+1]:
        return acsoSearch(image,y+1,x, imageChecked)
    
    #Was pixel above checked and to right checked? Go right
    elif imageChecked[y-1,x] and not imageChecked[y,x+1]:
        return acsoSearch(image,y,x+1, imageChecked)
    
    #First movment has to go up
    else:
        return acsoSearch(image,y-1,x, imageChecked) 
    
    
def outlineDetection(image, seedy, seedx, minOutlineSize, outlineShape, searchPattern, maxGap, maxAngle):
    y=seedy
    x=seedx
    if searchPattern.isequal("acso"):
        1+1
        



def showImg(image):
    if str(image.dtype)=='uint8':
           cv2.imshow('frame ', image)
           c=cv2.waitKey(0)
           cv2.destroyAllWindows()
    else:
        print("Achtung, Daten zum abspielen kein uint8")

#Gives Grayscale Differencemap for x and y displacement
def giveDxDxDiff(data,dx,dy):
    #Analyze grayscale for now
    data=rgb2gray(data)
    data_dx=image_functions.shiftx(data, -2)
    data_dy=image_functions.shifty(data,-2)
    stackdx=np.stack((data,data_dx),axis=0)
    stackdy=np.stack((data,data_dy),axis=0)
    
    diffx=np.squeeze(np.abs(np.diff(stackdx,axis=0)))
    diffy=np.squeeze(np.abs(np.diff(stackdy,axis=0)))
    stackDiff=np.stack((diffx,diffy),axis=0)
    return np.squeeze(np.max(stackDiff,axis=0))
    
    
    
#Returns the pgird of a mesh e.g. a 2x4 mesh with [[0,1,2], [0,1,2,3,4]]
def createGrate(vidData,xNr,yNr):
    h=np.shape(vidData)[1]
    w=np.shape(vidData)[2]
    dx=w/xNr
    dy=h/yNr
    return np.ogrid[h:0:-dy,w:0:-dx]
    
    return

def playNpArray(data):
    length=np.shape(data)[0]
    if str(data.dtype)=='uint8':
        fc=0
        while(fc<length):
           frame =data[fc]
           fc +=1
           cv2.imshow('frame ', frame)
           c=cv2.waitKey(10)
           if c & 0xFF == ord('q'):
            break
        
           if c & 0xFF == ord('p'):
              c=cv2.waitKey(0)    

        cv2.destroyAllWindows()
    else:
        print("Achtung, Daten zum abspielen kein uint8")


def resizeAndSaveVideo(path,horizontalRes,frameNr):
    cap = cv2.VideoCapture(path)
    outputResolution=horizontalRes
    outputHeight=int(0.5625*outputResolution)
    
    fourcc = cv2.VideoWriter_fourcc(*'MP4V')
    out = cv2.VideoWriter('output.mp4', fourcc, 20.0, (outputResolution,outputHeight))
    fc=0
    while(fc<frameNr):
        ret, frame = cap.read()
        

        frame=util.img_as_ubyte(resize(frame, (outputHeight, outputResolution))) #.astype(np.uint8)
    
        out.write(frame)
        cv2.imshow('frame', frame)
        c = cv2.waitKey(1)
        fc +=1
        if c & 0xFF == ord('q'):
            break
    
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    
def  whereIsOutside(image,y,x,lengthCheck):
    ymin=int(max(y-lengthCheck/2,0))
    xmin=int(max(x-lengthCheck/2,0))
    ymax=int(min(y+lengthCheck/2,np.shape(image)[0]))  
    xmax=int(min(x+lengthCheck/2,np.shape(image)[1]))      
    subImage=image[ymin:ymax,xmin:xmax]
    alpha=np.arange(0,180,10)
    
    results=map(partial(rotAndSum, subImage), alpha)
    
    # for alpha in range(0,180,30):
    #     rotAndSum(subImage,alpha)
    
    return list(results),list(alpha)


def rotAndSum(image,alpha):
    image=rotate(image,alpha,resize=True,center=None)
    # print(alpha)
    # showImg(util.img_as_ubyte(image))
    return np.sum(image[0:-1,0:np.shape(image)[1]//2])-np.sum(image[0:-1,np.shape(image)[1]//2:-1])


def f360to180(angle):
    return angle % 360 -180


def findStartAngle(image,y,x,lengthCheck):
    startAngles=(whereIsOutside(image,y,x,lengthCheck))
    index_min = min(range(len(startAngles[0])), key=startAngles[0].__getitem__)
    index_max = max(range(len(startAngles[0])), key=startAngles[0].__getitem__)
    hlp=max([0,1],key=[abs(startAngles[0][index_min]),abs(startAngles[0][index_max])].__getitem__)
    return (hlp==0)*(f360to180(startAngles[1][index_min])+180-90)+(hlp==1)*(f360to180(startAngles[1][index_max]+90+180))    
   
def giveImgsFromVid(filename):

    #Create CV Video object from mp4
    cap =cv2.VideoCapture(filename)
    frameCount =100 #int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    frameWidth = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frameHeight = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    
    #Create np with correct Dimensions with 3 Layers for RGB or HSV etc.
    buf=np.empty((frameCount,frameHeight,frameWidth,3),np.dtype('uint8'))
    fc=0
    ret =True
    
    #Read Video data to npArray
    while (fc<frameCount and ret):
        ret, buf[fc] =cap.read()
        fc +=1
        
    # playNpArray(buf)
    
    #Closes video stream
    cap.release()
    cv2.destroyAllWindows()
    return buf



if __name__ == "__main__":
    image=main()

    

# def test_func(values):
#     print(values)
#     return values.sum()


# x = np.array([[1,2,3],[4,5,6],[7,8,9]])

# footprint = np.array([[1,1,1],
#                       [1,0,1],
#                       [1,1,1]])

# results = ndimage.generic_filter(x[x>5], test_func, footprint=footprint,mode='constant', cval=0)


# #Buffer dt mask with boarder as search start
# pic=np.random.random_integers(255,size=(10,10))
# dtPic=np.zeros((10,10))
# dtPic[4,5]=255

# dtpicBuf=np.zeros((12,12))
# dtpicBuf[1:11,1:11]=dtPic

# #Buffer Picture with boarder
# pic[3:5,3:7]=5
# picBuf=np.zeros((12,12))
# picBuf[1:11,1:11]=pic

# #Searched Pic to show which pixel has been searched already
# searchedPic=np.zeros(np.shape(picBuf)).astype(bool)

# #Go through all pixels


# y=1
# while y<np.shape(picBuf)[0]-1:
#     x=1
#     while x<np.shape(picBuf)[1]-1:
#        pixel=dtpicBuf[y,x]
#        if pixel>=254:
#             #If Neighbouring Pixel is same as center pixel, set dtPic to 255, set this pixel to searchedPic=True. only search pixels if searchedPic=false
#             if not searchedPic[y,x]:
#                 print("found smth")
#                 searchedPic[y,x]=True
                
#                 block=picBuf[y-1:y+2,x-1:x+2]
#                 mask=[block!=block[1,1]]
#                 fill=np.ones((3,3))*255
#                 fill[tuple(mask)]=0
#                 dtpicBuf[y-1:y+2,x-1:x+2]=np.max((fill,dtpicBuf[y-1:y+2,x-1:x+2]),axis=0)
#                 # searchedPic[y-1:y+2,x-1:x+2]=fill.astype(bool)
#                 #to make sure thate pixels to the top left are also searched if set to 255 through this loop, go back 1 row and 1 column
#                 #x will be incremented right after this conditional --> -2
#                 x-=2
#                 y-=1
#        x+=1         
#     y+=1            
                


# def pixelsAround(y,x,dtPic):
#     footprint = np.array([[1,1,1],
#                           [1,0,1],
#                           [1,1,1]])
#     pixelAroundMatte=np.zeros(np.shape(dtPic)).astype(bool)
    
    
#     return 


# def similarGreyVal(dtPic, gPic):
#     searchedPic=np.zeros(np.shape(dtPic)).astype(bool)
#     for row in dtPic:
#         for pixel in row:
#             if pixel>=254:
#                 pixelsAround=pixelsAround(pixel,dtPic)
                
                

