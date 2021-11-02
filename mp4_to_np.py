import numpy as np
import cv2
from skimage.transform import resize
from skimage import util
from skimage.color import rgb2gray
import image_functions



# Starts searching for an outline from the defined seed as y,x Pixel coordinate. An outline (1=perfect´white)
# must have at least mitOutlineSize as size. Size measurement depends on the chosen shape default is cube.
# Size->width, other shapes could be circle. Size->Diameter. The searchpattern describes how the outline is
# searched for from the seed. Default is anti-clockwork spiral outwards("acso"). maxGap describes the gap in pixels
# between parts of the outline which are still considered continous. First the direction of the previous
# outline is continued. If no outline is found the search adds an increasing angle up to maxAngle until an outline
# or none are foudn. Imageborder is considered an outline
# the outline is filled with 1s afterwards

# def func(my_array):
#     my_array[:3] = [1,2,3]

# a = np.zeros(4)
# print(a)

# func(a)
# print(a)




def test():
    imageChecked=np.zeros(np.shape(testimage), dtype=bool)
    
    searchOutline=True
    #Seed for searching
    y=100
    x=250
    
    #Must be odd!
    minSize=3
    maxSize=0
    counter=0
    schwelle=0.70
    
    while searchOutline:
        y,x=leftSearch(testimage,y,x-1,imageChecked)
        searchOutline,y,x,maxSize, schwelleReached=checkCubeSize(testimage,y,x,minSize,schwelle, 1)
        counter +=1
        # print("Search Outline Tries: " + str(counter))
        # print("Erreichter Fillgrad :" + str(schwelleReached))
        if counter >40:
            break
            
    dist2Center=minSize
    
    xyCoords=walkAlongOutline(y,x,minSize,schwelle,dist2Center)
    
    #Right =0, down=90,up=-90
    startAngle=90
    xyCoords=walkAlongOutlineBiased(startAngle,y,x,maxSize,schwelle/3,3*dist2Center, True)
    
    #Start in the opposite Direction to last time
    buffer=20
    if xyCoords[1,2]+buffer<0:
        counterAngle=xyCoords[1,2]+buffer+180
    else:
        counterAngle=-180+((xyCoords[1,2]+buffer+180)%180)
        
        
    xyCoords=np.append(xyCoords,walkAlongOutlineBiased(counterAngle,y,x,maxSize,schwelle/3,3*dist2Center, False),axis=0)
    
    # outlineCord=createSearchCoord(y,x, dist2Center, angle12,searchAngleWidth)
    # outLineAr=np.empty((0,5),np.uint8)
    # for p in outlineCord:
    # #     outLineAr=np.append(outLineAr, np.array([(checkCubeSize(testimage,p[0],p[1],minSize,schwelle,2))]),axis=0)
    
    # outLineAr=outLineAr[outLineAr[:,4]>=np.max(outLineAr[:,4])]
    # bol, y2,x2,sizeFound,areaFill2 =outLineAr[0,:]
    # angle12=np.arctan2(([y2-y]),([x2-x]))*180/np.pi
    # print(str(x2) + "x     :y " + str(y2)  +"  with angle " + str(angle12))
    
    bgAr=np.zeros(np.shape(testimage))
    for p in xyCoords:
        bgAr=np.max((bgAr,printRedCube(testimage,np.array([None,p[0],p[1],minSize,1]))),axis=0)
    
    bgAr=bgAr.astype(np.uint8)
    showImg(util.img_as_ubyte(resize(np.stack((np.zeros(np.shape(testimage),np.uint8),testimage,bgAr),axis=2),(600,600))))
    
    # bgAr=printRedCube(testimage,np.array([None,y,x,maxSize,1]))
    
    # if not searchOutline:
        # for p in outLineAr:
            
                # bgAr=np.max((bgAr,printRedCube(testimage,p)),axis=0)
                # showImg(util.img_as_ubyte(resize(np.stack((np.zeros(np.shape(testimage),np.uint8),testimage,bgAr),axis=2),(600,600))))
    
    

    # showImg(util.img_as_ubyte(resize(np.stack((np.zeros(np.shape(testimage),np.uint8),testimage,bgAr),axis=2),(1200,1200))))
    1+1


#Search while loop
def walkAlongOutlineBiased(startAngle,y,x,minSize,schwelle,dist2Center,leftTurn):
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
    xyCoordsFound=np.array([y,x,searchAngle])
    xyCoordsFound=np.expand_dims(xyCoordsFound, axis=0)     
    circleFinished=False
    
    #Schleife während auch bei max Distanz kein punkt gefunden wird und kein Punkt zwei mal erreicht wird.
    while not cubeNotFoundAtAll and not circleFinished:
        outlineCord=createSearchCoordSorted(y,x, dist2Center, searchAngle,searchAngleWidth)
        # print(str(x) + "x     :y " + str(y))

        outLineAr=np.empty((0,5),np.uint8)
        
        
        #Searches on right side first 
        if not leftTurn:
            outlineCord=np.flipud(outlineCord)
            

        for p in outlineCord:     
            data=np.array([(checkCubeSize(testimage,p[0],p[1],minSize,schwelle,2))])
            outLineAr=np.append(outLineAr, data,axis=0)
            if data[0,4]>schwelle:
                break
    
        outLineAr=outLineAr[outLineAr[:,4]>=np.max(outLineAr[:,4])]
        cubeNotFound, y2,x2,sizeFound,areaFill2 =outLineAr[0,:]
        # print(xyCoordsFound)
        # print(str(y2)+":"+ str(x2))
        
        if cubeNotFound:
            dist2Center +=dist2Center
            if dist2Center>maxDist:
                cubeNotFoundAtAll=True
                print("auch bei maximaler weite kein punkt gefunden")
            
        
        if any(np.equal(xyCoordsFound[:,0:2],[y2,x2]).all(1)):
            print("circle finished")
            circleFinished=True
        
        
        angle12=np.arctan2(([y2-y]),([x2-x]))*180/np.pi
            
        #Only print x and y into found xy, if they where in bound and reached the schwellwert
        if cubeNotFound:
            print("cube out of bounds oder kein schwellwert erreicht")
        else:
            xyCoordsFound=np.vstack((xyCoordsFound,([y2,x2,angle12])))
            

        
        # print(str(x2) + "x     :y " + str(y2)  +"  with angle " + str(angle12))
        
        searchAngleWidth=180
        searchAngle=angle12
        y=y2
        x=x2
        

        
    return xyCoordsFound


 #Search while loop
def walkAlongOutline(y,x,minSize,schwelle,dist2Center):
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
            outLineAr=np.append(outLineAr, np.array([(checkCubeSize(testimage,p[0],p[1],minSize,schwelle,2))]),axis=0)
    
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

def printRedCube(testimage,arr):
        bol,y,x,cubeLength, schwelleReached=arr
        y=int(y)
        x=int(x)
        # print(str(y) + ":" + str(x))
        length=int(cubeLength+8)
        l2=(length/2)
        bgAr= np.zeros(np.shape(testimage),np.uint8)
        cubeFillAr=np.ones((length,length),np.uint8)*255
        cubeClearAr=np.zeros((length-4,length-4),np.uint8)*255
        cubeFillAr[2:length-2,2:length-2]=cubeClearAr
        # if np.shape(bgAr[int(y-l2):int(y+l2),int(x-l2):int(x+l2)])!=(11,11):
        #     Print("achtung")
        
        if np.shape(bgAr[int(y-l2):int(y+l2),int(x-l2):int(x+l2)])==np.shape(cubeFillAr)      :   
            bgAr[int(y-l2):int(y+l2),int(x-l2):int(x+l2)]=cubeFillAr
        return bgAr
        
def createSearchCoord(y,x, dist2Center, searchAngle,searchAngleWidth):

    
    #Creates coordintes around found x,y cube
    centerXTop=np.transpose(np.vstack((np.ones((dist2Center*2)+1)*(y+dist2Center),np.arange(x-dist2Center,x+dist2Center+1))))
    centerXBot=np.transpose(np.vstack((np.ones((dist2Center*2)+1)*(y-dist2Center),np.arange(x-dist2Center,x+dist2Center+1))))
    centerYLeft= np.transpose(np.vstack((np.arange(y-(dist2Center-1)/2-1,y+(dist2Center-1)/2+2), np.ones((dist2Center)+2)*(x-dist2Center))))
    centerYRight= np.transpose(np.vstack((np.arange(y-(dist2Center-1)/2-1,y+(dist2Center-1)/2+2), np.ones((dist2Center)+2)*(x+dist2Center))))
    outlineCord=np.vstack(((centerXTop, centerXBot , centerYLeft , centerYRight)))
    
    
    #Creates angles to filter for. 0° is straight right 180 straight left, 90 up, -90 down
    outlineCordsOrigin=np.transpose(np.vstack((outlineCord[:,0]-y,outlineCord[:,1]-x)))
    outlineAngles=np.empty(np.shape(outlineCord)[0])
    
    
    #Creates mask according to allowe angles
    outlineAngles=np.arctan2(outlineCordsOrigin[:,0],outlineCordsOrigin[:,1])/(np.pi)*180
    if searchAngleWidth==360:
        outlineCord=outlineCord
    elif searchAngle+searchAngleWidth/2>180:
        angleLow=-180-((searchAngle+searchAngleWidth/2) % 180)
        angleHigh=searchAngle-searchAngleWidth/2
        mask=np.max((np.less(outlineAngles,angleLow),np.greater(outlineAngles,angleHigh)),axis=0)
        outlineCord=outlineCord[mask]
    elif (searchAngle-searchAngleWidth/2)<-180:
        angleHigh=180-((searchAngle-searchAngleWidth/2) % 180)
        angleLow=searchAngle-searchAngleWidth/2        
        mask=np.max((np.less(outlineAngles,angleLow),np.greater(outlineAngles,angleHigh)),axis=0)
        outlineCord=outlineCord[mask]
    else:
        angleLow=searchAngle-searchAngleWidth/2
        angleHigh=searchAngle+searchAngleWidth/2
        mask=np.min((np.greater(outlineAngles,angleLow),np.less(outlineAngles,angleHigh)),axis=0)
        outlineCord=outlineCord[mask]
        
    return outlineCord
    
def createSearchCoordSorted(y,x, dist2Center, searchAngle,searchAngleWidth):

    
    #Creates coordintes around found x,y cube
    centerXTop=np.transpose(np.vstack((np.ones((dist2Center*2)+1)*(y+dist2Center),np.arange(x-dist2Center,x+dist2Center+1))))
    centerXBot=np.transpose(np.vstack((np.ones((dist2Center*2)+1)*(y-dist2Center),np.arange(x-dist2Center,x+dist2Center+1))))
    centerYLeft= np.transpose(np.vstack((np.arange(y-(dist2Center-1)/2-1,y+(dist2Center-1)/2+2), np.ones((dist2Center)+2)*(x-dist2Center))))
    centerYRight= np.transpose(np.vstack((np.arange(y-(dist2Center-1)/2-1,y+(dist2Center-1)/2+2), np.ones((dist2Center)+2)*(x+dist2Center))))
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
    outlineCord=outlineCoordsAndAnglesSorted[np.squeeze(mask),:]
    
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
        print("Search for outline left bounds in checkCubeSize")
        # print("x=" + str(x) + " y=" + str(y))
        return True,y,x,0,0
    
    
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
        

def main():

    #Create CV Video object from mp4
    cap =cv2.VideoCapture('small.mp4')
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
    
    
    buf_gray=(255*giveDxDxDiff(buf,5,5)).astype(np.uint8)
    # playNpArray(buf_gray)
    test=np.array(image_functions.schwellwert(buf_gray,10), dtype=np.uint8)
    playNpArray(test)
    
    return test[50]

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
    data_dx=image_functions.shiftx(data, 2)
    data_dy=image_functions.shifty(data,2)
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
    
if __name__ == "__main__":
    # testimage=main()
    test()