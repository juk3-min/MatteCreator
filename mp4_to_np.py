import numpy as np
import cv2
from skimage.transform import resize
from skimage import util
from skimage.color import rgb2gray
import image_functions



def main():
    print("teeeeeeest")

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
        
    playNpArray(buf)
    
    #Closes video stream
    cap.release()
    cv2.destroyAllWindows()
    
    
    buf_gray=(255*giveDxDxDiff(buf,5,5)).astype(np.uint8)
    # playNpArray(buf_gray)
    test=np.array(image_functions.schwellwert(buf_gray,10), dtype=np.uint8)
    playNpArray(test)

    return 1

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
    main()