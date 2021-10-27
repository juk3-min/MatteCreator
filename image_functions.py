from scipy.ndimage import gaussian_filter
import numpy as np



#Short Call for Scipy function
def gauss(image,sigma):
    return gaussian_filter(image, sigma)

def shiftx(images,x):
    if len(np.shape(images))==3:
        return np.roll(images,x,axis=2)

def shifty(images,y):
    if len(np.shape(images))==3:
        return np.roll(images,y,axis=1)
    
def schwellwert(images, schwellwert):
    images =np.greater(images,schwellwert)*255
    return images

def mergeOnlyBrighten(images,firstDimIsStackDim):
    if firstDimIsStackDim:      
        return np.max(images,0)
    else:
        return np.max(images,np.shape(images)[-1])


# test=np.ones((5,3))
# test[0,1]=2
# test[1,1]=5

# print(test)

# print(schwellwert(test,4))

