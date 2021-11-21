import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull as ConvexHull

# rng = np.random.default_rng()

# points = rng.random((30, 2))   # 30 random points in 2-D

# hull = ConvexHull(points)
# plt.plot(points[:,0], points[:,1], 'o')

# for simplex in hull.simplices:

#     plt.plot(points[simplex, 0], points[simplex, 1], 'k-')
    
def main():
    # arr=rnd2dPoints(1000,1000)
    arr=test
    plotPointsAndLines(arr,convHull(arr))
    
def convHull(arr):
    vertices=ConvexHull(arr).vertices
    return np.take(arr[:,],vertices,axis=0)


def nearestPoint(arr,point,k):
    distance=(arr[:,0]-point[0])*(arr[:,0]-point[0])+(arr[:,1]-point[1])*(arr[:,1]-point[1])
    idx = np.argpartition(distance, k)
    return arr[idx[:k]],idx
    
def sortByAngle(kNearestPoints,currentPoint,prevAngle):
    return np.arctan2(kNearestPoints[:,0]-currentPoint[0],kNearestPoints[:,1]-currentPoint[1])+180
    
def concHull(arr,k):
    #Input. List of points to process (pointsList); number of neighbours (k) 
    #Output. An ordered list of points representing the computed polygon
    #from http://repositorium.sdum.uminho.pt/bitstream/1822/6429/1/ConcaveHull_ACM_MYS.pdf
    #ONCAVE HULL: A K-NEAREST NEIGHBOURS APPROACH 
    # FOR THE COMPUTATION OF THE REGION OCCUPIED BY A 
    # SET OF POINTS 
    # Adriano Moreira and Maribel Yasmina Santos
    
    #Make sure k is at least 3
    kk=max(k,3)
    
    if len(arr)<3:
        return None
    elif len(arr)==3:
        return arr
    
    index=np.argmin(arr)
    firstPoint=np.array(arr[index,0],arr[index,1])
    hull=firstPoint
    currentPoint=firstPoint
    arr=np.delete(arr,index,axis=0)
    previousAngle=0
    step=2
    
    while (((currentPoint!=firstPoint) or( step==2)) and (len[arr]>0) ) :
        if step==5:
            arr=np.insert(arr,firstPoint,axis=0)
            
        kNearestPoints, idx=nearestPoint(arr,firstPoint)
        cPoints=sortByAngle(kNearestPoints,currentPoint,prevAngle)
        

def rnd2dPoints(n,limit):
    return np.random.randint(-limit,limit,(n,2))

def plotPoints(array2d):
    # fig, ax = plt.subplots()
    plt.scatter(array2d[:,0], array2d[:,1],s=5)    
    plt.show()


    
def plotLines(array2d):
    plt.plot(array2d[:,0], array2d[:,1])
    # plt.ylabel('some numbers')
    plt.show()
    

def plotPointsAndLines(arr1,arr2):
        # fig, ax = plt.subplots()
    plt.scatter(arr1[:,1], -arr1[:,0],s=15)  
    plt.plot(arr2[:,1], -arr2[:,0], 'r')
    plt.show()
    



if __name__ == "__main__":
    main()
