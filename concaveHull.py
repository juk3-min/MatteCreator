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
    arr=rnd2dPoints(1000,400)
    # arr=abc
    plotPointsAndLines(arr,convHull(arr))
    conc=concHull(arr,3)
    plotPointsAndLines(arr,conc)
    
    
    
def convHull(arr):
    vertices=ConvexHull(arr).vertices
    return np.take(arr[:,],vertices,axis=0)


def nearestPoint(arr,point,k):
    if len(arr)<=k:
        return arr
    
    distance=(arr[:,0]-point[0])*(arr[:,0]-point[0])+(arr[:,1]-point[1])*(arr[:,1]-point[1])
    idx = np.argpartition(distance, k)

    return arr[idx[:k]]
    
def anglesF(kNearestPoints,currentPoint):
    return np.arctan2(kNearestPoints[:,0]-currentPoint[0],kNearestPoints[:,1]-currentPoint[1])*180/np.pi+180

def angleF(kNearestPoints,currentPoint):
    return np.arctan2(kNearestPoints[0]-currentPoint[0],kNearestPoints[1]-currentPoint[1])*180/np.pi+180

def sortByAngle(kNearestPoints,currentPoint,prevAngle):
    angles=(anglesF(kNearestPoints,currentPoint)+360-((prevAngle+180))) %360
    idx=np.argsort(angles)
    anglesSorted=angles[idx]
    pointsSorted=kNearestPoints[idx]
    
    index=np.searchsorted(anglesSorted,0, side="right")
    if not index==0:
        1+1
        
    pointsSortedRolled=np.roll(pointsSorted,-index,axis=0)
    anglesSortedRolled=np.roll(anglesSorted,-index,axis=0)
    return pointsSortedRolled, anglesSortedRolled
    
def concHull(arr,k):
    #Input. List of points to process (pointsList); number of neighbours (k) 
    #Output. An ordered list of points representing the computed polygon
    #from http://repositorium.sdum.uminho.pt/bitstream/1822/6429/1/ConcaveHull_ACM_MYS.pdf
    #ONCAVE HULL: A K-NEAREST NEIGHBOURS APPROACH 
    # FOR THE COMPUTATION OF THE REGION OCCUPIED BY A 
    # SET OF POINTS 
    # Adriano Moreira and Maribel Yasmina Santos
    
    #Make sure k is at least 3
    pointList=arr
    kk=max(k,3)
    
    if len(arr)<3:
        return None
    elif len(arr)==3:
        return arr
    
    index=np.argmin(arr,axis=0)[0]
    firstPoint=np.array((arr[index,0],arr[index,1]))
    hull=np.expand_dims(firstPoint,axis=0)
    
    currentPoint=firstPoint
    arr=np.delete(arr,index,axis=0)
    previousAngle=270
    step=2
    
   
    while (( not np.array_equal(currentPoint,firstPoint) or( step==2)) and (len(arr)>0) ) :
        if step==5:
            arr=np.insert(arr,-1,firstPoint,axis=0)
            
        kNearestPoints=nearestPoint(arr,currentPoint,kk)
        # hlp=np.copy(kNearestPoints)
        
        
        # for tt in range(len(hlp),0,-2):
        #       hlp=np.insert(hlp,tt,currentPoint,axis=0)
        # plotPointsAndLines(pointList,kNearestPoints)
        
        cPoints,angles=sortByAngle(kNearestPoints,currentPoint,previousAngle)
        # if step>4:
        #     plotPoints3(currentPoint, arr, cPoints,hull,angles)
        
        its= True
        i=-1
        
        #select the first candidate that does not intersects any of the polygon edges 
        while (its and i<len(cPoints)-1):
            # print(cPoints[i,:])
            i+=1
            lastPoint=1 if (np.array_equal(cPoints[i,:],firstPoint)) else 0
            j=2
            its=False
            
            while(not its and j<len(hull)-lastPoint):
                its=intersectsQ(hull[step-2,:],cPoints[i,:],hull[step-2-j,:],hull[step-j-1,:])

                # if its and step>4:
                #      print("intersection zw knoten " + str(j) + "und Knoten " + str(j+1))
                    
                j+=1
                
        if its==True:
            # kNearestPoints, idx=nearestPoint(arr,currentPoint,kk)
            # plotPoints(kNearestPoints)
            # plotPointsAndLines(pointList,hull)
            # plotPointsAndLines(pointList,cPoints)
            
            # j=2
            # its=False
            # while(not its and j<len(hull)-lastPoint):
            #     print(intersectsQ(hull[step-2,:],cPoints[i,:],hull[step-2-j,:],hull[step-j-1,:]))
            #     j+=1
            
            plotPointsAndLines(pointList,kNearestPoints)
            print("all intercepted")
            print(kk+1)
            return concHull(pointList,kk+1)
        
        currentPoint=cPoints[i,:]
        hull=np.append(hull,np.expand_dims(currentPoint, axis=0),axis=0)
        # plotPointsAndLines(pointList,hull)
        
        
        previousAngle=angleF(hull[step-1,:],hull[step-2,:])
        # print(previousAngle)
        index=arr.tolist().index(currentPoint.tolist())
        arr=np.delete(arr,index,axis=0)
        step +=1
    allInside=True
    i=len(arr)-1
    
    while (allInside==True and i>=0):
        allInside=PointInPolyGonQ(arr[i,:],hull)
        i-=1
    if allInside==False:
        return concHull(pointList,kk+10)
    
    return hull


#To be implemented
def PointInPolyGonQ(point,hull):
    return True
    
    
def intersectsQ(p1,p2,p3,p4):
    x1,y1=p1
    x2,y2=p2
    x3,y3=p3
    x4,y4=p4
    
    # print('x1,y1= [{},{}]'.format(p1[0],p1[1]))
    # print('x2,y2= [{},{}]'.format(p2[0],p2[1]))
    # print('31,y3= [{},{}]'.format(p3[0],p3[1]))
    # print('x4,y4= [{},{}]'.format(p4[0],p4[1]))
    
    # After belzier https://en.wikipedia.org/wiki/Line%E2%80%93line_intersection
    divider=(x1-x2)*(y3-y4)-(y1-y2)*(x3-x4)
    #Wenn Teiler =0 dann parallel oder antiparallel -> Kein schnittpunkt, sondern maximal überlappung. sollte ausgeschlossen werden können
    if divider ==0:
        return False
        # numerator1=((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))
        # numerator2=((x1-x3)*(y1-y2)-(y1-y3)*(x1-x2))
        # if numerator1==0 and numerator2==0:
        #     return False
        # else:
        #     return True
    
    t=((x1-x3)*(y3-y4)-(y1-y3)*(x3-x4))/(divider)
    u=((x1-x3)*(y1-y2)-(y1-y3)*(x1-x2))/(divider)
    if t>=0 and t<=1 and u>=0 and u<=1:
        # print("intersected")
        return True
    return False
       
    
    
    
def rnd2dPoints(n,limit):
    return np.random.randint(-limit,limit,(n,2))


def plotPoints(array2d):
    # fig, ax = plt.subplots()
    plt.scatter(array2d[:,0], array2d[:,1],s=5)    
    plt.show()
    
def plotPoints2(array2d,array2d2):
    # fig, ax = plt.subplots()
    plt.scatter(array2d[:,0], array2d[:,1],s=5)    
    plt.scatter(array2d2[:,0], array2d2[:,1],s=10)    
    plt.show()    

    
def plotPoints3(point,array2d,array2d2,arr2,angles):
    fig, ax = plt.subplots()

    plt.scatter(array2d[:,1], array2d[:,0],s=5)  
    plt.scatter(point[1], point[0],s=40)  
    if len(np.shape(array2d2))==1:
        array2d2=np.expand_dims(array2d2, axis=0)
    plt.scatter(array2d2[:,1],array2d2[:,0],s=10)
    plt.plot(arr2[:,1], arr2[:,0], 'r')
    
    

    for i, txt in enumerate(angles):
        ax.annotate("{:+.1f}".format(txt), (array2d2[i,1], array2d2[i,0]))

    size=10
    plt.xticks(range(point[1]-size,point[1]+size))
    plt.yticks(range(point[0]-size,point[0]+size))
    plt.xlim(point[1]-size,point[1]+size)
    plt.ylim(point[0]-size,point[0]+size)
    
    plt.show()  
    1+1



    
def plotLines(array2d):
    plt.plot(array2d[:,0], array2d[:,1])
    # plt.ylabel('some numbers')
    plt.show()
    

def plotPointsAndLines(arr1,arr2):
        # fig, ax = plt.subplots()
    plt.scatter(arr1[:,1], arr1[:,0],s=15)  
    plt.plot(arr2[:,1], arr2[:,0], 'r')
    plt.show()
    



if __name__ == "__main__":
    main()
