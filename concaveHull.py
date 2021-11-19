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
    arr=rnd2dPoints(100,10)
    plotPoints(arr)
    plotLines(arr)
    
    vertices=ConvexHull(arr).vertices
    Hull=np.take(arr[:,],vertices,axis=0)

    plotPointsAndLines(arr,Hull)

    
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
    plt.scatter(arr1[:,0], arr1[:,1],s=15)  
    plt.plot(arr2[:,0], arr2[:,1])
    plt.show()
    



if __name__ == "__main__":
     main()
