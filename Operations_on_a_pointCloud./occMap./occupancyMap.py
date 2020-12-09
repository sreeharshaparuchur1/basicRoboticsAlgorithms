import open3d as o3d
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
import utils
#We aim to create a basic occupancy map based on the generated point clouds. We do not use a bayesian update rule, 
#for simplicity, we use a threshold value in judging if the "grid" is occupied or not.

def find_occMap(pcdf):
    points = np.asarray(pcdf.points)
    #o3d.visualization.draw_geometries([pcdf])
    print(points.shape)
    occMap = np.zeros((300, 300))
    maxX = 0
    maxY = 0
    maxZ = 0
    minX = 1000
    minY = 1000
    minZ = 1000
    for p in points:
        occMap[int(p[0]) + 100][int(p[2]) + 100] += 1
        if maxX < p[0]:
            maxX = p[0]
        if maxY < p[1]:
            maxY = p[1]
        if maxZ < p[2]:
            maxZ = p[2]
        if minX > p[0]:
            minX = p[0]
        if minY > p[1]:
            minY = p[1]
        if minZ > p[2]:
            minZ = p[2]
    #print(f"min x {minX} max X {maxX} min y {minY} max y {maxY} minZ {minZ} and maxZ {maxZ}")
    #Uncomment to see the bounds of x,y,z
    return occMap

occMap = find_occMap(pcdf)

def plot_occMap(name,occMap):
    for i in range(200):
        for j in range(200):
            if occMap[i][j] > 0 :
                occMap[i][j] = 1
            else:
                occMap[i][j] = 0

    fig = plt.figure(figsize=(16,16))
    cmap = mpl.colors.LinearSegmentedColormap.from_list('my_colormap',['black','white'],256)
    bounds=[-1,0.5, 2]
    norm = mpl.colors.BoundaryNorm(bounds, cmap.N)
    # tell imshow about color map so that only set colors are used
    img = plt.imshow(occMap,interpolation='nearest',cmap = cmap,norm=norm,origin='lower')
    
    # make a color bar
    fig.colorbar(img,cmap=cmap,
                    norm=norm,boundaries=bounds,ticks=[0,1])
  
    plt.savefig(name)
    plt.axis("off") 
    plt.show()
    return occMap

# def save_occMap(img,name):
#     plt.imsave(name,img,cmap = cmap,origin='lower')
    
# save_occMap(img,"combinedMap")

occlist = []
transform = utils.readData("01.txt")
lidar_to_cam = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]]).T
pcdf = o3d.geometry.PointCloud()
for i in range(10,87):
    tr = transform[i, :].reshape((3, 4))
    pc = np.matmul(lidar_to_cam,utils.readPointCloud("01/%06d.bin" % i)[:, :3].T).T
    pad = np.array([[0, 0, 0, 1.0]])
    tf = np.append(tr,pad, axis = 0) 
    ones = np.ones((pc.shape[0], 1))
    pc = np.matmul(tf, np.append(pc, ones, axis = 1).T).T
    pcreshape = pc[:, 3].reshape(-1, 1) 
    pc = pc[:, :3] / pcreshape
    pcd = o3d.geometry.PointCloud() 
    pcd.points = o3d.utility.Vector3dVector(pc)
    occMap = find_occMap(pcd)
    occlist.append(occMap)
    pcdf += pcd
    
#for i in range(len(occlist)):
#    plot_occMap("occMap"+str(i),occlist[i])
    
#Run this loop to visualise the generated occupancy maps.

