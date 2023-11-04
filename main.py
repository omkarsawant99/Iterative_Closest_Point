import open3d as o3d
import numpy as np
import copy
from utils import *


demo_icp_pcds = o3d.data.DemoICPPointClouds()
source = o3d.io.read_point_cloud(demo_icp_pcds.paths[0])
target = o3d.io.read_point_cloud(demo_icp_pcds.paths[1])

# Visualizing both point clouds before alignment
target.paint_uniform_color([1, 0, 0])  # Red
o3d.visualization.draw_geometries([target, source])
# Close the visualization window to continue

s_pts = np.asarray(source.points)
t_pts = np.asarray(target.points)


if __name__ == "__main__":
    dist = 10
    final_T = np.eye(4,4)
    while(dist >= 0.001):
        # Finding correspondences using kd tree
        s_corr = findCorrespondences(s_pts, t_pts, source)

        # Finding distance between source and target
        dist = findDistanceBetweenPCDs(s_corr, t_pts)                       # This step is done before transforming the target cloud as correspondences change after each transformation of target cloud. 
                                                                            # Thus we would have to perform another kd tree (findCorrespondences) to find new source correspondences (s_corr) and then,
                                                                            # subtract with transformed t_pts

        # Finding transformation using procrustes
        transformation = procrustes(s_corr, t_pts)

        #Transforming target cloud
        t_pts = transformation @ np.vstack((t_pts.T, np.ones((1, t_pts.shape[0]))))
        t_pts = np.delete(t_pts, 3, 0)
        t_pts = t_pts.T

        # Final transformation
        final_T = final_T @ transformation


    # Creating the new point cloud from the transformed points
    end_target = o3d.geometry.PointCloud()
    end_target.points = o3d.utility.Vector3dVector(t_pts)

    # Visualizing the results
    source_cloud = copy.deepcopy(source)
    o3d.visualization.draw_geometries([end_target, source_cloud])
