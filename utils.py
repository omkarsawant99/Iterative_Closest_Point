import numpy as np
import open3d as o3d

def procrustes(s_pts, t_pts):
  '''Performs Orthogonal Procrustes Analysis on two point sets
  '''
  source_centroid = np.mean(s_pts, axis=0)
  target_centroid = np.mean(t_pts, axis=0)

  Z = 0
  for i in range(len(t_pts)):
    term1 = (s_pts[i, :] - source_centroid).reshape(3,1)
    term2 = (t_pts[i, :] - target_centroid).reshape(3,1)
    Z = Z + term1 @ term2.T
  
  u, s, v_transpose = np.linalg.svd(Z)
  v = v_transpose.T
  det = np.linalg.det(v @ u.T)
  R_ts = v @ np.array([[1, 0, 0], [0, 1, 0], [0, 0, det]]) @ u.T                  # Required rotation of source w.r.t target
  t_ts = target_centroid.reshape(3,1) - (R_ts @ source_centroid.reshape(3,1))

  R_st = R_ts.T
  t_st = -R_st @ t_ts

  H = np.hstack((R_st, t_st))
  H = np.vstack((H, [0,0,0,1]))

  return H

def findDistanceBetweenPCDs(s_pts, t_pts):
  '''Gives the distance between the centroids of two point clouds
  '''
  source_centroid = np.mean(s_pts, axis=0)
  target_centroid = np.mean(t_pts, axis=0)

  dist = np.linalg.norm(source_centroid - target_centroid)

  return dist

# KD Tree correspondences
def findCorrespondences(s_pts, t_pts, source_cloud):
  '''Gives approximate corresponding source points for a given set of 
  target points. For this function, the set of source points >= the set
  of target points.
  '''
  pcd_tree = o3d.geometry.KDTreeFlann(source_cloud)

  s_pts_corr = np.zeros_like(t_pts)
  for i in range(np.size(t_pts, 0)):
    [_, idx, _] = pcd_tree.search_knn_vector_3d(t_pts[i], 1)
    s_pts_corr[i, :] = s_pts[idx, :]

  return s_pts_corr

def draw_registration_result(source, target, transformation):
  source_temp = copy.deepcopy(source)
  target_temp = copy.deepcopy(target)
  source_temp.paint_uniform_color([1, 0.706, 0])
  target_temp.paint_uniform_color([0, 0.651, 0.929])
  source_temp.transform(transformation)
  o3d.visualization.draw_geometries([source_temp, target_temp])