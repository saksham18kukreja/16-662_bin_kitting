import open3d as o3d
import numpy as np

# Base directory for .ply file
ply_base_dir = "/home/shashwat/Documents/MRSD/Semester_2/16662-RobotAutonomy/Project/16-662_bin_kitting/"


# Function to load pointcloud from file
def load_pc(file_path_):
    return o3d.io.read_point_cloud(file_path_)

def perfom_ransac(input_pc_):
    # Apply RANSAC plane segmentation
    plane_model, inliers = input_pc_.segment_plane(distance_threshold= 0.005, 
                                                 ransac_n=3, 
                                                 num_iterations=1000)
    # Extract outlier plane(which is our object)
    return input_pc_.select_by_index(inliers, invert=True)

# Use this function to remove outliers after performing RANSAC
def remove_outliers(input_pc_):
    output_pc_, ind = input_pc_.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
    return output_pc_


# Load point-clouds
duster_pc_source_ = load_pc(ply_base_dir + "duster_gt.ply")



duster_pc_target_= load_pc(ply_base_dir + "duster_val1.ply")


# Paint point-clouds
duster_pc_source_.paint_uniform_color([0.3, 0.9, 0.3])
# Paint point-clouds
duster_pc_target_.paint_uniform_color([0.9, 0.3, 0.3])


# Visualize both pointclouds
o3d.visualization.draw_geometries([duster_pc_source_])
o3d.visualization.draw_geometries([duster_pc_target_])



# Perform ransac and outlier rejection
duster_pc_source_filtered_ = remove_outliers(perfom_ransac(duster_pc_source_))
duster_pc_target_filtered_ = remove_outliers(perfom_ransac(duster_pc_target_))


# Perform Go-ICP registration
result = o3d.pipelines.registration.registration_icp(
    duster_pc_source_filtered_,
    duster_pc_target_filtered_,
    max_correspondence_distance=0.05,
    estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
    criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000)
)

print(f"Tranformation matrix is :{result.transformation}")

source_pc_transformed = duster_pc_source_filtered_.transform(result.transformation)
o3d.visualization.draw_geometries([duster_pc_target_filtered_, source_pc_transformed])

