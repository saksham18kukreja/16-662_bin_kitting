# This code loades the point-clouds and removes the plane and saves it in the given dir
import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class CleanPointCloud:
    def __init__(self):
        print(f"[INFO] Clean-PointCloud Class Initiated")

    # Function to Visualize PointCloud
    def visualize_pc(self, pc_):
        o3d.visualization.draw_geometries([pc_])

    # Function to load pointcloud from file
    def load_pc(self, file_path):
        return o3d.io.read_point_cloud(file_path)

    def perfom_ransac(self, input_pc_):
        # Apply RANSAC plane segmentation
        plane_model, inliers = input_pc_.segment_plane(distance_threshold= 0.005, 
                                                    ransac_n=3, 
                                                    num_iterations=1000)
        # Extract outlier plane(which is our object)
        return input_pc_.select_by_index(inliers, invert=True)

    # Use this function to remove outliers after performing RANSAC
    def remove_outliers(self,input_pc_):
        output_pc_, ind = input_pc_.remove_statistical_outlier(nb_neighbors=50, std_ratio=1.0)
        return output_pc_

    # Function to save point cloud to file
    def save_pc(self, pc_, file_path):
        o3d.io.write_point_cloud(file_path, pc_)
        print(f"Point cloud saved to {file_path}")


def main():
    cleaner_ = CleanPointCloud()
    load_dir = "../dataset/ground_truth/Cylinder/Manual_Cleaned/"
    write_dir = "../dataset/ground_truth/Cylinder/Plane_Removed/"
    # Number of point-clouds
    num_pcs = 5

    for i in range(1, num_pcs+1):
        pc_ = cleaner_.load_pc(load_dir + str(i) + '_.ply')
        cleaner_.visualize_pc(pc_)
        clean_pc_= cleaner_.remove_outliers(cleaner_.perfom_ransac(pc_))
        cleaner_.visualize_pc(clean_pc_)
        cleaner_.save_pc(clean_pc_, write_dir + str(i) + '_.ply')


if __name__ == '__main__':
    main()