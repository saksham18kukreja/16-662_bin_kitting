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

    
    def stitch_pointcloud(self, pc_1, pc_2):
    
        # Perform Go-ICP registration
        result = o3d.pipelines.registration.registration_icp(
            pc_1,
            pc_2,
            max_correspondence_distance=0.1,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000)
        )

        print(f"Tranformation matrix is :{result.transformation}")
        pc_1.transform(result.transformation)
        # o3d.visualization.draw_geometries([pc_2, pc_1])
        return (pc_2 + pc_1)


def main():
    cleaner_ = CleanPointCloud()
    load_dir = "../dataset/ground_truth/Cylinder/Plane_Removed/"
    write_dir = "../dataset/ground_truth/Cylinder/Reconstructed/"
    pc_1 = cleaner_.load_pc(load_dir + '1_.ply')
    pc_2 = cleaner_.load_pc(load_dir + 'o2_1.ply')


    pc_3 = cleaner_.load_pc(load_dir + 'o2_3.ply')
    # pc_4 = cleaner_.load_pc(load_dir + 'o2_2.ply')
    # pc_5 = cleaner_.load_pc(load_dir + '5_.ply')
    


    o3d.visualization.draw_geometries([pc_1, pc_2])
    
    stiched_pc_12= cleaner_.stitch_pointcloud(pc_1, pc_2)
    # o3d.visualization.draw_geometries([stiched_pc_12, pc_3])
    
    cleaner_.visualize_pc(stiched_pc_12)
    
    stiched_pc_123 = cleaner_.stitch_pointcloud(pc_3, stiched_pc_12)

    cleaner_.visualize_pc(stiched_pc_123)
    

    # stiched_pc_1234= cleaner_.stitch_pointcloud(pc_4, stiched_pc_123)

    # cleaner_.visualize_pc(stiched_pc_1234)
    

    # o3d.visualization.draw_geometries([stiched_pc_12, pc_3])

    # stiched_pc_123= cleaner_.stitch_pointcloud(pc_3, stiched_pc_12)
    # o3d.visualization.draw_geometries([pc_3, stiched_pc_123])
    
    # cleaner_.visualize_pc(stiched_pc_123)
    # stiched_pc_1234 = cleaner_.stitch_pointcloud(pc_4, stiched_pc_123)
    # o3d.visualization.draw_geometries([pc_5, stiched_pc_1234])


    # stiched_pc_12345 = cleaner_.stitch_pointcloud(pc_4, stiched_pc_1234)
    # # o3d.visualization.draw_geometries([pc_5, stiched_pc_1234])

    
    # cleaner_.save_pc(stiched_pc_123, write_dir + 'Block.ply')
    
    
    # stiched_pc_2 = cleaner_.stitch_pointcloud(stiched_pc_, pc_2)
    
    
    # cleaner_.visualize_pc(pc_1 + pc_2)
    # exit()
    # cleaner_.visualize_pc(pc_1 + pc_2)s
    # transform_ = np.array([[1, 0, 0, 0],
    #                            [0, 1, 0, 0],
    #                            [0, 0, 1, 0.9], 
    #                            [0, 0, 0, 1]])
    # self.duster_pc_source_ = self.duster_pc_source_.transform(transform_)
        


    
    # for i in range(1, num_pcs+1):
    #     pc_ = cleaner_.load_pc(load_dir + str(i) + '_.ply')
    #     # cleaner_.visualize_pc(pc_)
    #     clean_pc_= cleaner_.remove_outliers(cleaner_.perfom_ransac(pc_))
    #     # cleaner_.visualize_pc(clean_pc_)
    #     cleaner_.save_pc(clean_pc_, write_dir + str(i) + '_.ply')


if __name__ == '__main__':
    main()