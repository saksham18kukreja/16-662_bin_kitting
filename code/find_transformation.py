import open3d as o3d
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
from sensor_msgs import point_cloud2


class MatchPointCloud:
    def __init__(self):
        # Base directory for .ply file
        self.ply_base_dir = "../dataset/"

        # Load ground-truths (currently just duster point-clouds)
        self.duster_pc_source_ = self.load_pc(self.ply_base_dir + "ground_truth/" + "duster_gt.ply")
        # Paint ground-truth point-cloud
        self.duster_pc_source_.paint_uniform_color([0.9, 0.9, 0.3])
        
        
        transform_ = np.array([[1, 0, 0, 0],
                               [0, 1, 0, 0],
                               [0, 0, 1, 0.9], 
                               [0, 0, 0, 1]])
        self.duster_pc_source_ = self.duster_pc_source_.transform(transform_)
        
        # Visualize ground-truth pointcloud
        self.visualize_pc(self.duster_pc_source_)
        # Perform ransac and outlier rejection
        self.duster_pc_source_ = self.remove_outliers(self.perfom_ransac(self.duster_pc_source_))

        # # Code to test with a validation ply
        # self.duster_target_pc_ = self.load_pc(self.ply_base_dir + "validation/" + "duster_validation1.ply") 
        # # Paint ground-truth point-cloud
        # self.duster_target_pc_.paint_uniform_color([0.9, 0.3, 0.3])
        # self.visualize_pc(self.duster_target_pc_)

        # self.match_pointcloud(self.duster_target_pc_) 
        

    # Function to Visualize PointCloud
    def visualize_pc(self, pc_):
        o3d.visualization.draw_geometries([pc_])

    # Function to load pointcloud from file
    def load_pc(self, file_path_):
        return o3d.io.read_point_cloud(file_path_)

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

    def add_noise(self):
        # Define the rotation angle in radians (90 degrees)
        theta = np.pi / 4

        # Define the rotation matrix
        R = np.array([[np.cos(theta), -np.sin(theta), 0],
                    [np.sin(theta), np.cos(theta), 0],
                    [0, 0, 1]])

        # Apply rotation to the point cloud
        self.duster_pc_source_.rotate(R)

    # TODO(Make the function match with other ground-truth pc's)
    def match_pointcloud(self, input_pc_):
        # TODO(Remove, just for testing): Add noise
        self.add_noise()

        target_pc_= self.remove_outliers(self.perfom_ransac(input_pc_))

        o3d.visualization.draw_geometries([self.duster_pc_source_, target_pc_])
        
        # Perform Go-ICP registration
        result = o3d.pipelines.registration.registration_icp(
            self.duster_pc_source_,
            target_pc_,
            max_correspondence_distance=0.1,
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPoint(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=5000)
        )

        print(f"Tranformation matrix is :{result.transformation}")
        o3d.visualization.draw_geometries([target_pc_, self.duster_pc_source_.transform(result.transformation)])


# Class to convert ROS PointCloud to O3D Type 
class ROS_PointCloudProcessor:
    def __init__(self):
        rospy.init_node('find_transformation')
        rospy.Subscriber('/camera/depth/color/points', PointCloud2, self.point_cloud_callback)

        self.input_pc_ =  o3d.geometry.PointCloud()
        self.pc_received_ = False

    def point_cloud_callback(self, msg):
        # Convert ROS point cloud message to numpy array
        points = []
        for p in point_cloud2.read_points(msg, skip_nans=True):
            points.append([p[0], p[1], p[2]])
        np_points = np.array(points)

        # Change to Open3D point-cloud object
        self.input_pc_.points = o3d.utility.Vector3dVector(np_points)
        
        #TODO: Remove flag and find a better way
        self.pc_received_ = True

    def get_pointcloud(self):
        return self.ros_pc_input_
    
    
    def run(self):
        rospy.spin()


def main():
    # Conver ROS input to o3d object type 
    ros_processor = ROS_PointCloudProcessor()

    # ros_processor.run()

    # Match pointcloud
    pc_matcher = MatchPointCloud()
    

    while not rospy.is_shutdown():
        if ros_processor.pc_received_ is True:
            print("PointCloud Received")
            pc_matcher.visualize_pc(ros_processor.input_pc_)
            # Match point-cloud
            pc_matcher.match_pointcloud(ros_processor.input_pc_)
            
            # Change Flag back to false 
            ros_processor.pc_received_ = False        
        else:
            print("Waiting for ROS PointCloud")
        

        rospy.sleep(0.1)  
    
    print("Successfully Exited the code")

if __name__ == '__main__':
    main()