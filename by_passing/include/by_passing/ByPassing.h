#ifndef CATKIN_WS_BYPASSING_H
#define CATKIN_WS_BYPASSING_H


#include <ros/ros.h>
#include "surface.h"

#include <string>
#include <cmath>
#include <algorithm>
#include <vector>
#include <thread>
#include <chrono>

#include <pcl/filters/passthrough.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>
#include <pcl/features/moment_of_inertia_estimation.h>

#include <Eigen/Geometry>

#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>

#include <sensor_msgs/PointCloud2.h>

// MoveIt
#include "iiwa_ros/iiwa_ros.hpp"

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/ApplyPlanningScene.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometric_shapes/shape_operations.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64.h>
#include <geometry_msgs/WrenchStamped.h>
#include <gazebo_msgs/GetLinkState.h>

#include <tf2/LinearMath/Quaternion.h>

#include "camera.h"
#include "model_control.h"

using moveit::planning_interface::MoveItErrorCode;
using robot_trajectory::RobotTrajectory;

typedef struct _AbdominalModelPlaneSurface
{
    double width, height;
    pcl::PointXYZ origin;
} AbdominalModelPlaneSurface;

typedef struct _AbdominalModelExplorationArea
{
    double width, height;
    pcl::PointXYZ origin;
    pcl::PointXYZ parent_origin;
} AbdominalModelExplorationArea;

typedef struct _TransducerSensor
{
    double width, height;
} TransducerSensor;

typedef struct _AbdominalModelFeatures
{
    std::vector<float> moment_of_inertia;
    std::vector<float> eccentricity;
    pcl::PointXYZ min_point_AABB;
    pcl::PointXYZ max_point_AABB;
    pcl::PointXYZ min_point_OBB;
    pcl::PointXYZ max_point_OBB;
    pcl::PointXYZ position_OBB;
    Eigen::Matrix3f rotational_matrix_OBB;
    float major_value, middle_value, minor_value;
    Eigen::Vector3f major_vector, middle_vector, minor_vector;
    Eigen::Vector3f mass_center;
    double width, height, depth;
    Eigen::Vector3f position;
    Eigen::Quaternionf rotation;
} AbdominalModelFeatures;

class ByPassing
{
public:
    ByPassing(const ros::NodeHandle &nh, const std::string &pointcloud_topic);
    Eigen::Vector3f getAbdominalModelPosition() const;
    Eigen::Quaternionf getAbdominalModelRotation() const;
    void visualizeAbdominalModelPointCloud();
    Eigen::Vector3f getAbdominalModelSize() const;
    double getAbdominalModelCoveragePlaneHeight() const;
    void setTransducerSensor(const TransducerSensor &transducer_sensor);
    void scan(int scan_count=3);


private:
    ros::NodeHandle node_handle;
    std::string camera_topic, pointcloud_topic;
    boost::shared_ptr<Surface> abdominal_surface;
    AbdominalModelFeatures abdominal_model_features;
    AbdominalModelPlaneSurface abdominal_model_plane_surface;
    AbdominalModelExplorationArea abdominal_model_exploration_area;
    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr downsampled_point_cloud;
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;
    pcl::PointCloud<pcl::PointXYZRGBNormal>::Ptr pc_normals;
    pcl::visualization::PCLVisualizer::Ptr viewer;
    std::vector<pcl::PointXYZ> scan_points_estimated;
    std::vector<geometry_msgs::Pose> scan_points_estimated_pose;
    std::vector<pcl::PointXYZ> scan_points_passed;
    TransducerSensor transducer_sensor;
    ros::ServiceClient set_model_state_client, get_model_state_client;
    geometry_msgs::Pose kinect_start_pose;
    geometry_msgs::Twist kinect_start_twist;
    ros::Publisher marker_pub;
    int marker_id = 0;
    geometry_msgs::PoseStamped end_effector_init_pose;
    int scan_count;
    boost::shared_ptr<Camera> camera;
    boost::shared_ptr<ModelControl> model_control;
    std::string home_directory;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> group;
    ros::Publisher tissues_scan_pub;
    ros::ServiceClient take_photo_srv, execute_ultrasound_srv;
    ros::ServiceClient tissue_names_srv;
    ros::ServiceClient take_scan_srv;
    ros::ServiceClient merge_scans_srv;
    std::vector<std::string> tissue_names;
    ros::ServiceClient ray_intersect_srv;
    void explorePoints();
};

#endif //CATKIN_WS_BYPASSING_H