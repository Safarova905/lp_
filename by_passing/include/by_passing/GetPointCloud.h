#ifndef CATKIN_WS_GETPOINTCLOUD_H
#define CATKIN_WS_GETPOINTCLOUD_H
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <Eigen/Geometry>

#include <pcl/common/geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/filter.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>

#include <string>
#include <cmath>
#include <algorithm>
#include <vector>

#include "marker.h"
#include "pose_generator.h"

using namespace std;
//using namespace Eigen;

class GetPointCloud {

private:
    ros::NodeHandle nh;

    std::string pointcloud_topic = "/camera/depth/points";

    bool point_cloud_validity;
    float search_radius;

    int exploration_point;
    std::vector<int> exploration_point_order;

    Vector4f plane_parameters;
    Vector3d normal_vector;

    pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr full_point_cloud;
    pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;

    Marker marker;
    PoseGenerator pose_generator;

    tf2_ros::Buffer tf_buffer;
    tf2_ros::TransformListener tf_listener;

    double x_min_pos;
    double x_max_pos;
    double y_min_pos;
    double y_max_pos;
    double z_min_pos;
    double z_max_pos;

    void get_point_cloud();

public:
    GetPointCloud(double x_min_pos, double x_max_pos, double y_min_pos, double y_max_pos, double z_min_pos, double z_max_pos);
};
#endif //CATKIN_WS_GETPOINTCLOUD_H
