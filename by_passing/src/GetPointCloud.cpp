#include "GetPointCloud.h"
#include <random>
#include <iterator>

GetPointCloud::GetPointCloud(double x_min_pos, double x_max_pos, double y_min_pos, double y_max_pos, double z_min_pos, double z_max_pos)
        : point_cloud(new pcl::PointCloud<pcl::PointXYZ>),
          full_point_cloud(new pcl::PointCloud<pcl::PointXYZ>),
          tf_listener(tf_buffer)
{
    //sleep(20);
    search_radius = 0.03;
    exploration_point = 0;
    this->x_min_pos = x_min_pos;
    this->x_max_pos = x_max_pos;
    this->y_min_pos = y_min_pos;
    this->y_max_pos = y_max_pos;
    this->z_min_pos = z_min_pos;
    this->z_max_pos = z_max_pos;

}
void GetPointCloud::get_point_cloud() {
    sensor_msgs::PointCloud2 msgs_point_cloud;
    boost::shared_ptr<sensor_msgs::PointCloud2 const> sharedPtr;
    sharedPtr = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pointcloud_topic, ros::Duration(1000));
    if (sharedPtr == NULL) {
        ROS_INFO("UNABLE TO GET POINT CLOUD MESSAGE!");
        point_cloud_validity = false;
    } else {
        msgs_point_cloud = *sharedPtr;
        full_point_cloud = filter_point_cloud(remove_invalid_points(from_msgs_to_pcl(msgs_point_cloud)));
        point_cloud = downsample_point_cloud(full_point_cloud);
        if (point_cloud->points.size() == 0) {
            point_cloud_validity = false;
        } else {
            ROS_INFO("POINT CLOUD DATA ACQUIRED!");
            point_cloud_validity = true;
        }
    }
}
