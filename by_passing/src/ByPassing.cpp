#include "ByPassing.h"
#include <random>
#include <iterator>
#include <gazebo_msgs/ModelState.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/GetModelState.h>
#include <urdf_parser/urdf_parser.h>
#include <ros/package.h>
#include <sensor_msgs/Image.h>
#include <ultrasound_sensor/ArrayImages.h>
#include <ultrasound_sensor/TissueNames.h>
#include <ultrasound_sensor/TakeScan.h>
#include <ultrasound_sensor/RayIntersect.h>

#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>

#include <std_srvs/Empty.h>
#include <bypassing.h>

// spdlog
#include "spdlog/spdlog.h"
#include "spdlog/cfg/env.h"

ByPassing::ByPassing(const ros::NodeHandle &nh, const std::string &pointcloud_topic):
        node_handle(nh), pointcloud_topic(pointcloud_topic) {

}


void ByPassing::explorePoints()
{
    std::size_t points_passed = 0;
    std::size_t point_id = 0;
    std::size_t point_id_need_organ = 0;
    bool is_need_organ = false
    bool exploration_plan_success = false;
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    ros::Duration half_sec(0.5);
    while(is_need_organ) 
    {
    	point_id_need_organ++;
    }
    while(points_passed < point_id_need_organ)
    {
        // Fill point's position to explore
        /*estimated_point_pose.position.x = exploration_point->x;
        estimated_point_pose.position.y = exploration_point->y;
        estimated_point_pose.position.z = exploration_point->z;*/
        auto estimated_point_pose = abdominal_surface->get_next_random_pose();

        // Ultrasound sensor is always directed at an angle of 90 degrees to the abdomen
        // Get a quaternion of 90 degrees
        double roll, pitch, yaw;
        tf::Quaternion quat(
                estimated_point_pose.orientation.x,
                estimated_point_pose.orientation.y,
                estimated_point_pose.orientation.z,
                estimated_point_pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // Set the quaternion
        estimated_point_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                roll, pitch, 1.57
        );

        // MoveIt configuration
        group->clearPathConstraints();
        group->allowReplanning(true);

        // Set target pose for end-effector
        group->setStartStateToCurrentState();
        group->setPoseTarget(estimated_point_pose);

        // Plan end-effector's movement
        exploration_plan_success = (group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing plan (pose goal) %s", exploration_plan_success ? "" : "FAILED");

        // Execute end-effector's movement
//in the ultrasound, all the organs are hidden one by one, but I need all but n to be hidden
//points_passed responsible for hiding the organ, I need it to accumulate until the right organ is found
//the problem is that I do not know how to designate each organ id so that the hiding cycle is interrupted on this particular organ
//we have point_id - this is our our organ account 

//I need it to accumulate until the right organ is found
        if(exploration_plan_success) {
            ROS_WARN("Motion plan successfull for point %d!", point_id);
            points_passed++;
            ros::spinOnce();
            // Publish marker
            publishMarker(estimated_point_pose);
            group->execute(plan);
            makeUltrasoundWave(points_passed, ros::Time::now());
            point_id++;
        }
        else ROS_WARN("Failed to plan motion for point %d!", point_id);
    }
    while(points_passed > point_id_need_organ)
    {
        // Fill point's position to explore
        /*estimated_point_pose.position.x = exploration_point->x;
        estimated_point_pose.position.y = exploration_point->y;
        estimated_point_pose.position.z = exploration_point->z;*/
        auto estimated_point_pose = abdominal_surface->get_next_random_pose();

        // Ultrasound sensor is always directed at an angle of 90 degrees to the abdomen
        // Get a quaternion of 90 degrees
        double roll, pitch, yaw;
        tf::Quaternion quat(
                estimated_point_pose.orientation.x,
                estimated_point_pose.orientation.y,
                estimated_point_pose.orientation.z,
                estimated_point_pose.orientation.w);
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
        // Set the quaternion
        estimated_point_pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(
                roll, pitch, 1.57
        );

        // MoveIt configuration
        group->clearPathConstraints();
        group->allowReplanning(true);

        // Set target pose for end-effector
        group->setStartStateToCurrentState();
        group->setPoseTarget(estimated_point_pose);

        // Plan end-effector's movement
        exploration_plan_success = (group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
        ROS_INFO("Visualizing plan (pose goal) %s", exploration_plan_success ? "" : "FAILED");

        // Execute end-effector's movement
//in the ultrasound, all the organs are hidden one by one, but I need all but n to be hidden
//points_passed responsible for hiding the organ, I need it to accumulate until the right organ is found
//the problem is that I do not know how to designate each organ id so that the hiding cycle is interrupted on this particular organ
//we have point_id - this is our our organ account 

I need it to accumulate until the right organ is found
        if(exploration_plan_success) {
            ROS_WARN("Motion plan successfull for point %d!", point_id);
            points_passed++;
            ros::spinOnce();
            // Publish marker
            publishMarker(estimated_point_pose);
            group->execute(plan);
            makeUltrasoundWave(points_passed, ros::Time::now());
            point_id++;
        }
        else ROS_WARN("Failed to plan motion for point %d!", point_id);
    }

    // Returning to init position
    // Set target pose for end-effector
    group->setStartStateToCurrentState();
    group->setPoseTarget(end_effector_init_pose);
    // Plan end-effector's movement
    exploration_plan_success = (group->plan(plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    ROS_INFO("Visualizing plan (pose goal) %s", exploration_plan_success ? "" : "FAILED");
    // Execute end-effector's movement
    if(exploration_plan_success) {
        ROS_INFO("Motion plan of returning successfull!");
        ros::spinOnce();
        group->execute(plan);
    }
    else ROS_INFO("Failed to plan motion for returning to base!");
}
