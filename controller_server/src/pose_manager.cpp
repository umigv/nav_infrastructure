#include "controller_server/pose_manager.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <iostream>

PoseManager::PoseManager()
{
    set_origin(default_pose());
    update_absolute_pose(default_pose());
}

void PoseManager::set_origin(const geometry_msgs::msg::Pose &pose)
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::cout << "Setting origin to " << pose.position.x << ", " << pose.position.y << std::endl;
    _origin = pose;
}

void PoseManager::reset_origin()
{
    std::lock_guard<std::mutex> lock(_mutex);
    _origin = default_pose();
}

void PoseManager::update_absolute_pose(const geometry_msgs::msg::Pose &absolute_pose)
{
    std::lock_guard<std::mutex> lock(_mutex);
    _absolute_pose = absolute_pose;
}

geometry_msgs::msg::Pose PoseManager::get_relative_pose()
{
    std::lock_guard<std::mutex> lock(_mutex);
    std::cout << "Calculating difference between " << _absolute_pose.position.x << ", " << _absolute_pose.position.y << " and " << _origin.position.x << ", " << _origin.position.y << std::endl;
    return pose_difference(_absolute_pose, _origin);
}   

geometry_msgs::msg::Pose PoseManager::get_absolute_pose()
{
    std::lock_guard<std::mutex> lock(_mutex);
    return _absolute_pose;
}

geometry_msgs::msg::Pose PoseManager::pose_difference(const geometry_msgs::msg::Pose &pose1, 
    const geometry_msgs::msg::Pose &pose2)
{
    geometry_msgs::msg::Pose diff;
    diff.position.x = pose1.position.x - pose2.position.x;
    diff.position.y = pose1.position.y - pose2.position.y;
    diff.position.z = pose1.position.z - pose2.position.z;
    
    tf2::Quaternion q1, q2;
    tf2::fromMsg(pose1.orientation, q1);
    tf2::fromMsg(pose2.orientation, q2);
    
    tf2::Quaternion q_diff = q2 * q1.inverse();        
    q_diff.normalize();

    diff.orientation = tf2::toMsg(q_diff);
    return diff;
}

geometry_msgs::msg::Pose PoseManager::default_pose()
{
    geometry_msgs::msg::Pose pose;
    pose.position.x = 0;
    pose.position.y = 0;
    pose.position.z = 0;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;
    return pose;
}