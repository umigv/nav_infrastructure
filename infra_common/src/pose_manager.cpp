#include "infra_common/pose_manager.hpp"

#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

#include <iostream>

PoseManager::PoseManager()
{
    set_origin(default_pose());
}

void PoseManager::set_origin(const geometry_msgs::msg::Pose &pose)
{
    std::lock_guard<std::mutex> lock(_pose_lock);
    std::cout << "Setting origin to " << pose.position.x << ", " << pose.position.y << std::endl;
    _origin = pose;
}

void PoseManager::reset_origin()
{
    std::lock_guard<std::mutex> lock(_pose_lock);
    _origin = default_pose();
}

void PoseManager::update_absolute_pose(const geometry_msgs::msg::Pose &absolute_pose)
{
    std::lock_guard<std::mutex> lock(_pose_lock);
    if (!_initial_pose_set)
    {
        _initial_pose_set = true;
        _pose_cv.notify_all();
    }
    _absolute_pose = absolute_pose;
}

geometry_msgs::msg::Pose PoseManager::get_relative_pose()
{
    std::unique_lock<std::mutex> lock(_pose_lock);
    if (!_initial_pose_set)
    {
        wait_for_pose_update(lock);
    }
    return pose_difference(_absolute_pose, _origin);
}   

geometry_msgs::msg::Pose PoseManager::get_absolute_pose()
{
    std::unique_lock<std::mutex> lock(_pose_lock);
    if (!_initial_pose_set)
    {
        wait_for_pose_update(lock);
    }
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

void PoseManager::wait_for_pose_update(std::unique_lock<std::mutex> &pose_lock) 
{
    _pose_cv.wait(pose_lock, [this] { return _initial_pose_set; });
}