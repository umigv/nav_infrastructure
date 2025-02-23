#pragma once

#include "geometry_msgs/msg/pose.hpp"
#include <mutex>

// Manages storing and calculating pose relative to an origin, which can be set and reset an 
// arbitrary number of times; thread-safe
class PoseManager
{
public:

    PoseManager();

    // Sets _origin to the given pose
    void set_origin(const geometry_msgs::msg::Pose &pose);

    // Sets _origin to a default pose at the origin with no rotation
    void reset_origin();

    // Updates the current absolute pose with the given pose
    void update_absolute_pose(const geometry_msgs::msg::Pose &absolute_pose);

    // Calculates and returns the distance from the starting pose to the current absolute pose
    geometry_msgs::msg::Pose get_relative_pose();

    // Returns the current absolute pose
    geometry_msgs::msg::Pose get_absolute_pose();

    // Returns pose1 - pose2
    static geometry_msgs::msg::Pose pose_difference(const geometry_msgs::msg::Pose &pose1, 
        const geometry_msgs::msg::Pose &pose2);

    // Returns a pose with position at (0, 0, 0) and no rotation
    static geometry_msgs::msg::Pose default_pose();

private:

    geometry_msgs::msg::Pose _origin;
    geometry_msgs::msg::Pose _absolute_pose;
    std::mutex _mutex;
};