//
// Created by kehan on 2019/8/23.
//

#include <geometry_msgs/PoseStamped.h>
#include "local_planner/smooth_setpoint_position.h"

double_t convertYaw2BetweenFabsPI(double_t _yaw)
{
    while (_yaw > M_PI)
    {
        _yaw -= 2 * M_PI;
    }
    while (_yaw <= -M_PI)
    {
        _yaw += 2 * M_PI;
    }

    return _yaw;
}

int8_t smooth_setpoint_position(geometry_msgs::PoseStamped &_goal_pose, const geometry_msgs::PoseStamped& _cur_pose)
{
    // The vector between goal pose and current pose
    tf::Vector3 vector_3_goal_cur = toTfVector3(subTwoPoint(_goal_pose.pose.position, _cur_pose.pose.position));

    double_t length_threshold = 0.8;
    double_t smoothed_goal_cur_distance =
            vector_3_goal_cur.length() < length_threshold ? vector_3_goal_cur.length() : length_threshold;
    vector_3_goal_cur.normalize();
    vector_3_goal_cur *= smoothed_goal_cur_distance;

    _goal_pose.pose.position.x = _cur_pose.pose.position.x + vector_3_goal_cur.getX();
    _goal_pose.pose.position.y = _cur_pose.pose.position.y + vector_3_goal_cur.getY();
    _goal_pose.pose.position.z = _cur_pose.pose.position.z + vector_3_goal_cur.getZ();


    double_t goal_yaw = tf::getYaw(_goal_pose.pose.orientation);
    double_t cur_yaw = tf::getYaw(_cur_pose.pose.orientation);

    double_t yaw_theta = goal_yaw - cur_yaw;
    yaw_theta = convertYaw2BetweenFabsPI(yaw_theta);

    double_t yaw_threshold = 30 * M_PI / 180.;
    if (fabs(yaw_theta) > yaw_threshold)
    {
        yaw_theta = yaw_theta / fabs(yaw_theta) * yaw_threshold;
    }

    goal_yaw = cur_yaw + yaw_theta;
    goal_yaw = convertYaw2BetweenFabsPI(goal_yaw);
    _goal_pose.pose.orientation = tf::createQuaternionMsgFromYaw(goal_yaw);
}
