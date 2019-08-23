//
// Created by kehan on 2019/8/23.
//

#include "local_planner/smooth_setpoint_position.h"

int8_t smooth_setpoint_position(geometry_msgs::PoseStamped &_goal_pose, const geometry_msgs::PoseStamped& _cur_pose)
{
    // The vector between goal pose and current pose
    tf::Vector3 vector_3_goal_cur = toTfVector3(subTwoPoint(_goal_pose.pose.position, _cur_pose.pose.position));

    double_t length_threshold = 1.0;
    double_t smoothed_goal_cur_distance =
            vector_3_goal_cur.length() < length_threshold ? vector_3_goal_cur.length() : length_threshold;
    vector_3_goal_cur.normalize();
    vector_3_goal_cur *= smoothed_goal_cur_distance;

    _goal_pose.pose.position.x = _cur_pose.pose.position.x + vector_3_goal_cur.getX();
    _goal_pose.pose.position.y = _cur_pose.pose.position.y + vector_3_goal_cur.getY();
    _goal_pose.pose.position.z = _cur_pose.pose.position.z + vector_3_goal_cur.getZ();
}
