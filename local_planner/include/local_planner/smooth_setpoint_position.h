//
// Created by kehan on 2019/8/23.
//

#ifndef LOCAL_PLANNER_SMOOTH_SETPOINT_POSITION_H_
#define LOCAL_PLANNER_SMOOTH_SETPOINT_POSITION_H_

#include <tf/transform_datatypes.h>
#include <geometry_msgs/PoseStamped.h>

template <typename P1, typename P2>
P1 subTwoPoint(const P1& _p_1, const P2& _p_2)
{
    P1 ans_p;
    ans_p.x = _p_1.x - _p_2.x;
    ans_p.y = _p_1.y - _p_2.y;
    ans_p.z = _p_1.z - _p_2.z;

    return ans_p;
}

template <typename P>
tf::Vector3 toTfVector3(const P& point)
{
    return tf::Vector3(point.x, point.y, point.z);
}

int8_t smooth_setpoint_position(geometry_msgs::PoseStamped &_goal_pose, const geometry_msgs::PoseStamped& _cur_pose);



#endif //LOCAL_PLANNER_SMOOTH_SETPOINT_POSITION_H_
