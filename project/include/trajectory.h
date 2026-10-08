/*
 * trajectory.h
 *
 * Created: 2026-09-07 오후 8:08:54
 *  Author: ijaew
 */ 


#ifndef TRAJECTORY_H_
#define TRAJECTORY_H_

#include "robot.h"



Pose* make_goal(Pose start_pose, Pose goal_pose);

Pose* make_target(Pose step_start_pose,
Pose step_goal_pose,
float t,
float step_total_T);

float* estimate_total_T(Pose* poses,
float velocity,
float angular_velocity);



#endif /* TRAJECTORY_H_ */