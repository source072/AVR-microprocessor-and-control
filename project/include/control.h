/*
 * control.h
 *
 * Created: 2026-09-07 오후 10:11:11
 *  Author: ijaew
 */ 


#ifndef CONTROL_H_
#define CONTROL_H_

#include "robot.h"

float normalize_angle_pi(float angle);
wheel_velocity control(Robot *robot,
Pose current_pose,
Pose target_pose);



#endif /* CONTROL_H_ */