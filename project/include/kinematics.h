// 260820_estimate_robot_pose_final/kinematics.h

#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "robot.h"

//function
void Robot_init(Robot* robot, Pose input_pose);

car_velocity FK_wheel2car(Robot* robot, wheel_velocity* wheel);
wheel_velocity IK_car2wheel(Robot* robot, car_velocity* car);
car_velocity IK_world2car(Robot* robot, world_velocity* world);
world_velocity FK_car2world(Robot* robot, car_velocity* car);

world_velocity odometry_update(Robot* robot, wheel_velocity estimated_wheel_v);
void pose_update(Robot* robot, world_velocity odometry, Pose gyro_pose);
Pose pose_get(Robot *robot);

#endif
