// 260820_estimate_robot_pose_final/robot.h

#ifndef ROBOT_H
#define ROBOT_H

typedef struct {
	float phi_dot;
	float x_dot;
	float y_dot;
} world_velocity;

typedef struct {
	float w;
	float v_x;
	float v_y;
} car_velocity;

typedef struct {
	float u_L;
	float u_R;
} wheel_velocity;

typedef struct {
	float phi;
	float x;
	float y;
} Pose;

typedef struct{
	Pose pose;
	int wheel_radius; // 3cm,30mm
	int wheel_to_center; //12cm,120mm
} Robot;

#endif