/*
 * trajectory.c
 *
 * Created: 2026-09-07 오후 8:08:28
 *  Author: ijaew
 */ 
#include <avr/io.h>
#include <avr/interrupt.h>
#include <math.h>

#include "trajectory.h"
#include "robot.h"
#include "kinematics.h"


Pose* make_goal(Pose start_pose, Pose goal_pose)
{
	static Pose goal[4];

	float dx;
	float dy;
	float goal_direction_phi;

	dx = goal_pose.x - start_pose.x;
	dy = goal_pose.y - start_pose.y;
	goal_direction_phi = atan2(dy, dx);

	// 0. Start pose
	goal[0] = start_pose;

	// 1. inter_pose1 (first_rotation 목표)
	goal[1].x   = start_pose.x;
	goal[1].y   = start_pose.y;
	goal[1].phi = goal_direction_phi;

	// 2. inter_pose2 (translation 목표)
	goal[2].x   = goal_pose.x;
	goal[2].y   = goal_pose.y;
	goal[2].phi = goal_direction_phi;

	// 3. goal_pose (final_rotatoin 목표)
	goal[3] = goal_pose;

	return goal;
}


float* estimate_total_T(Pose* Poses, float w_max, float v_max)
{
	static float T[3];
	float first_rotation;
	float distance;
	float final_rotation;

	first_rotation = Poses[1].phi - Poses[0].phi;

	distance = sqrt(
	(Poses[2].x - Poses[1].x) * (Poses[2].x - Poses[1].x)
	+
	(Poses[2].y - Poses[1].y) * (Poses[2].y - Poses[1].y)
	);

	final_rotation = Poses[3].phi - Poses[2].phi;

	// 결과를 ms 단위로 저장
	T[0] = 1.5f * fabs(first_rotation) / w_max;
	T[1] = 1.5f * distance / v_max ;
	T[2] = 1.5f * fabs(final_rotation) / w_max;

	return T;
}

Pose* make_target(Pose step_start_pose, Pose step_goal_pose, float t, float step_total_T )
{
	static Pose target;

	float tau = t /step_total_T;
	
	if (tau > 1.0f)
	tau = 1.0f;
	if (tau < 0.0f)
	tau = 0.0f;
	
	float s = 3.0f * tau * tau - 2.0f * tau * tau * tau;

	// target pose
	 target.x=
	step_start_pose.x
	+ s * (step_goal_pose.x - step_start_pose.x);

	target.y =
	step_start_pose.y
	+ s * (step_goal_pose.y - step_start_pose.y);

	target.phi =
	step_start_pose.phi
	+ s * (step_goal_pose.phi - step_start_pose.phi);

	return &target;
}