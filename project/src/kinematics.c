// 260820_estimate_robot_pose_final/kinematics.c


#include <math.h>
#include "robot.h"
#include "timer.h"
#include "kinematics.h"

void Robot_init(
	Robot* robot,
	Pose input_pose)
{
	robot->pose = input_pose;
	robot->wheel_radius=30;
	robot->wheel_to_center=120;
}


// kinematics
car_velocity FK_wheel2car(Robot* robot,wheel_velocity* wheel){
	car_velocity car;

	car.v_x= (robot->wheel_radius * (wheel->u_L + wheel->u_R)) / 2.0f;
	car.v_y=0;
	car.w= (robot->wheel_radius * (wheel->u_R - wheel->u_L)) / (2.0f * robot->wheel_to_center);
	
	
	return car;
}

wheel_velocity IK_car2wheel(Robot* robot, car_velocity* car){
	wheel_velocity wheel;
	wheel.u_L  = (car->v_x - robot->wheel_to_center * car->w) / robot->wheel_radius;
	wheel.u_R = (car->v_x + robot->wheel_to_center * car->w) / robot->wheel_radius;
	
	return wheel;
}

car_velocity IK_world2car(Robot* robot, world_velocity* world){
	car_velocity car;
	car.v_x = world->x_dot * cosf(robot->pose.phi) + world->y_dot * sinf(robot->pose.phi);
	car.v_y = -world->x_dot * sinf(robot->pose.phi) + world->y_dot * cosf(robot->pose.phi);
	car.w = world->phi_dot;

	return car;
}

world_velocity FK_car2world(Robot* robot, car_velocity* car){
	world_velocity world;
	world.phi_dot = car->w;
	world.x_dot = car->v_x * cosf(robot->pose.phi);
	world.y_dot = car->v_x * sinf(robot->pose.phi);

	return world;
}

static float wrap_pi(float angle)
{
	while (angle > M_PI) {
		angle -= 2.0f * M_PI;
	}
	while (angle < -M_PI) {
		angle += 2.0f * M_PI;
	}
	return angle;
}

void pose_update(Robot* robot, world_velocity odometry, Pose gyro_pose)
{
	const float dt = 0.020f;
	const float gyro_weight = 0.98f;
	float odometry_phi;
	float gyro_error;

	/* x/y are supplied by wheel odometry.  Heading uses a complementary
	 * correction so encoder information remains available if the gyro drifts. */
	robot->pose.x += dt * odometry.x_dot;
	robot->pose.y += dt * odometry.y_dot;

	odometry_phi = wrap_pi(robot->pose.phi + dt * odometry.phi_dot);
	gyro_error = wrap_pi(gyro_pose.phi - odometry_phi);
	robot->pose.phi = wrap_pi(odometry_phi + gyro_weight * gyro_error);
}

world_velocity odometry_update(Robot* robot, wheel_velocity estimated_wheel_v)
{
	car_velocity estimated_car_v;

	estimated_car_v = FK_wheel2car(robot,&estimated_wheel_v);
	return FK_car2world(robot,&estimated_car_v);
}

Pose pose_get(Robot *robot){
	return robot->pose;
}
