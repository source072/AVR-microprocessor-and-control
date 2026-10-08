#include <math.h>
#include "control.h"
#include "robot.h"
#include "kinematics.h"
#include "stdint.h"
#include "stdio.h"





float normalize_angle_pi(float angle)
{
	while (angle > M_PI)
	angle -= 2.0f * M_PI;

	while (angle < -M_PI)
	angle += 2.0f * M_PI;

	return angle;
}

/*
//PID control.ver
wheel_velocity control(Pose current_pose, Pose target_pose)
{
	wheel_velocity wheel_v;
	float dt = 0.020f;
					

	static float prev_error_tran_sum = 0.0f;
	static float prev_error_rot_sum = 0.0f;

	static float integral_tran = 0.0f;
	static float integral_rot = 0.0f;

	// PID control gain value
	const float KP_tran = 0.50f;
	const float KI_tran = 0.08f;
	const float KD_tran = 0.002f;

	const float KP_rot =170.0f;
	const float KI_rot = 120.0f; 
	const float KD_rot = 10.0f;
	
	const float rot_offset = 3.0f;

	// error calculate
	float error_x = target_pose.x - current_pose.x;
	float error_y = target_pose.y - current_pose.y;
	float error_phi =	normalize_angle_pi(target_pose.phi - current_pose.phi);

	// translation / rotation error
	float error_tran_sum =cosf(current_pose.phi) * error_x + sinf(current_pose.phi) * error_y;
	float error_rot_sum = error_phi;


	// integral error
	integral_tran += error_tran_sum * dt;
	integral_rot += error_rot_sum * dt;
	//printf("integral_tran: %d\n',(int)integral_tran);

	// integral windup protection
	if (integral_tran > 1000.0f)
	integral_tran = 1000.0f;
	
	if (integral_tran < -1000.0f)
	integral_tran = -1000.0f;
	
	if (integral_rot > 10.0f)
	integral_rot = 10.0f;
	
	if (integral_rot < -10.0f)
	integral_rot = -10.0f;


	// derivative error
	float d_error_tran_sum =(error_tran_sum - prev_error_tran_sum) / dt;
	float d_error_rot_sum = (error_rot_sum - prev_error_rot_sum) / dt;


	// PID control output
	float tran_value =
	KP_tran * error_tran_sum +
	KI_tran * integral_tran +
	KD_tran * d_error_tran_sum;

	float rot_value =
	(KP_rot * error_rot_sum +
	KI_rot * integral_rot +
	KD_rot * d_error_rot_sum)*rot_offset;


	// wheel velocity
	wheel_v.u_L =
	tran_value - rot_value;

	wheel_v.u_R =
	tran_value + rot_value;


	// previous error save
	prev_error_tran_sum = error_tran_sum;
	prev_error_rot_sum = error_rot_sum;


	return wheel_v;
}
*/

// GPT generate
wheel_velocity control(Robot *robot,
                       Pose current_pose,
                       Pose target_pose)
{
    const float dt = 0.020f;


    /* ==============================
       이전 target pose
       ============================== */

    static Pose prev_target_pose;
    static uint8_t first_run = 1;


    /* ==============================
       PID 상태값
       ============================== */

    static float prev_error_tran = 0.0f;
    static float prev_error_rot  = 0.0f;

    static float integral_tran = 0.0f;
    static float integral_rot  = 0.0f;


    /* ==============================
       PID Gain
       ============================== */
	const float KP_tran = 2.5f;
	const float KI_tran = 0.00f;
	const float KD_tran = 0.01f;

	const float KP_rot  = 75.0f;
	const float KI_rot  = 0.00f;
	const float KD_rot  = 0.05f;

    /* 첫 호출 */
    if (first_run) //
    {
        prev_target_pose = target_pose;
        first_run = 0;
    }


    /* =====================================================
       1. Target trajectory 자체의 reference velocity
       ===================================================== */

    float target_dx =
        target_pose.x - prev_target_pose.x;

    float target_dy =
        target_pose.y - prev_target_pose.y;

    float target_dphi =
        normalize_angle_pi(
            target_pose.phi - prev_target_pose.phi
        );


    /*
        target trajectory의 이동량을
        로봇 진행방향 성분으로 변환
    */
    float target_ds =
        cosf(target_pose.phi) * target_dx
        +
        sinf(target_pose.phi) * target_dy;


    /* Feedforward reference */
    float v_ref = target_ds / dt;
    float w_ref = target_dphi / dt;

    /* =====================================================
       2. current pose - target pose Error
       ===================================================== */

    float error_x =
        target_pose.x - current_pose.x;

    float error_y =
        target_pose.y - current_pose.y;

    float error_phi =
        normalize_angle_pi(
            target_pose.phi - current_pose.phi
        );


    /*
        위치 error도 로봇 진행방향 성분으로 변환
    */
    float error_tran =
        cosf(current_pose.phi) * error_x
        +
        sinf(current_pose.phi) * error_y;

    float error_rot = error_phi;


    /* =====================================================
       3. PID
       ===================================================== */

    integral_tran += error_tran * dt;
    integral_rot  += error_rot  * dt;


    /* Integral windup */
    if (integral_tran > 1000.0f)
        integral_tran = 1000.0f;

    if (integral_tran < -1000.0f)
        integral_tran = -1000.0f;

    if (integral_rot > 10.0f)
        integral_rot = 10.0f;

    if (integral_rot < -10.0f)
        integral_rot = -10.0f;


    float d_error_tran =
        (error_tran - prev_error_tran) / dt;
		
	
    float d_error_rot =
        (error_rot - prev_error_rot) / dt;

    float pid_tran =
        KP_tran * error_tran
        +
        KI_tran * integral_tran
        +
        KD_tran * d_error_tran;


    float pid_rot =
        KP_rot * error_rot
        +
        KI_rot * integral_rot
        +
        KD_rot * d_error_rot;


    /* =====================================================
       4. Reference + PID correction
       ===================================================== */

    car_velocity cmd_v;

    cmd_v.v_x = v_ref + pid_tran;
    cmd_v.v_y = 0.0f;
    cmd_v.w = w_ref + pid_rot;
		
	//회전이 직진에 묻힐 때 방지
	float abs_e = fabsf(error_rot);

	if (abs_e > 0.20f)
	cmd_v.v_x *= 0.3f;
	else if (abs_e > 0.10f)
	cmd_v.v_x *= 0.5f;
	else if (abs_e > 0.05f)
	cmd_v.v_x *= 0.8f;


    /* =====================================================
       5. IK : car velocity -> wheel velocity
       ===================================================== */

    wheel_velocity wheel_v = IK_car2wheel(robot, &cmd_v);


    /* =====================================================
       6. 상태 저장
       ===================================================== */

    prev_target_pose = target_pose;

    prev_error_tran = error_tran;
    prev_error_rot  = error_rot;


    return wheel_v;
}