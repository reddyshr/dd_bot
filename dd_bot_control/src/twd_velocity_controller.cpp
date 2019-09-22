/*

This is a simple, pd velocity controller to control the velocity of each of the two wheels. It outputs the desired torque for each of the wheel, which is passed on to the lower level JointEffortController for each of the wheel hinge joints.

-reddyshr

*/

#include <ros/ros.h>
#include <std_msgs/Float64.h>
#include <sensor_msgs/JointState.h>
#include "dd_bot/DDWheelVelCmd.h"

float curr_left_wheel_vel;
float curr_right_wheel_vel;

float des_left_wheel_vel;
float des_right_wheel_vel;

void jointStateCallback(const sensor_msgs::JointState &msg) {

	curr_left_wheel_vel = msg.velocity[0];
	curr_right_wheel_vel = msg.velocity[1];

}

void twdVelocityCallback(const dd_bot::DDWheelVelCmd &msg) {

	des_left_wheel_vel = msg.left_wheel_vel_cmd;
	des_right_wheel_vel = msg.right_wheel_vel_cmd;

}

int main(int argc, char** argv) {

	ros::init(argc, argv, "twd_velocity_controller");
	ros::NodeHandle n;

	ros::Subscriber joint_state_sub = n.subscribe("dd_bot/joint_states", 5, jointStateCallback);
	ros::Subscriber cmd_vel_sub = n.subscribe("dd_bot/twd_velocity/cmd", 5, twdVelocityCallback);

	ros::Publisher left_wheel_torque_pub = n.advertise<std_msgs::Float64>("/dd_bot/left_wheel_hinge_effort_controller/command", 5);
	ros::Publisher right_wheel_torque_pub = n.advertise<std_msgs::Float64>("/dd_bot/right_wheel_hinge_effort_controller/command", 5);


	std_msgs::Float64 left_wheel_torque_cmd;
	std_msgs::Float64 right_wheel_torque_cmd;

	float control_rate_hz;
	float kp;
	float ki;
	float kd;

	float left_vel_err = 0.0;
	float old_left_vel_err = 0.0;
	float left_vel_err_dot = 0.0;

	float right_vel_err = 0.0;
	float old_right_vel_err = 0.0;
	float right_vel_err_dot = 0.0;

	des_left_wheel_vel = 0.0;
	des_right_wheel_vel = 0.0;

	n.getParam("/wheel_velocity_controller/control_rate_hz", control_rate_hz);
	n.getParam("/wheel_velocity_controller/kp", kp);
	n.getParam("/wheel_velocity_controller/ki", ki);
	n.getParam("/wheel_velocity_controller/kd", kd);

	ros::Rate loop_rate(control_rate_hz);

	while (ros::ok()) {

		ros::spinOnce();

		old_left_vel_err = left_vel_err;
		left_vel_err = des_left_wheel_vel - curr_left_wheel_vel;
		left_vel_err_dot = (left_vel_err - old_left_vel_err) * (1 / control_rate_hz);

		old_right_vel_err = right_vel_err;
		right_vel_err = des_right_wheel_vel - curr_right_wheel_vel;
		right_vel_err_dot = (right_vel_err - old_right_vel_err) * (1 / control_rate_hz);

		left_wheel_torque_cmd.data = kp * left_vel_err + kd * left_vel_err_dot;
		right_wheel_torque_cmd.data = kp * right_vel_err + kd * right_vel_err_dot;

		left_wheel_torque_pub.publish(left_wheel_torque_cmd);
		right_wheel_torque_pub.publish(right_wheel_torque_cmd);

		loop_rate.sleep();

	}


}