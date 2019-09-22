

#include "dd_mpc_controller.h"

float v_cmd = 0.0;
float w_cmd = 0.0;

float getYawFromQuat(const geometry_msgs::Quaternion &q) {

	double siny_cosp = +2.0 * (q.w * q.z + q.x * q.y);
	double cosy_cosp = +1.0 - 2.0 * (q.y * q.y + q.z * q.z);
	return (float) std::atan2(siny_cosp, cosy_cosp);

}

void odomCallback(const nav_msgs::Odometry &msg) {

	cftoc_input_msg.curr_state.x = msg.pose.pose.position.x;
	cftoc_input_msg.curr_state.y = msg.pose.pose.position.y;
	cftoc_input_msg.curr_state.theta = getYawFromQuat(msg.pose.pose.orientation);

}

void compute_wheel_velocity_cmd(MpcParameterStore &mpc_param) {

	float wheel_dist = mpc_param.getWheelDist();
	float wheel_rad = mpc_param.getWheelRad();

	lr_wheel_vel_cmd.left_wheel_vel_cmd = (v_cmd - (0.5 * w_cmd * wheel_dist)) * (1.0 / wheel_rad);

	lr_wheel_vel_cmd.right_wheel_vel_cmd = (v_cmd + (0.5 * w_cmd * wheel_dist)) * (1.0 / wheel_rad);


}

void cftocOutputCallback(const dd_bot::CFTOCOutput &msg) {

	v_cmd = msg.v_out;
	w_cmd = msg.w_out;

	received_cftoc_output = true;

}

void trajStateCallback(const std_msgs::Int16 &msg) {

	if (msg.data == 0) {
		traj_state = STOP;
		sw.pause();
	} else if (msg.data == 1) {
		traj_state = START;
		sw.play();
	}

}


void update_ref_traj(double curr_time, MpcParameterStore &mpc_param) {

	float curr_x_ref;
	float curr_y_ref;
	float curr_x_ref_dot;
	float curr_y_ref_dot;
	float curr_x_ref_ddot;
	float curr_y_ref_ddot;

	int N = mpc_param.getHorizonLength();
	float a1 = mpc_param.getLC_a1();
	float a2 = mpc_param.getLC_a2();
	float w1 = mpc_param.getLC_w1();
	float w2 = mpc_param.getLC_w2();
	float del = mpc_param.getLC_del();
	float time_step = mpc_param.getTimeStep();

	cftoc_input_msg.curr_time = curr_time;

	for (int i = 0; i < N; i++) {

		curr_x_ref = a1 * std::sin(w1 * curr_time + del);
		curr_y_ref = a2 * std::sin(w2 * curr_time);
		curr_x_ref_dot = a1 * w1 * std::cos(w1 * curr_time + del);
		curr_y_ref_dot = a2 * w2 * std::cos(w2 * curr_time);
		curr_x_ref_ddot = -a1 * std::pow(w1, 2) * std::sin(w1 * curr_time + del);
		curr_y_ref_ddot = -a2 * std::pow(w2, 2) * std::sin(w2 * curr_time);


		/*curr_x_ref = 0.2*curr_time;
		curr_y_ref = 0;
		curr_x_ref_dot = 0.2;
		curr_y_ref_dot = 0;
		curr_x_ref_ddot = 0;
		curr_y_ref_ddot = 0;*/

		cftoc_input_msg.ref_traj[i].x = curr_x_ref;
		cftoc_input_msg.ref_traj[i].y = curr_y_ref;
		cftoc_input_msg.ref_traj[i].theta = std::atan2(curr_y_ref_dot, curr_x_ref_dot);
		cftoc_input_msg.ref_traj[i].v = std::sqrt(std::pow(curr_x_ref_dot, 2) + std::pow(curr_y_ref_dot, 2));

		if (curr_x_ref_dot == 0 && curr_y_ref_dot == 0) {
			cftoc_input_msg.ref_traj[i].w = 0.0;
		} else {
			cftoc_input_msg.ref_traj[i].w = (curr_x_ref_dot * curr_y_ref_ddot - curr_y_ref_dot * curr_x_ref_ddot) / (std::pow(curr_x_ref_dot, 2) + std::pow(curr_y_ref_dot, 2));
		}

		curr_time = curr_time + time_step;

	}


	return;

}

void initialize_parameters(MpcParameterStore &mpc_param) {

	int N = mpc_param.getHorizonLength();
	cftoc_input_msg.ref_traj.resize(N);

}

int main(int argc, char** argv) {


	ros::init(argc, argv, "dd_mpc_controller");

	ros::NodeHandle n;

	ros::Subscriber odom_sub = n.subscribe("ground_truth/state", 5, odomCallback);
	ros::Subscriber vel_cmd_sub = n.subscribe("dd_bot/cftoc_output", 5, cftocOutputCallback);
	ros::Subscriber traj_state_sub = n.subscribe("dd_bot/traj_state", 5, trajStateCallback);

	ros::Publisher cftoc_input_pub = n.advertise<dd_bot::CFTOCInput> ("/dd_bot/cftoc_input", 5);
	ros::Publisher wheel_vel_cmd_pub = n.advertise<dd_bot::DDWheelVelCmd> ("/dd_bot/twd_velocity/cmd", 5);
	ros::Publisher des_traj_pub = n.advertise<dd_bot::DDBotState> ("/dd_bot/des_traj", 5);

	MpcParameterStore mpc_param {n};

	initialize_parameters(mpc_param);
	ros::Rate loop_rate(30);

	lr_wheel_vel_cmd.left_wheel_vel_cmd = 0.0;
	lr_wheel_vel_cmd.right_wheel_vel_cmd = 0.0;

	while (ros::ok()) {

		/*while (!received_cftoc_output) {
			ros::spinOnce();
		}*/
		ros::spinOnce();
		compute_wheel_velocity_cmd(mpc_param);
		wheel_vel_cmd_pub.publish(lr_wheel_vel_cmd);
		t = sw.getElapsedTime();
		//printf("time: %f\n", t);
		if (t != 0) {
			update_ref_traj(t, mpc_param);

		}
		curr_des_traj = cftoc_input_msg.ref_traj[0];
		cftoc_input_pub.publish(cftoc_input_msg);
		des_traj_pub.publish(curr_des_traj);

		received_cftoc_output = false;

		loop_rate.sleep();

	}
}