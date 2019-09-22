#ifndef DD_MPC_CONTROLLER_H
#define DD_MPC_CONTROLLER_H

#include "ros/ros.h"

#include <Eigen/Dense>
#include <cmath>

#include "std_msgs/Int16.h"
#include "std_msgs/Float32.h"
#include "std_msgs/Float64MultiArray.h"
#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Quaternion.h"
#include "dd_bot/DDBotState.h"
#include "dd_bot/CFTOCInput.h"
#include "dd_bot/CFTOCOutput.h"
#include "dd_bot/DDWheelVelCmd.h"

#include "StopWatch.h"
#include "MpcParameterStore.h"



enum TRAJ_STATE {STOP, START};

dd_bot::CFTOCInput cftoc_input_msg;
dd_bot::DDBotState curr_des_traj;
dd_bot::DDWheelVelCmd lr_wheel_vel_cmd;
TRAJ_STATE traj_state = STOP;

StopWatch sw {};


bool firstStart = true;

double t;

bool received_cftoc_output;

void initialize_parameters(MpcParameterStore &mpc_param);
float getYawFromQuat(const geometry_msgs::Quaternion &q);;
void update_ref_traj(double curr_time, MpcParameterStore &mpc_param);
void compute_wheel_velocity_cmd(MpcParameterStore &mpc_param);
#endif