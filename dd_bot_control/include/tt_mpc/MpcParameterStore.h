#ifndef DD_MPC_PARAMETER_STORE_H
#define DD_MPC_PARAMETER_STORE_H

#include "ros/ros.h"


class MpcParameterStore {

private:

	static bool m_has_been_initialized;

	static int m_N;
	static float m_time_step;

	//Parameters for Lissajous Curve Trajectory
	static float m_LC_a1;
	static float m_LC_a2;
	static float m_LC_w1;
	static float m_LC_w2;
	static float m_LC_del;

	//Parameters for DD_Bot Geomerty

	static float m_wheel_dist;
	static float m_wheel_rad;

	int initialize_parameters(ros::NodeHandle &nh);

public:

	MpcParameterStore();

	MpcParameterStore(ros::NodeHandle &nh);

	int getHorizonLength();

	float getTimeStep();

	float getLC_a1();

	float getLC_a2();

	float getLC_w1();

	float getLC_w2();

	float getLC_del();

	float getWheelDist();

	float getWheelRad();

};


#endif
