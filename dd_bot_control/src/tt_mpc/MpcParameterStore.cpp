#include "MpcParameterStore.h"

bool MpcParameterStore::m_has_been_initialized = false;
int MpcParameterStore::m_N = 0.0;
float MpcParameterStore::m_time_step = 0.0;
float MpcParameterStore::m_LC_a1 = 0.0;
float MpcParameterStore::m_LC_a2 = 0.0;
float MpcParameterStore::m_LC_w1 = 0.0;
float MpcParameterStore::m_LC_w2 = 0.0;
float MpcParameterStore::m_LC_del = 0.0;
float MpcParameterStore::m_wheel_dist = 0.0;
float MpcParameterStore::m_wheel_rad = 0.0;

int MpcParameterStore::initialize_parameters(ros::NodeHandle &nh) {

	nh.getParam("/dd_mpc/horizon_length", m_N);
	nh.getParam("/dd_mpc/time_step", m_time_step);
	nh.getParam("/LC_traj/a1", m_LC_a1);
	nh.getParam("/LC_traj/a2", m_LC_a2);
	nh.getParam("/LC_traj/w1", m_LC_w1);
	nh.getParam("/LC_traj/w2", m_LC_w2);
	nh.getParam("/LC_traj/del", m_LC_del);
	nh.getParam("/dd_bot_geometry/wheel_dist", m_wheel_dist);
	nh.getParam("/dd_bot_geometry/wheel_radius", m_wheel_rad);

}

MpcParameterStore::MpcParameterStore() {}

MpcParameterStore::MpcParameterStore(ros::NodeHandle &nh) {

	if (!m_has_been_initialized) {
		initialize_parameters(nh);
		m_has_been_initialized = true;
	}

}

int MpcParameterStore::getHorizonLength() {
	return m_N;
}

float MpcParameterStore::getTimeStep() {
	return m_time_step;
}

float MpcParameterStore::getLC_a1() {
	return m_LC_a1;
}

float MpcParameterStore::getLC_a2() {
	return m_LC_a2;
}

float MpcParameterStore::getLC_w1() {
	return m_LC_w1;
}

float MpcParameterStore::getLC_w2() {
	return m_LC_w2;
}

float MpcParameterStore::getLC_del() {
	return m_LC_del;
}

float MpcParameterStore::getWheelDist() {
	return m_wheel_dist;
}

float MpcParameterStore::getWheelRad() {
	return m_wheel_rad;
}