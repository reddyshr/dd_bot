
#include "StopWatch.h"

StopWatch::StopWatch() : m_elapsed_time {0.0}, m_abs_start_time {0.0}, m_state {STOPPED} {}

void StopWatch::play() {
	if (m_state == STOPPED) {
		m_abs_start_time = ros::Time::now().toSec();
		m_state = RUNNING;
	}
}

void StopWatch::pause() {
	if (m_state == RUNNING) {
		m_elapsed_time = m_elapsed_time + (ros::Time::now().toSec() - m_abs_start_time);
		m_state = STOPPED;
	}
}

void StopWatch::reset() {
	m_elapsed_time = 0.0;
	m_state = STOPPED;
}

double StopWatch::getElapsedTime() {
	if (m_state == RUNNING) {
		m_elapsed_time = m_elapsed_time + (ros::Time::now().toSec() - m_abs_start_time);
		m_abs_start_time = ros::Time::now().toSec();
	}

	return m_elapsed_time;

}