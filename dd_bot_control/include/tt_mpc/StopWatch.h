#ifndef DD_BOT_STOPWATCH_H
#define DD_BOT_STOPWATCH_H

#include "ros/ros.h"

enum stopWatchState { RUNNING, STOPPED};

class StopWatch {

private:

	double m_elapsed_time;
	double m_abs_start_time;
	stopWatchState m_state;

public:

	StopWatch();

	void play();

	void pause();

	void reset();

	double getElapsedTime();


};


#endif