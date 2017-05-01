/*
	EGP Graphics Framework
	(c) 2017 Dan Buckstein
	High-precision timer for Windows & Mac by Dan Buckstein
*/

#ifndef __EGPFW_UTILS_TIMER_H
#define __EGPFW_UTILS_TIMER_H


#ifdef __cplusplus
extern "C"
{
#endif	// __cplusplus


	// timer structure
	struct egpTimer
	{
		// started flag
		int started;

		// tick count
		unsigned int ticks;

		// duration of the current tick as float or double
		float dt;
		double ddt;

		// accumulation of time counted
		double totalTime;

		// frame rate and inverse frame rate
		union {
			double tps[1], spt[1];
			double ticksPerSecond, secondsPerTick;
		};
		
		// internal time values
		union {
			struct {
				long q[2];
				long long qs;
			};
			long long qu;
		} tf[1], t0[1], t1[1];
	};

#ifndef __cplusplus
	typedef struct egpTimer	egpTimer;
#endif	// !__cplusplus


	// start timer: 
	// begin counting time for the specified timer
	// 'timer' param cannot be null
	void egpTimerStart(egpTimer *timer);

	// stop timer: 
	// stop counting time for the specified timer
	// 'timer' param cannot be null
	void egpTimerStop(egpTimer *timer);

	// reset timer: 
	// stop timer and reset its values (except frame rate)
	// 'timer' param cannot be null
	void egpTimerReset(egpTimer *timer);

	// set timer: 
	// begin counting time
	// 'timer' param cannot be null
	// 'ticksPerSecond' param must be >= 0
	// if ticksPerSecond (frame rate) is 0, then the 
	//	timer will run in "continuous mode" (update 
	//	will always return true)
	void egpTimerSet(egpTimer *timer, double ticksPerSecond);

	// update timer: 
	// check if it is time to tick the timer
	// returns 1 if tick occurred, 0 if not
	// 'timer' param cannot be null
	int egpTimerUpdate(egpTimer *timer);


#ifdef __cplusplus
}
#endif	// __cplusplus


#endif	// __EGPFW_UTILS_TIMER_H