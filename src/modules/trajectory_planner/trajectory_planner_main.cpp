/****************************************************************************
 *
 *   Copyright (c) 2013-2015 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file trajectory_planner_main.cpp
 * Hippocampus trajectory_planner
 *
 * based on mc_att_control_main.cpp from
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * adjusted by
 * @author Nils Rottmann    <Nils.Rottmann@tuhh.de>
 *
 * This app creates a trajectory and publishes it to the setpoint topics
 */


#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>
// uORB topics
#include <uORB/uORB.h>
#include <uORB/topics/trajectory_setpoint.h>
#include <uORB/topics/parameter_update.h>
// system libraries
#include <systemlib/param/param.h>
#include <systemlib/err.h>
#include <systemlib/perf_counter.h>
#include <systemlib/systemlib.h>
#include <systemlib/circuit_breaker.h>
// internal libraries
#include <lib/mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <lib/tailsitter_recovery/tailsitter_recovery.h>


// Hippocampus trajectory_planner
extern "C" __EXPORT int trajectory_planner_main(int argc, char *argv[]);

// the class from which an instance will be initiated by starting this application
class HippocampusTrajectoryPlanner
{
public:
	// Constructor
	HippocampusTrajectoryPlanner();

    // Destructor, also kills the main task
	~HippocampusTrajectoryPlanner();

    // Start the multicopter attitude control task, @return OK on success.
	int	start();

private:

	bool	_task_should_exit;		// if true, task_main() should exit
	int		_control_task;			// task handle

    // topic subscription
    int		        _params_sub;			// parameter updates subscription

    // topic publications
    orb_advert_t    _v_traj_sp_pub;

    // topic structures, in this structures the data of the topics are stored
	struct trajectory_setpoint_s	        _v_traj_sp;			// vehicle attitude setpoint

    // performance counters
	perf_counter_t	_loop_perf;
	perf_counter_t	_controller_latency_perf;

	// time counter
	float t_ges;

	struct {
	    param_t Dummy;
	}		_params_handles;		// handles for to find parameters

	struct {
        float Dummy;                  // parameters, they will be filled when the function parameters_update is called
	}		_params;

    // path controller.
	void		plan_trajectory(float dt);

	// Update our local parameter cache.
	int			parameters_update();                // checks if parameters have changed and updates them

	// Check for parameter update and handle it.
	void		parameter_update_poll();            // receives parameters

	// Shim for calling task_main from task_create.
	static void	task_main_trampoline(int argc, char *argv[]);

	// Main attitude control task.
	void		task_main();
};


namespace trajectory_planner
{
HippocampusTrajectoryPlanner	*g_control;
}

// constructor of class HippocampusPathControl
HippocampusTrajectoryPlanner::HippocampusTrajectoryPlanner() :

    // First part is about function which are called with the constructor
	_task_should_exit(false),
	_control_task(-1),

    // subscriptions
    _params_sub(-1),

	// publications
    _v_traj_sp_pub(nullptr),

	// performance counters
	_loop_perf(perf_alloc(PC_ELAPSED, "path_controller")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))

// here starts the allocation of values to the variables
{
    // define publication settings
	memset(&_v_traj_sp, 0, sizeof(_v_traj_sp));

    // set parameters to zero
    _params.Dummy = 0.0f;

    // set time counter to zero
    t_ges = 0.0f;

    // allocate parameter handles
    _params_handles.Dummy	            = 	param_find("PC_Dummy");

	// fetch initial parameter values
	parameters_update();
}

// destructor of class HippocampusPathControl
HippocampusTrajectoryPlanner::~HippocampusTrajectoryPlanner()
{
	if (_control_task != -1) {
		/* task wakes up every 100ms or so at the longest */
		_task_should_exit = true;

		/* wait for a second for the task to quit at our request */
		unsigned i = 0;

		do {
			/* wait 20ms */
			usleep(20000);

			/* if we have given up, kill it */
			if (++i > 50) {
				px4_task_delete(_control_task);
				break;
			}
		} while (_control_task != -1);
	}

	trajectory_planner::g_control = nullptr;
}

// updates parameters
int HippocampusTrajectoryPlanner::parameters_update()
{
    param_get(_params_handles.Dummy, &_params.Dummy);

	return OK;
}

// check for parameter updates
void HippocampusTrajectoryPlanner::parameter_update_poll()
{
	bool updated;

	/* Check if parameters have changed */
	orb_check(_params_sub, &updated);

	if (updated) {
		struct parameter_update_s param_update;
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_update);
		parameters_update();
	}
}


// This function calculates the actual position and velocity for the given trajectory, here a circle is used as example
// function works correctly
void HippocampusTrajectoryPlanner::plan_trajectory(float dt) {

    t_ges = t_ges + dt;                     // add time passed

    // trajectory (e.g. circle)
    float T_round = 120;                   // time required for one round of the circle
    float ratio = (2*M_PI/(double)T_round);
    float r = 2.0;                         // radius

    // calculate sinus and cosinus one time
    float sinus = sin(ratio*t_ges);
    float cosinus = cos(ratio*t_ges);

    // calculate position and their derivations
    _v_traj_sp.x = r*cosinus - 2.0f;           // x
    _v_traj_sp.y = r*sinus;                   // y
    _v_traj_sp.z = 0.0f;                     // z

    _v_traj_sp.dx = -ratio*sinus*r;           // dx/dt
    _v_traj_sp.dy = ratio*cosinus*r;          // dy/dt
    _v_traj_sp.dz = 0.0f;                     // dz/dt

    _v_traj_sp.ddx = -ratio*ratio*cosinus*r;
    _v_traj_sp.ddy = -ratio*ratio*sinus*r;
    _v_traj_sp.ddz = 0.0f;

    _v_traj_sp.dddx = ratio*ratio*ratio*sinus*r;
    _v_traj_sp.dddy = -ratio*ratio*ratio*cosinus*r;
    _v_traj_sp.dddz = 0.0f;

    _v_traj_sp.roll = 0.0f;                  // no roll angle
    _v_traj_sp.droll = 0.0f;

    // publish setpoint data
    orb_publish(ORB_ID(trajectory_setpoint), _v_traj_sp_pub, &_v_traj_sp);


}

// Just starts the task_main function
void HippocampusTrajectoryPlanner::task_main_trampoline(int argc, char *argv[])
{
	trajectory_planner::g_control->task_main();
}

// this is the main_task function which does the control task
void HippocampusTrajectoryPlanner::task_main()
{

    PX4_INFO("Trajectory Planner has been started!");

    // subscriber
    _params_sub = orb_subscribe(ORB_ID(parameter_update));              // parameter update

    // publisher
    _v_traj_sp_pub = orb_advertise(ORB_ID(trajectory_setpoint), &_v_traj_sp);

	// initialize parameters cache
    parameters_update();

	while (!_task_should_exit) {

		perf_begin(_loop_perf);


		static uint64_t last_run = 0;
		float dt = (hrt_absolute_time() - last_run) / 1000000.0f;   // calculate the time delta_t between two runs
		last_run = hrt_absolute_time();

		// do trajectory_planning
		plan_trajectory(dt);

        // check for parameter updates
        parameter_update_poll();

        // wait for a 10ms
        usleep(10000);

		perf_end(_loop_perf);
	}

	_control_task = -1;
	return;
}

// start function
int HippocampusTrajectoryPlanner::start()
{
	ASSERT(_control_task == -1);        // give a value -1

	// start the control task, performs any specific accounting, scheduler setup, etc.
	_control_task = px4_task_spawn_cmd("trajectory_planner",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&HippocampusTrajectoryPlanner::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

 // main function
 int trajectory_planner_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: trajectory_planner {start|stop|status}");
		return 1;
	}

    // if command is start, then first control if class exists already, if not, allocate new one
	if (!strcmp(argv[1], "start")) {

		if (trajectory_planner::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

        // allocate new class HippocampusTrajectoryPlanner
		trajectory_planner::g_control = new HippocampusTrajectoryPlanner;

        // check if class has been allocated, if not, give back failure
		if (trajectory_planner::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

        // if function start() can not be called, delete instance of HippocampusTrajectoryPlanner and allocate null pointer
		if (OK != trajectory_planner::g_control->start()) {
			delete trajectory_planner::g_control;
			trajectory_planner::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

    // if command is start, check if class exists, if not can not stop anything
	if (!strcmp(argv[1], "stop")) {
		if (trajectory_planner::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

        // if class exists, delete it and allocate null pointer
		delete trajectory_planner::g_control;
		trajectory_planner::g_control = nullptr;
		return 0;
	}

    // if command is status and class exists, give back running, else give back not running
	if (!strcmp(argv[1], "status")) {
		if (trajectory_planner::g_control) {
			warnx("running");
			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}