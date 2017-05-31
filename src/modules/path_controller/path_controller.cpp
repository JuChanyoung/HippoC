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
 * @file path_controller.cpp
 * Hippocampus path controller.
 *
 * Publication for the desired attitude tracking:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011.
 *
 * based on mc_att_control_main.cpp from
 * @author Lorenz Meier		<lorenz@px4.io>
 * @author Anton Babushkin	<anton.babushkin@me.com>
 * @author Sander Smeets	<sander@droneslab.com>
 *
 * adjusted by
 * @author Nils Rottmann    <Nils.Rottmann@tuhh.de>
 *
 * The controller has two loops: P loop for angular error and PD loop for angular rate error.
 * Desired rotation calculated keeping in mind that yaw response is normally slower than roll/pitch.
 * For small deviations controller rotates copter to have shortest path of thrust vector and independently rotates around yaw,
 * so actual rotation axis is not constant. For large deviations controller rotates copter around fixed axis.
 * These two approaches fused seamlessly with weight depending on angular error.
 * When thrust vector directed near-horizontally (e.g. roll ~= PI/2) yaw setpoint ignored because of singularity.
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 * If rotation matrix setpoint is invalid it will be generated from Euler angles for compatibility with old position controllers.
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
#include <uORB/topics/att_pos_mocap.h>                  // this topic holds the position from gazebo
#include <uORB/topics/actuator_controls.h>              // this topic gives the actuators control input
#include <uORB/topics/control_state.h>                  // this topic holds the orientation calculated using sensor data
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/fw_virtual_rates_setpoint.h>
#include <uORB/topics/mc_virtual_rates_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/multirotor_motor_limits.h>
#include <uORB/topics/mc_att_ctrl_status.h>
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

/**
 * Hippocampus path controller
 *
 * @ingroup apps
 */
extern "C" __EXPORT int path_controller_main(int argc, char *argv[]);

/*#define YAW_DEADZONE	0.05f
#define MIN_TAKEOFF_THRUST    0.2f
#define RATES_I_LIMIT	0.3f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ATTITUDE_TC_DEFAULT 0.2f

#define AXIS_INDEX_ROLL 0
#define AXIS_INDEX_PITCH 1
#define AXIS_INDEX_YAW 2
#define AXIS_COUNT 3 */



// the class from which an instance will be initiated by starting this application
class HippocampusPathControl
{
public:
	// Constructor
	HippocampusPathControl();

    // Destructor, also kills the main task
	~HippocampusPathControl();

    // Start the multicopter attitude control task, @return OK on success.
	int	start();

private:

	bool	_task_should_exit;		// if true, task_main() should exit
	int		_control_task;			// task handle

    int     _att_pos_sub;           // att_pos_mocap subscription (position data)
	int		_ctrl_state_sub;		// control state subscription (orientation data)

	/*int		_v_att_sp_sub;			    // vehicle attitude setpoint subscription
	int		_v_rates_sp_sub;		    // vehicle rates setpoint subscription
	int		_v_control_mode_sub;	    // vehicle control mode subscription
	int		_params_sub;			    // parameter updates subscription
	int		_manual_control_sp_sub;	    // manual control setpoint subscription
	int		_armed_sub;				    // arming status subscription
	int		_vehicle_status_sub;	    // vehicle status subscription
	int 	_motor_limits_sub;		    // < motor limits subscription */

    orb_advert_t	_actuators_0_pub;		// attitude actuator controls publication

    orb_id_t        _actuators_id;	        // pointer to correct actuator controls0 uORB metadata structure

	/*orb_advert_t	_v_rates_sp_pub;		// rate setpoint publication
	orb_advert_t	_controller_status_pub;	// controller status publication*/

	/*orb_id_t _rates_sp_id;	    // pointer to correct rates setpoint uORB metadata structure  */

	//bool		_actuators_0_circuit_breaker_enabled;	/**< circuit breaker to suppress output */

	struct actuator_controls_s			_actuators;			// actuator controls
	struct att_pos_mocap_s              _att_pos;           // attitude (orientation)
    struct control_state_s				_ctrl_state;		// control state

	/*struct vehicle_attitude_setpoint_s	_v_att_sp;			// vehicle attitude setpoint
	struct vehicle_rates_setpoint_s		_v_rates_sp;		// vehicle rates setpoint
	struct manual_control_setpoint_s	_manual_control_sp;	// manual control setpoint
	struct vehicle_control_mode_s		_v_control_mode;	// vehicle control mode
	struct actuator_armed_s				_armed;				// actuator arming status
	struct vehicle_status_s				_vehicle_status;	// vehicle status
	struct multirotor_motor_limits_s	_motor_limits;		// motor limits
	struct mc_att_ctrl_status_s 		_controller_status; // controller status */

	perf_counter_t	_loop_perf;			    // loop performance counter
	perf_counter_t	_controller_latency_perf;

    // storage vectors for positions and roll angle
    math::Vector<3>     _position_0;        // actual position
    math::Vector<3>     _position_1;        // previous position
    math::Vector<3>     _position_2;
    math::Vector<3>     _position_3;

    // time
    double              t_ges;
    double              counter;

    math::Matrix<3, 3>  _I;				// identity matrix

	/*math::Vector<3>		_rates_prev;	// angular rates on previous step
	math::Vector<3>		_rates_sp_prev; // previous rates setpoint
	math::Vector<3>		_rates_sp;		// angular rates setpoint
	math::Vector<3>		_rates_int;		// angular rates integral error
	float				_thrust_sp;		// thrust setpoint
	math::Vector<3>		_att_control;	// attitude control vector */

	/*

	struct {
		param_t roll_p;
		param_t roll_rate_p;
		param_t roll_rate_i;
		param_t roll_rate_d;
		param_t roll_rate_ff;
		param_t pitch_p;
		param_t pitch_rate_p;
		param_t pitch_rate_i;
		param_t pitch_rate_d;
		param_t pitch_rate_ff;
		param_t tpa_breakpoint;
		param_t tpa_slope;
		param_t yaw_p;
		param_t yaw_rate_p;
		param_t yaw_rate_i;
		param_t yaw_rate_d;
		param_t yaw_rate_ff;
		param_t yaw_ff;
		param_t roll_rate_max;
		param_t pitch_rate_max;
		param_t yaw_rate_max;
		param_t yaw_auto_max;

		param_t acro_roll_max;
		param_t acro_pitch_max;
		param_t acro_yaw_max;
		param_t rattitude_thres;

		param_t vtol_type;
		param_t roll_tc;
		param_t pitch_tc;
		param_t vtol_opt_recovery_enabled;
		param_t vtol_wv_yaw_rate_scale;

	}		_params_handles;		// handles for interesting parameters */

	/*struct {
		math::Vector<3> att_p;					// P gain for angular error
		math::Vector<3> rate_p;				// P gain for angular rate error
		math::Vector<3> rate_i;				// I gain for angular rate error
		math::Vector<3> rate_d;				// D gain for angular rate error
		math::Vector<3>	rate_ff;			// Feedforward gain for desired rates
		float yaw_ff;						// yaw control feed-forward

		float tpa_breakpoint;				// Throttle PID Attenuation breakpoint
		float tpa_slope;					// Throttle PID Attenuation slope

		float roll_rate_max;
		float pitch_rate_max;
		float yaw_rate_max;
		float yaw_auto_max;
		math::Vector<3> mc_rate_max;		// attitude rate limits in stabilized modes
		math::Vector<3> auto_rate_max;		// attitude rate limits in auto modes
		math::Vector<3> acro_rate_max;		// max attitude rates in acro mode
		float rattitude_thres;
		int vtol_type;						// 0 = Tailsitter, 1 = Tiltrotor, 2 = Standard airframe
		bool vtol_opt_recovery_enabled;
		float vtol_wv_yaw_rate_scale;			// Scale value [0, 1] for yaw rate setpoint
	}		_params;

	TailsitterRecovery *_ts_opt_recovery;	// Computes optimal rates for tailsitter recovery */


    // actualizes position data
	void        actualize_position();

	// writes position data into variables
	void        get_position(math::Vector<3>& x, math::Vector<3>& dx, math::Vector<3>& ddx, math::Vector<3>& dddx, double dt);

	// get desired position data
	void        get_position_des(math::Vector<3>& x, math::Vector<3>& dx, math::Vector<3>& ddx,
                                                    math::Vector<3>& dddx, double& PHI, double& dPHI, double dt);

	// cross product
	math::Vector<3> cross(math::Vector<3> a, math::Vector<3> b);

    // path controller.
	void		path_control(float dt);

	/*// Update our local parameter cache.
	int			parameters_update();


	// Check for parameter update and handle it.
	void		parameter_update_poll();


	// Check for changes in vehicle control mode.
	void		vehicle_control_mode_poll();


	// Check for changes in manual inputs.
	void		vehicle_manual_poll();

	// Check for attitude setpoint updates.
	//void		vehicle_attitude_setpoint_poll();


	// Check for rates setpoint updates.
	void		vehicle_rates_setpoint_poll();


	// Check for arming status updates.
	void		arming_status_poll();

	// Attitude rates controller.
	void		control_attitude_rates(float dt);

	// Check for vehicle status updates.
	void		vehicle_status_poll();

    // Check for vehicle motor limits status.
	void		vehicle_motor_limits_poll();*/

	// Shim for calling task_main from task_create.
	static void	task_main_trampoline(int argc, char *argv[]);

	// Main attitude control task.
	void		task_main();
};


namespace path_controller
{
HippocampusPathControl	*g_control;
}

// constructor of class HippocampusPathControl
HippocampusPathControl::HippocampusPathControl() :

    // First part is about function which are called with the constructor

	_task_should_exit(false),
	_control_task(-1),

	// subscriptions
	_att_pos_sub(-1),
    _ctrl_state_sub(-1),

	/*_v_att_sp_sub(-1),
	_v_control_mode_sub(-1),
	_params_sub(-1),
	_manual_control_sp_sub(-1),
	_armed_sub(-1),
	_vehicle_status_sub(-1),*/

	// publications
	_actuators_0_pub(nullptr),
    _actuators_id(0),

	/*_v_rates_sp_pub(nullptr),
	_controller_status_pub(nullptr),
	_rates_sp_id(0),

	_actuators_0_circuit_breaker_enabled(false),*/

	// performance counters
	_loop_perf(perf_alloc(PC_ELAPSED, "path_controller")),
	_controller_latency_perf(perf_alloc_once(PC_ELAPSED, "ctrl_latency"))
	// _ts_opt_recovery(nullptr)

// here starts the allocation of values to the variables
{
    memset(&_att_pos, 0, sizeof(_att_pos));
	memset(&_ctrl_state, 0, sizeof(_ctrl_state));
	memset(&_actuators, 0, sizeof(_actuators));

	/*memset(&_v_att_sp, 0, sizeof(_v_att_sp));
	memset(&_v_rates_sp, 0, sizeof(_v_rates_sp));
	memset(&_manual_control_sp, 0, sizeof(_manual_control_sp));
	memset(&_v_control_mode, 0, sizeof(_v_control_mode));
	memset(&_armed, 0, sizeof(_armed));
	memset(&_vehicle_status, 0, sizeof(_vehicle_status));
	memset(&_motor_limits, 0, sizeof(_motor_limits));
	memset(&_controller_status, 0, sizeof(_controller_status));
	_vehicle_status.is_rotary_wing = true; */

	/*_params.att_p.zero();
	_params.rate_p.zero();
	_params.rate_i.zero();
	_params.rate_d.zero();
	_params.rate_ff.zero();
	_params.yaw_ff = 0.0f;
	_params.roll_rate_max = 0.0f;
	_params.pitch_rate_max = 0.0f;
	_params.yaw_rate_max = 0.0f;
	_params.mc_rate_max.zero();
	_params.auto_rate_max.zero();
	_params.acro_rate_max.zero();
	_params.rattitude_thres = 1.0f;
	_params.vtol_opt_recovery_enabled = false;
	_params.vtol_wv_yaw_rate_scale = 1.0f;          */

    // set initial values of vectors to zero
    _position_0.zero();
    _position_1.zero();
    _position_2.zero();
    _position_3.zero();

    t_ges = 0.0;
    counter = 0.0;

	/*_rates_prev.zero();
	_rates_sp.zero();
	_rates_sp_prev.zero();
	_rates_int.zero();
	_thrust_sp = 0.0f;
	_att_control.zero();    */

	_I.identity();

	/*_params_handles.roll_p			= 	param_find("MC_ROLL_P");
	_params_handles.roll_rate_p		= 	param_find("MC_ROLLRATE_P");
	_params_handles.roll_rate_i		= 	param_find("MC_ROLLRATE_I");
	_params_handles.roll_rate_d		= 	param_find("MC_ROLLRATE_D");
	_params_handles.roll_rate_ff	= 	param_find("MC_ROLLRATE_FF");
	_params_handles.pitch_p			= 	param_find("MC_PITCH_P");
	_params_handles.pitch_rate_p	= 	param_find("MC_PITCHRATE_P");
	_params_handles.pitch_rate_i	= 	param_find("MC_PITCHRATE_I");
	_params_handles.pitch_rate_d	= 	param_find("MC_PITCHRATE_D");
	_params_handles.pitch_rate_ff 	= 	param_find("MC_PITCHRATE_FF");
	_params_handles.tpa_breakpoint 	= 	param_find("MC_TPA_BREAK");
	_params_handles.tpa_slope	 	= 	param_find("MC_TPA_SLOPE");
	_params_handles.yaw_p			=	param_find("MC_YAW_P");
	_params_handles.yaw_rate_p		= 	param_find("MC_YAWRATE_P");
	_params_handles.yaw_rate_i		= 	param_find("MC_YAWRATE_I");
	_params_handles.yaw_rate_d		= 	param_find("MC_YAWRATE_D");
	_params_handles.yaw_rate_ff	 	= 	param_find("MC_YAWRATE_FF");
	_params_handles.yaw_ff			= 	param_find("MC_YAW_FF");
	_params_handles.roll_rate_max	= 	param_find("MC_ROLLRATE_MAX");
	_params_handles.pitch_rate_max	= 	param_find("MC_PITCHRATE_MAX");
	_params_handles.yaw_rate_max	= 	param_find("MC_YAWRATE_MAX");
	_params_handles.yaw_auto_max	= 	param_find("MC_YAWRAUTO_MAX");
	_params_handles.acro_roll_max	= 	param_find("MC_ACRO_R_MAX");
	_params_handles.acro_pitch_max	= 	param_find("MC_ACRO_P_MAX");
	_params_handles.acro_yaw_max	= 	param_find("MC_ACRO_Y_MAX");
	_params_handles.rattitude_thres = 	param_find("MC_RATT_TH");
	_params_handles.vtol_type 		= 	param_find("VT_TYPE");
	_params_handles.roll_tc			= 	param_find("MC_ROLL_TC");
	_params_handles.pitch_tc		= 	param_find("MC_PITCH_TC");
	_params_handles.vtol_opt_recovery_enabled	= param_find("VT_OPT_RECOV_EN");
	_params_handles.vtol_wv_yaw_rate_scale		= param_find("VT_WV_YAWR_SCL");     */




	/* // fetch initial parameter values
	parameters_update();

	if (_params.vtol_type == 0 && _params.vtol_opt_recovery_enabled) {
		// the vehicle is a tailsitter, use optimal recovery control strategy
		_ts_opt_recovery = new TailsitterRecovery();
	}       */


}

// destructor of class HippocampusPathControl
HippocampusPathControl::~HippocampusPathControl()
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

	/*if (_ts_opt_recovery != nullptr) {
		delete _ts_opt_recovery;
	}*/

	path_controller::g_control = nullptr;
}


/**
* actualizes the position data if receiving new data
*/
void HippocampusPathControl::actualize_position() {

    /* check if there is a new setpoint */
	bool updated;
	orb_check(_att_pos_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(att_pos_mocap), _att_pos_sub, &_att_pos);
	}

	// write position data into vectors
	math::Vector<3> holder(_att_pos.x, _att_pos.y, _att_pos.z);
	// transform position in correct coordinate frame
	math::Matrix<3, 3> T_pos;
	T_pos.zero();
	T_pos(0, 1) = 1.0;
	T_pos(1, 0) = -1.0;
	T_pos(2, 2) = 1.0;
	_position_3 = _position_2;
    _position_2 = _position_1;
    _position_1 = _position_0;
    _position_0 = T_pos*holder;
}

/**
* Calculates the velocity and accelerations based on the position using differential quotient
*/
void HippocampusPathControl::get_position(math::Vector<3>& x, math::Vector<3>& dx, math::Vector<3>& ddx, math::Vector<3>& dddx, double dt) {
    x = _position_0;
    dx = (_position_0 - _position_1)/dt;
    ddx = (_position_0 - _position_1 * 2.0 + _position_2)/(dt*dt);
    dddx = (_position_0 - _position_1 * 2.0 + _position_2 * 2.0 - _position_3)/(2.0*dt*dt*dt);
}

// This function calculates the actual position and velocity for the given trajectory, here a circle is used as example
// function works correctly
void HippocampusPathControl::get_position_des(math::Vector<3>& x, math::Vector<3>& dx, math::Vector<3>& ddx,
                                                    math::Vector<3>& dddx, double& PHI, double& dPHI, double dt) {

    t_ges = t_ges + dt;

    /*double T_round = 120;           // time required for one round of the circle
    double ratio = (2*M_PI/T_round);

    // calculate sinus and cosinus one time
    double sinus = sin(ratio*t_ges);
    double cosinus = cos(ratio*t_ges);

    // calculate position and their derivations
    x(0) = cosinus - 1.0;           // x
    x(1) = sinus;                   // y
    x(2) = 0.0;                     // z

    dx(0) = -ratio*sinus;           // dx/dt
    dx(1) = ratio*cosinus;          // dy/dt
    dx(2) = 0.0;                    // dz/dt

    ddx(0) = -ratio*ratio*cosinus;
    ddx(1) = -ratio*ratio*sinus;
    ddx(2) = 0.0;

    dddx(0) = ratio*ratio*ratio*sinus;
    dddx(1) = -ratio*ratio*ratio*cosinus;
    dddx(2) = 0.0; */

    x(0) = 3.0;
    x(1) = 3.0;
    x(2) = 0.0;

    dx.zero();
    ddx.zero();
    dddx.zero();

    PHI = 0.0;                  // no roll angle
    dPHI = 0.0;

    /*PX4_INFO("Circle:\t%8.4f\t%8.4f\t%8.4f",
                     (double)x(0),
                     (double)x(1),
                     (double)t_ges); */


}

// calculates the cross product from vector a x b
math::Vector<3> HippocampusPathControl::cross(math::Vector<3> a, math::Vector<3> b) {
    math::Vector<3> c;
    c(0) = a(1)*b(2) - a(2)*b(1);
    c(1) = a(2)*b(0) - a(0)*b(2);
    c(2) = a(0)*b(1) - a(1)*b(0);
    return c;
}


/**
 * path controller.
 * Input: 'vehicle_attitude_setpoint' topics (depending on mode)
 * Output: '_rates_sp' vector, '_thrust_sp'
 */
void HippocampusPathControl::path_control(float dt)
{
	/* Define variables */

	// gain matrices and other parameter, later they should be saved in another file
	math::Matrix<3, 3> K_p = _I*1.0;        // position
	math::Matrix<3, 3> K_v = _I*1.0;        // velocity
	math::Matrix<3, 3> K_r = _I*10.0;        // orientation
	math::Matrix<3, 3> K_w = _I*1.0;        // angular velocity
	double m = 1.0;                       // mass
	double d = 1.0;                       // damping constant

	// error vectors
	math::Vector<3> e_p;            // position error
	math::Vector<3> e_v;            // velocity error
	math::Vector<3> e_r;            // orientation error
	math::Matrix<3, 3> e_r_matrix;
	math::Vector<3> e_w;            // angular velocity error

	// position vectors
	math::Vector<3> r;              // actual position
	math::Vector<3> r_T;            // desired position
	math::Vector<3> rd;             // actual velocity
	math::Vector<3> rd_T;           // desired velocity
	math::Vector<3> rdd;            // actual acceleration
	math::Vector<3> rdd_T;          // desired acceleration
    math::Vector<3> rddd;           // actual acceleration
	math::Vector<3> rddd_T;         // desired acceleration#

	// Roll angles and velocities
	double PHI_T = 0;
	double PHId_T = 0;

    // desired force and outputs
    math::Vector<3> F_des;
    double u_1;
    math::Vector<3> u_24;

    // rotation matrices and angular velocity vectors
    math::Vector<3> w_BW;               // actual angular velocity  (in world coordinates)
    math::Vector<3> w_BW_T;             // desired angular velocity (in world coordinates)
    math::Matrix<3, 3> R;                // actual rotation matrix
	math::Matrix<3, 3> R_des;           // desired rotation matrix

    /* Receive data and calculate necessary values for the control law */

    // get position data
    actualize_position();       // record new position data
    get_position(r, rd, rdd, rddd, dt);
    get_position_des(r_T, rd_T, rdd_T, rddd_T, PHI_T, PHId_T, dt);         // get desired data

	// get current rotation matrix from control state quaternions, the quaternions are generated by the
	// attitude_estimator_q application using the sensor data
	math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);     // control_state is frequently updated
	R = q_att.to_dcm();      //http://www.cs.ucr.edu/~vbz/resources/quatut.pdf

    // transpose orientation matrix to get p'=Rp instead of p'=pR
    R = R.transposed();

    // get actual angular velocity
	w_BW(0) = _ctrl_state.roll_rate;
	w_BW(1) = _ctrl_state.pitch_rate;
	w_BW(2) = _ctrl_state.yaw_rate;

	// transform angular velocity
	w_BW = R.transposed()*w_BW;

    // orientation vectors
	math::Vector<3> x_B(R(0, 0), R(1, 0), R(2, 0));         // orientation body x-axis (in world coordinates)
	math::Vector<3> x_B_des;                                // orientation body x-axis desired
	math::Vector<3> y_B_des;                                // orientation body y-axis desired
	math::Vector<3> z_B_des;                                // orientation body z-axis desired
	math::Vector<3> z_C_des(0, -sin(PHI_T), cos(PHI_T));    // orientation C-Coordinate system desired

    // projection on x_B
    math::Vector<3> h_w;
    math::Vector<3> h_w_des;

	/* Control Law, since all data are now available */

	// thrust input
	e_p = r - r_T;          // calculate position error
	e_v = rd - rd_T;        // calculate velocity error

	F_des = -K_p*e_p - K_v*e_v + rdd_T*m + rd_T*d;      // calculate desired force

	u_1 = F_des*x_B;                                   // calculate desired thrust

    // calculate orientation vectors
    x_B_des = F_des/F_des.length();
    y_B_des = cross(z_C_des,x_B_des);
    y_B_des = y_B_des/y_B_des.length();
    z_B_des = cross(x_B_des,y_B_des);

    // calculate desired rotation matrix
    R_des.set_col(0, x_B_des);
    R_des.set_col(1, y_B_des);
    R_des.set_col(2, z_B_des);

    // Calculate angular velocity
    h_w_des = (rddd_T-x_B_des*(x_B_des*rddd_T))*(m/u_1);

    double q_ang = -h_w_des*z_B_des;
    double r_ang = -h_w_des*y_B_des;
    double p_ang = PHId_T*(double)x_B_des(0);
    w_BW_T = x_B_des*p_ang + y_B_des*q_ang + z_B_des*r_ang;

	// roll, pitch and yaw input
	e_r_matrix = (R_des.transposed()*R - R.transposed()*R_des)*0.5;    // orientation error
	e_w = R.transposed()*w_BW - R.transposed()*w_BW_T;
	e_r(0) = e_r_matrix(2,1);                                   // vee map
	e_r(1) = e_r_matrix(0,2);
	e_r(2) = e_r_matrix(1,0);

	u_24 = -K_r*e_r - K_w*e_w;                         // calculate inputs for roll, pitch and yaw

    // give the inputs to the actuators, where roll and pitch have to be given in negative orientation
    _actuators.control[0] = u_24(0);           // roll
    _actuators.control[1] = u_24(1);           // pitch
    _actuators.control[2] = u_24(2);           // yaw
    _actuators.control[3] = u_1;               // thrust

    //_actuators.control[1] = 1.0;

    // Debugging
    if (counter < t_ges) {

        counter = counter + 5.0;

        PX4_INFO("x_B:\t\t%8.4f\t%8.4f\t%8.4f",
                     (double)R(0, 0),
                     (double)R(1, 0),
                     (double)R(2, 0));
        PX4_INFO("x_B_des:\t%8.4f\t%8.4f\t%8.4f",
                     (double)x_B_des(0),
                     (double)x_B_des(1),
                     (double)x_B_des(2));
        PX4_INFO("y_B:\t\t%8.4f\t%8.4f\t%8.4f",
                     (double)R(0, 1),
                     (double)R(1, 1),
                     (double)R(2, 1));
        PX4_INFO("y_B_des:\t%8.4f\t%8.4f\t%8.4f",
                     (double)y_B_des(0),
                     (double)y_B_des(1),
                     (double)y_B_des(2));
        PX4_INFO("z_B:\t\t%8.4f\t%8.4f\t%8.4f",
                     (double)R(0, 2),
                     (double)R(1, 2),
                     (double)R(2, 2));
        PX4_INFO("z_B_des:\t%8.4f\t%8.4f\t%8.4f",
                     (double)z_B_des(0),
                     (double)z_B_des(1),
                     (double)z_B_des(2));
        PX4_INFO("Pos:\t\t%8.4f\t%8.4f\t%8.4f",
                     (double)r(0),
                     (double)r(1),
                     (double)r(2));
        PX4_INFO("Pos_des:\t%8.4f\t%8.4f\t%8.4f",
                     (double)r_T(0),
                     (double)r_T(1),
                     (double)r_T(2));
        /*PX4_INFO("w_BW:\t\t%8.4f\t%8.4f\t%8.4f",
                     (double)w_BW(0),
                     (double)w_BW(1),
                     (double)w_BW(2));
        PX4_INFO("w_BW_des:\t%8.4f\t%8.4f\t%8.4f",
                     (double)w_BW_T(0),
                     (double)w_BW_T(1),
                     (double)w_BW_T(2)); */
        PX4_INFO("e_p:\t\t%8.4f\t%8.4f\t%8.4f",
                     (double)e_p(0),
                     (double)e_p(1),
                     (double)e_p(2));
        /*PX4_INFO("e_v:\t%8.4f\t%8.4f\t%8.4f",
                     (double)e_v(0),
                     (double)e_v(1),
                     (double)e_v(2));*/
        PX4_INFO("e_r:\t\t%8.4f\t%8.4f\t%8.4f",
                     (double)e_r(0),
                     (double)e_r(1),
                     (double)e_r(2));
        /*PX4_INFO("e_w:\t%8.4f\t%8.4f\t%8.4f",
                     (double)e_w(0),
                     (double)e_w(1),
                     (double)e_w(2));
        PX4_INFO("F_des:\t%8.4f\t%8.4f\t%8.4f",
                     (double)F_des(0),
                     (double)F_des(1),
                     (double)F_des(2));*/


    }

}

/*
 * Attitude rates controller.
 * Input: '_rates_sp' vector, '_thrust_sp'
 * Output: '_att_control' vector
 */
/*void MulticopterAttitudeControl::control_attitude_rates(float dt)
{
	// reset integral if disarmed
	if (!_armed.armed || !_vehicle_status.is_rotary_wing) {
		_rates_int.zero();
	}

	// current body angular rates
	math::Vector<3> rates;
	rates(0) = _ctrl_state.roll_rate;
	rates(1) = _ctrl_state.pitch_rate;
	rates(2) = _ctrl_state.yaw_rate;

	// throttle pid attenuation factor
	float tpa =  fmaxf(0.0f, fminf(1.0f, 1.0f - _params.tpa_slope * (fabsf(_v_rates_sp.thrust) - _params.tpa_breakpoint)));

	// angular rates error
	math::Vector<3> rates_err = _rates_sp - rates;

	_att_control = _params.rate_p.emult(rates_err * tpa) + _params.rate_d.emult(_rates_prev - rates) / dt + _rates_int +
		       _params.rate_ff.emult(_rates_sp);

	_rates_sp_prev = _rates_sp;
	_rates_prev = rates;

	// update integral only if not saturated on low limit and if motor commands are not saturated
	if (_thrust_sp > MIN_TAKEOFF_THRUST && !_motor_limits.lower_limit && !_motor_limits.upper_limit) {
		for (int i = AXIS_INDEX_ROLL; i < AXIS_COUNT; i++) {
			if (fabsf(_att_control(i)) < _thrust_sp) {
				float rate_i = _rates_int(i) + _params.rate_i(i) * rates_err(i) * dt;

				if (PX4_ISFINITE(rate_i) && rate_i > -RATES_I_LIMIT && rate_i < RATES_I_LIMIT &&
				    _att_control(i) > -RATES_I_LIMIT && _att_control(i) < RATES_I_LIMIT &&
				    // if the axis is the yaw axis, do not update the integral if the limit is hit
				    !((i == AXIS_INDEX_YAW) && _motor_limits.yaw)) {
					_rates_int(i) = rate_i;
				}
			}
		}
	}
} */

// Just starts the task_main function
void HippocampusPathControl::task_main_trampoline(int argc, char *argv[])
{
	path_controller::g_control->task_main();
}

// this is the main_task function which does the control task
void HippocampusPathControl::task_main()
{

    PX4_INFO("Path Controller has been started!");
	// subscribe to uORB topics
	_att_pos_sub = orb_subscribe(ORB_ID(att_pos_mocap));        // subscribes to the att_pos_mocap topic
    //orb_set_interval(_att_pos_sub, 10);                     // limit the update rate to 100 Hz
    _ctrl_state_sub = orb_subscribe(ORB_ID(control_state));     // subscribe zo control state topic to get orientation

	/*_v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
	_v_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_control_sp_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_armed_sub = orb_subscribe(ORB_ID(actuator_armed));
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_motor_limits_sub = orb_subscribe(ORB_ID(multirotor_motor_limits));*/

	/*// initialize parameters cache
	parameters_update(); */

	// wakeup source: vehicle pose
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _ctrl_state_sub;
	fds[0].events = POLLIN;

	while (!_task_should_exit) {

		// wait for up to 100ms for data, we try to poll the data, thus to receive some from the att_pos_mocap topic
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 100);

		// timed out - periodic check for _task_should_exit
		if (pret == 0) {
		    PX4_INFO("Got no data in 100ms!");
			continue;
		}

		// this is undesirable but not much we can do - might want to flag unhappy status
		if (pret < 0) {
			warn("path_controller: poll error %d, %d", pret, errno);
			// sleep a bit before next try
			usleep(100000);
			continue;
		}

		perf_begin(_loop_perf);

		// run controller on pose changes
		if (fds[0].revents & POLLIN) {
			static uint64_t last_run = 0;
			float dt = (hrt_absolute_time() - last_run) / 1000000.0f;   // calculate the time delta_t between two runs
			last_run = hrt_absolute_time();

			// guard against too small (< 2ms) and too large (> 20ms) dt's
			if (dt < 0.002f) {
				dt = 0.002f;

			} else if (dt > 0.02f) {
				dt = 0.02f;
			}

			// copy pose
			orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

			// do path control
			path_control(dt);

			// publish actuator controls
			_actuators.timestamp = hrt_absolute_time();
			_actuators.timestamp_sample = _ctrl_state.timestamp;

			if (_actuators_0_pub != nullptr) {
                orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
				perf_end(_controller_latency_perf);
			} else if (_actuators_id) {
				_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
			}


            _actuators_id = ORB_ID(actuator_controls_0);

			/*// check for updates in other topics
			parameter_update_poll();
			vehicle_control_mode_poll();
			arming_status_poll();
			vehicle_manual_poll();
			vehicle_status_poll();
			vehicle_motor_limits_poll(); */

            // No need to check this for the Hippocampus
			/* Check if we are in rattitude mode and the pilot is above the threshold on pitch
			 * or roll (yaw can rotate 360 in normal att control).  If both are true don't
			 * even bother running the attitude controllers */
			/*if (_v_control_mode.flag_control_rattitude_enabled) {
				if (fabsf(_manual_control_sp.y) > _params.rattitude_thres ||
				    fabsf(_manual_control_sp.x) > _params.rattitude_thres) {
					_v_control_mode.flag_control_attitude_enabled = false;
				}
			}*/

			/*if (_v_control_mode.flag_control_attitude_enabled) {

				if (_ts_opt_recovery == nullptr) {
					// the  tailsitter recovery instance has not been created, thus, the vehicle
					// is not a tailsitter, do normal attitude control
					path_control(dt);

				} else {
					vehicle_attitude_setpoint_poll();
					_thrust_sp = _v_att_sp.thrust;
					math::Quaternion q(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
					math::Quaternion q_sp(&_v_att_sp.q_d[0]);
					_ts_opt_recovery->setAttGains(_params.att_p, _params.yaw_ff);
					_ts_opt_recovery->calcOptimalRates(q, q_sp, _v_att_sp.yaw_sp_move_rate, _rates_sp);

					// limit rates
					for (int i = 0; i < 3; i++) {
						_rates_sp(i) = math::constrain(_rates_sp(i), -_params.mc_rate_max(i), _params.mc_rate_max(i));
					}
				}       */

				/*// publish attitude rates setpoint
				_v_rates_sp.roll = _rates_sp(0);
				_v_rates_sp.pitch = _rates_sp(1);
				_v_rates_sp.yaw = _rates_sp(2);
				_v_rates_sp.thrust = _thrust_sp;
				_v_rates_sp.timestamp = hrt_absolute_time();

				if (_v_rates_sp_pub != nullptr) {
					orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

				} else if (_rates_sp_id) {
					_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
				}

				//}

			} else {
				// attitude controller disabled, poll rates setpoint topic
				if (_v_control_mode.flag_control_manual_enabled) {
					// manual rates control - ACRO mode
					_rates_sp = math::Vector<3>(_manual_control_sp.y, -_manual_control_sp.x,
								    _manual_control_sp.r).emult(_params.acro_rate_max);
					_thrust_sp = math::min(_manual_control_sp.z, MANUAL_THROTTLE_MAX_MULTICOPTER);

					// publish attitude rates setpoint
					_v_rates_sp.roll = _rates_sp(0);
					_v_rates_sp.pitch = _rates_sp(1);
					_v_rates_sp.yaw = _rates_sp(2);
					_v_rates_sp.thrust = _thrust_sp;
					_v_rates_sp.timestamp = hrt_absolute_time();

					if (_v_rates_sp_pub != nullptr) {
						orb_publish(_rates_sp_id, _v_rates_sp_pub, &_v_rates_sp);

					} else if (_rates_sp_id) {
						_v_rates_sp_pub = orb_advertise(_rates_sp_id, &_v_rates_sp);
					}

				} else {
					// attitude controller disabled, poll rates setpoint topic
					vehicle_rates_setpoint_poll();
					_rates_sp(0) = _v_rates_sp.roll;
					_rates_sp(1) = _v_rates_sp.pitch;
					_rates_sp(2) = _v_rates_sp.yaw;
					_thrust_sp = _v_rates_sp.thrust;
				}
			} */

            /*
			if (_v_control_mode.flag_control_rates_enabled) {
				control_attitude_rates(dt);

				// publish actuator controls
				_actuators.control[0] = (PX4_ISFINITE(_att_control(0))) ? _att_control(0) : 0.0f;
				_actuators.control[1] = (PX4_ISFINITE(_att_control(1))) ? _att_control(1) : 0.0f;
				_actuators.control[2] = (PX4_ISFINITE(_att_control(2))) ? _att_control(2) : 0.0f;
				_actuators.control[3] = (PX4_ISFINITE(_thrust_sp)) ? _thrust_sp : 0.0f;
				_actuators.timestamp = hrt_absolute_time();
				_actuators.timestamp_sample = _ctrl_state.timestamp;

				_controller_status.roll_rate_integ = _rates_int(0);
				_controller_status.pitch_rate_integ = _rates_int(1);
				_controller_status.yaw_rate_integ = _rates_int(2);
				_controller_status.timestamp = hrt_absolute_time();

				if (!_actuators_0_circuit_breaker_enabled) {
					if (_actuators_0_pub != nullptr) {

						orb_publish(_actuators_id, _actuators_0_pub, &_actuators);
						perf_end(_controller_latency_perf);

					} else if (_actuators_id) {
						_actuators_0_pub = orb_advertise(_actuators_id, &_actuators);
					}

				}

				// publish controller status
				if (_controller_status_pub != nullptr) {
					orb_publish(ORB_ID(mc_att_ctrl_status), _controller_status_pub, &_controller_status);

				} else {
					_controller_status_pub = orb_advertise(ORB_ID(mc_att_ctrl_status), &_controller_status);
				}
			} */
		}

		perf_end(_loop_perf);
	}

	_control_task = -1;
	return;
}

// start function
int HippocampusPathControl::start()
{
	ASSERT(_control_task == -1);        // give a value -1

	// start the control task, performs any specific accounting, scheduler setup, etc.
	_control_task = px4_task_spawn_cmd("path_controller",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1500,
					   (px4_main_t)&HippocampusPathControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

 // main function
 int path_controller_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_att_control {start|stop|status}");
		return 1;
	}

    // if command is start, then first control if class exists already, if not, allocate new one
	if (!strcmp(argv[1], "start")) {

		if (path_controller::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

        // allocate new class HippocampusPathControl
		path_controller::g_control = new HippocampusPathControl;

        // check if class has been allocated, if not, give back failure
		if (path_controller::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

        // if function start() can not be called, delete instance of HippocampusPathControl and allocate null pointer
		if (OK != path_controller::g_control->start()) {
			delete path_controller::g_control;
			path_controller::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

    // if command is start, check if class exists, if not can not stop anything
	if (!strcmp(argv[1], "stop")) {
		if (path_controller::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

        // if class exists, delete it and allocate null pointer
		delete path_controller::g_control;
		path_controller::g_control = nullptr;
		return 0;
	}

    // if command is status and class exists, give back running, else give back not running
	if (!strcmp(argv[1], "status")) {
		if (path_controller::g_control) {
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