/****************************************************************************
 *
 *   Copyright (c) 2013 - 2016 PX4 Development Team. All rights reserved.
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
 * @file mc_pos_control_main.cpp
 * Multicopter position controller xxx.
 *
 * Original publication for the desired attitude generation:
 * Daniel Mellinger and Vijay Kumar. Minimum Snap Trajectory Generation and Control for Quadrotors.
 * Int. Conf. on Robotics and Automation, Shanghai, China, May 2011
 *
 * Also inspired by https://pixhawk.org/firmware/apps/fw_pos_control_l1
 *
 * The controller has two loops: P loop for position error and PID loop for velocity error.
 * Output of velocity controller is thrust vector that splitted to thrust direction
 * (i.e. rotation matrix for multicopter orientation) and thrust module (i.e. multicopter thrust itself).
 * Controller doesn't use Euler angles for work, they generated only for more human-friendly control and logging.
 *
 * @author Anton Babushkin <anton.babushkin@me.com>
 */

#include <px4_config.h>
#include <px4_defines.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <math.h>
#include <poll.h>
#include <drivers/drv_hrt.h>
#include <arch/board/board.h>

#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_rates_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/mc_virtual_attitude_setpoint.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/position_setpoint_triplet.h>
#include <uORB/topics/vehicle_global_velocity_setpoint.h>
#include <uORB/topics/vehicle_local_position_setpoint.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/sonar_distance.h>
#include <uORB/topics/alt_estimate.h>
//iffpc
#include <uORB/topics/pos_helper.h>
#include <uORB/topics/ukf_localization.h>

//iffpc
#include <systemlib/systemlib.h>
#include <systemlib/mavlink_log.h>
#include <mathlib/mathlib.h>
#include <lib/geo/geo.h>
#include <platforms/px4_defines.h>

#include <controllib/blocks.hpp>
#include <controllib/block/BlockParam.hpp>

#define TILT_COS_MAX	0.7f
#define SIGMA			0.000001f
#define MIN_DIST		0.01f
#define MANUAL_THROTTLE_MAX_MULTICOPTER	0.9f
#define ONE_G	9.8066f

float ref_height1,height1;
//float thrust1,alt_sp1,alt_now1;
uint64_t time1,time2,time_cha;
unsigned int count1=0;
float thrust_test;
float thrust_vertical;
//test by fxk
bool take_off=0;
//static float yx;
float P11,D11,I11;
float er,erp;
float u_pos_x0=0,u_pos_y0=0;
/**
 * Multicopter position control app start / stop handling function
 *
 * @ingroup apps
 */
extern "C" __EXPORT int mc_pos_control_main(int argc, char *argv[]);

class MulticopterPositionControl : public control::SuperBlock
{
public:
	/**
	 * Constructor
	 */
	MulticopterPositionControl();

	/**
	 * Destructor, also kills task.
	 */
	~MulticopterPositionControl();

	/**
	 * Start task.
	 *
	 * @return		OK on success.
	 */
	int		start();

	bool		cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
					  const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res);

private:
	bool		_task_should_exit;/**< if true, task should exit */

	//iffpc

	int		_control_task;			/**< task handle for task */
	orb_advert_t	_mavlink_log_pub;      /**< mavlink log advert */
	int     testtime;


	int		_vehicle_status_sub;		/**< vehicle status subscription */
	int		_vehicle_land_detected_sub;	/**< vehicle land detected subscription */
	int		_ctrl_state_sub;		/**< control state subscription */
	int		_att_sp_sub;			/**< vehicle attitude setpoint */
	int		_control_mode_sub;		/**< vehicle control mode subscription */
	int		_params_sub;			/**< notification of parameter updates */
	int		_manual_sub;			/**< notification of manual control updates */
	int		_arming_sub;			/**< arming status of outputs */
	int		_local_pos_sub;			/**< vehicle local position */
	int		_pos_sp_triplet_sub;		/**< position setpoint triplet */
	int		_local_pos_sp_sub;		/**< offboard local position setpoint */
	int		_global_vel_sp_sub;		/**< offboard global velocity setpoint */
	//IFFPC
	int 	_sonar_sub;
	int 	_alt_estimate_sub;
	int     _ukf_localization_sub;
    //iffpc
	int  _pos_helper_sub;

	orb_advert_t	_att_sp_pub;			/**< attitude setpoint publication */

	orb_advert_t	_local_pos_sp_pub;		/**< vehicle local position setpoint publication */
	orb_advert_t	_global_vel_sp_pub;		/**< vehicle global velocity setpoint publication */

	//orb_advert_t  _alt_ctrl_pub;
	orb_id_t _attitude_setpoint_id;


	struct vehicle_status_s 			_vehicle_status; 	/**< vehicle status */
	struct vehicle_land_detected_s 			_vehicle_land_detected;	/**< vehicle land detected */
	struct control_state_s				_ctrl_state;		/**< vehicle attitude */
	struct vehicle_attitude_setpoint_s		_att_sp;		/**< vehicle attitude setpoint */
	struct manual_control_setpoint_s		_manual;		/**< r/c channel data */
	struct vehicle_control_mode_s			_control_mode;		/**< vehicle control mode */
	struct actuator_armed_s				_arming;		/**< actuator arming status */
	struct vehicle_local_position_s			_local_pos;		/**< vehicle local position */
	struct position_setpoint_triplet_s		_pos_sp_triplet;	/**< vehicle global position setpoint triplet */
	struct vehicle_local_position_setpoint_s	_local_pos_sp;		/**< vehicle local position setpoint */
	struct vehicle_global_velocity_setpoint_s	_global_vel_sp;		/**< vehicle global velocity setpoint */
	struct sonar_distance_s 	sonar;
	struct alt_estimate_s 		alt;
	//iffpc
	struct pos_helper_s    pos_helper;
	struct ukf_localization_s ukf_localization;
	//memset(&alt_ctrl_s, 0, sizeof(alt_ctrl_s));

	//orb_advert_t  _alt_ctrl_pub = orb_advertise(ORB_ID(alt_ctrl), &alt_ctrl_control);		/*publish altitude control data*/

	control::BlockParamFloat _manual_thr_min;
	control::BlockParamFloat _manual_thr_max;

	control::BlockDerivative _vel_x_deriv;
	control::BlockDerivative _vel_y_deriv;
	control::BlockDerivative _vel_z_deriv;

	//IFFPC
	//struct alt_ctrl_s    alt_ctrl_control;
	//memset(&alt_ctrl_control, 0, sizeof(alt_ctrl_control));
	//orb_advert_t  _alt_ctrl_pub = orb_advertise(ORB_ID(alt_ctrl), &alt_ctrl_control);		/*publish altitude control data*/

	struct {
		param_t thr_min;
		param_t thr_max;
		param_t thr_hover;
		param_t alt_ctl_dz;
		param_t alt_ctl_dy;
		param_t z_p;
		param_t z_vel_p;
		param_t z_vel_i;
		param_t z_vel_d;
		param_t z_vel_max_up;
		param_t z_vel_max_down;
		param_t z_ff;
		param_t xy_p;
		param_t xy_vel_p;
		param_t xy_vel_i;
		param_t xy_vel_d;
		param_t xy_vel_max;
		param_t xy_vel_cruise;
		param_t xy_ff;
		param_t tilt_max_air;
		param_t land_speed;
		param_t tko_speed;
		param_t tilt_max_land;
		param_t man_roll_max;
		param_t man_pitch_max;
		param_t man_yaw_max;
		param_t global_yaw_max;
		param_t mc_att_yaw_p;
		param_t hold_xy_dz;
		param_t hold_max_xy;
		param_t hold_max_z;
		param_t acc_hor_max;
		param_t alt_mode;
		param_t opt_recover;

	}		_params_handles;		/**< handles for interesting parameters  */

	struct {
		float thr_min;
		float thr_max;
		float thr_hover;
		float alt_ctl_dz;
		float alt_ctl_dy;
		float tilt_max_air;
		float land_speed;
		float tko_speed;
		float tilt_max_land;
		float man_roll_max;
		float man_pitch_max;
		float man_yaw_max;
		float global_yaw_max;
		float mc_att_yaw_p;
		float hold_xy_dz;
		float hold_max_xy;
		float hold_max_z;
		float acc_hor_max;
		float vel_max_up;
		float vel_max_down;
		uint32_t alt_mode;

		int opt_recover;

		math::Vector<3> pos_p;
		math::Vector<3> vel_p;
		math::Vector<3> vel_i;
		math::Vector<3> vel_d;
		math::Vector<3> vel_ff;
		math::Vector<3> vel_max;
		math::Vector<3> vel_cruise;
		math::Vector<3> sp_offs_max;
	}		_params;

	struct map_projection_reference_s _ref_pos;
	float _ref_alt;
	hrt_abstime _ref_timestamp;

	bool _reset_pos_sp;
	bool _reset_alt_sp;
	bool _mode_auto;
	bool _pos_hold_engaged;
	bool _alt_hold_engaged;
	bool _run_pos_control;
	bool _run_alt_control;

	math::Vector<3> _pos;
	math::Vector<3> _pos_sp;
	math::Vector<3> _vel;
	math::Vector<3> _vel_sp;
	math::Vector<3> _vel_prev;			/**< velocity on previous step */
	math::Vector<3> _vel_ff;
	math::Vector<3> _vel_sp_prev;
	math::Vector<3> _vel_err_d;		/**< derivative of current velocity */

	math::Matrix<3, 3> _R;			/**< rotation matrix from attitude quaternions */
	float _yaw;				/**< yaw angle (euler) */
	bool _in_landing;	/**< the vehicle is in the landing descent */
	bool _lnd_reached_ground; /**< controller assumes the vehicle has reached the ground after landing */
	bool _takeoff_jumped;
	float _vel_z_lp;
	float _acc_z_lp;
	float _takeoff_thrust_sp;
	bool control_vel_enabled_prev;	/**< previous loop was in velocity controlled mode (control_state.flag_control_velocity_enabled) */

	/**
	 * Update our local parameter cache.
	 */
	int		parameters_update(bool force);

	/**
	 * Update control outputs
	 */
	void		control_update();

	/**
	 * Check for changes in subscribed topics.
	 */
	void		poll_subscriptions();

	static float	scale_control(float ctl, float end, float dz, float dy);
	static float    throttle_curve(float ctl, float ctr);

	/**
	 * Update reference for local position projection
	 */
	void		update_ref();
	/**
	 * Reset position setpoint to current position.
	 *
	 * This reset will only occur if the _reset_pos_sp flag has been set.
	 * The general logic is to first "activate" the flag in the flight
	 * regime where a switch to a position control mode should hold the
	 * very last position. Once switching to a position control mode
	 * the last position is stored once.
	 */
	void		reset_pos_sp();

	/**
	 * Reset altitude setpoint to current altitude.
	 *
	 * This reset will only occur if the _reset_alt_sp flag has been set.
	 * The general logic follows the reset_pos_sp() architecture.
	 */
	void		reset_alt_sp();

	/**
	 * Check if position setpoint is too far from current position and adjust it if needed.
	 */
	void		limit_pos_sp_offset();

	/**
	 * Set position setpoint using manual control
	 */
	void		control_manual(float dt);

	float 		caculate_thrust(float mz);

	/**
	 * Set position setpoint using offboard control
	 */
	void		control_offboard(float dt);

	/**
	 * Set position setpoint for AUTO
	 */
	void		control_auto(float dt);

	/**
	 * Select between barometric and global (AMSL) altitudes
	 */
	void		select_alt(bool global);

	/**
	 * Shim for calling task_main from task_create.
	 */
	static void	task_main_trampoline(int argc, char *argv[]);

	/**
	 * Main sensor collection task.
	 */
	void		task_main();
};

namespace pos_control
{

MulticopterPositionControl	*g_control;
}

MulticopterPositionControl::MulticopterPositionControl() :
	SuperBlock(NULL, "MPC"),
	_task_should_exit(false),
	_control_task(-1),
	_mavlink_log_pub(nullptr),
    testtime(0),
	/* subscriptions */
	_ctrl_state_sub(-1),
	_att_sp_sub(-1),
	_control_mode_sub(-1),
	_params_sub(-1),
	_manual_sub(-1),
	_arming_sub(-1),
	_local_pos_sub(-1),
	_pos_sp_triplet_sub(-1),
	_global_vel_sp_sub(-1),
	_sonar_sub(-1),
    //iffpc
	_ukf_localization_sub(-1),
	_pos_helper_sub(-1),

	/* publications */
	//_alt_ctrl_pub(nullptr),
	_att_sp_pub(nullptr),
	_local_pos_sp_pub(nullptr),
	_global_vel_sp_pub(nullptr),
	_attitude_setpoint_id(0),
	_vehicle_status{},
	_vehicle_land_detected{},
	_ctrl_state{},
	_att_sp{},
	_manual{},
	_control_mode{},
	_arming{},
	_local_pos{},
	_pos_sp_triplet{},
	_local_pos_sp{},
	_global_vel_sp{},
	//alt_ctrl_control{},
	//alt_ctrl_s{},
	_manual_thr_min(this, "MANTHR_MIN"),
	_manual_thr_max(this, "MANTHR_MAX"),
	_vel_x_deriv(this, "VELD"),
	_vel_y_deriv(this, "VELD"),
	_vel_z_deriv(this, "VELD"),
	_ref_alt(0.0f),
	_ref_timestamp(0),

	_reset_pos_sp(true),
	_reset_alt_sp(true),
	_mode_auto(false),
	_pos_hold_engaged(false),
	_alt_hold_engaged(false),
	_run_pos_control(true),
	_run_alt_control(true),
	_yaw(0.0f),
	_in_landing(false),
	_lnd_reached_ground(false),
	_takeoff_jumped(false),
	_vel_z_lp(0),
	_acc_z_lp(0),
	_takeoff_thrust_sp(0.0f),
	control_vel_enabled_prev(false)
{
	// Make the quaternion valid for control state
	_ctrl_state.q[0] = 1.0f;

	memset(&_ref_pos, 0, sizeof(_ref_pos));

	_params.pos_p.zero();
	_params.vel_p.zero();
	_params.vel_i.zero();
	_params.vel_d.zero();
	_params.vel_max.zero();
	_params.vel_cruise.zero();
	_params.vel_ff.zero();
	_params.sp_offs_max.zero();

	_pos.zero();
	_pos_sp.zero();
	_vel.zero();
	_vel_sp.zero();
	_vel_prev.zero();
	_vel_ff.zero();
	_vel_sp_prev.zero();
	_vel_err_d.zero();

	_R.identity();

	_params_handles.thr_min		= param_find("MPC_THR_MIN");
	_params_handles.thr_max		= param_find("MPC_THR_MAX");
	_params_handles.thr_hover	= param_find("MPC_THR_HOVER");
	_params_handles.alt_ctl_dz	= param_find("MPC_ALTCTL_DZ");
	_params_handles.alt_ctl_dy	= param_find("MPC_ALTCTL_DY");
	_params_handles.z_p		= param_find("MPC_Z_P");
	_params_handles.z_vel_p		= param_find("MPC_Z_VEL_P");
	_params_handles.z_vel_i		= param_find("MPC_Z_VEL_I");
	_params_handles.z_vel_d		= param_find("MPC_Z_VEL_D");
	_params_handles.z_vel_max_up	= param_find("MPC_Z_VEL_MAX_UP");
	_params_handles.z_vel_max_down	= param_find("MPC_Z_VEL_MAX");

	// transitional support: Copy param values from max to down
	// param so that max param can be renamed in 1-2 releases
	// (currently at 1.3.0)
	float p;
	param_get(param_find("MPC_Z_VEL_MAX"), &p);
	param_set(param_find("MPC_Z_VEL_MAX_DN"), &p);

	_params_handles.z_ff		= param_find("MPC_Z_FF");
	_params_handles.xy_p		= param_find("MPC_XY_P");
	_params_handles.xy_vel_p	= param_find("MPC_XY_VEL_P");
	_params_handles.xy_vel_i	= param_find("MPC_XY_VEL_I");
	_params_handles.xy_vel_d	= param_find("MPC_XY_VEL_D");
	_params_handles.xy_vel_max	= param_find("MPC_XY_VEL_MAX");
	_params_handles.xy_vel_cruise	= param_find("MPC_XY_CRUISE");
	_params_handles.xy_ff		= param_find("MPC_XY_FF");
	_params_handles.tilt_max_air	= param_find("MPC_TILTMAX_AIR");
	_params_handles.land_speed	= param_find("MPC_LAND_SPEED");
	_params_handles.tko_speed	= param_find("MPC_TKO_SPEED");
	_params_handles.tilt_max_land	= param_find("MPC_TILTMAX_LND");
	_params_handles.man_roll_max = param_find("MPC_MAN_R_MAX");
	_params_handles.man_pitch_max = param_find("MPC_MAN_P_MAX");
	_params_handles.man_yaw_max = param_find("MPC_MAN_Y_MAX");
	_params_handles.global_yaw_max = param_find("MC_YAWRATE_MAX");
	_params_handles.mc_att_yaw_p = param_find("MC_YAW_P");
	_params_handles.hold_xy_dz = param_find("MPC_HOLD_XY_DZ");
	_params_handles.hold_max_xy = param_find("MPC_HOLD_MAX_XY");
	_params_handles.hold_max_z = param_find("MPC_HOLD_MAX_Z");
	_params_handles.acc_hor_max = param_find("MPC_ACC_HOR_MAX");
	_params_handles.alt_mode = param_find("MPC_ALT_MODE");
	_params_handles.opt_recover = param_find("VT_OPT_RECOV_EN");

	/* fetch initial parameter values */
	parameters_update(true);
}

MulticopterPositionControl::~MulticopterPositionControl()
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

	pos_control::g_control = nullptr;
}

int
MulticopterPositionControl::parameters_update(bool force)
{
	bool updated;
	struct parameter_update_s param_upd;

	orb_check(_params_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(parameter_update), _params_sub, &param_upd);
	}

	if (updated || force) {
		/* update C++ param system */
		updateParams();

		/* update legacy C interface params */
		param_get(_params_handles.thr_min, &_params.thr_min);
		param_get(_params_handles.thr_max, &_params.thr_max);
		param_get(_params_handles.thr_hover, &_params.thr_hover);
		param_get(_params_handles.alt_ctl_dz, &_params.alt_ctl_dz);
		param_get(_params_handles.alt_ctl_dy, &_params.alt_ctl_dy);
		param_get(_params_handles.tilt_max_air, &_params.tilt_max_air);
		_params.tilt_max_air = math::radians(_params.tilt_max_air);
		param_get(_params_handles.land_speed, &_params.land_speed);
		param_get(_params_handles.tko_speed, &_params.tko_speed);
		param_get(_params_handles.tilt_max_land, &_params.tilt_max_land);
		_params.tilt_max_land = math::radians(_params.tilt_max_land);

		float v;
		uint32_t v_i;
		param_get(_params_handles.xy_p, &v);
		_params.pos_p(0) = v;
		_params.pos_p(1) = v;
		param_get(_params_handles.z_p, &v);
		_params.pos_p(2) = v;
		param_get(_params_handles.xy_vel_p, &v);
		_params.vel_p(0) = v;
		_params.vel_p(1) = v;
		param_get(_params_handles.z_vel_p, &v);
		_params.vel_p(2) = v;
		param_get(_params_handles.xy_vel_i, &v);
		_params.vel_i(0) = v;
		_params.vel_i(1) = v;
		param_get(_params_handles.z_vel_i, &v);
		_params.vel_i(2) = v;
		param_get(_params_handles.xy_vel_d, &v);
		_params.vel_d(0) = v;
		_params.vel_d(1) = v;
		param_get(_params_handles.z_vel_d, &v);
		_params.vel_d(2) = v;
		param_get(_params_handles.xy_vel_max, &v);
		_params.vel_max(0) = v;
		_params.vel_max(1) = v;
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_max_up = v;
		_params.vel_max(2) = v;
		param_get(_params_handles.z_vel_max_down, &v);
		_params.vel_max_down = v;
		param_get(_params_handles.xy_vel_cruise, &v);
		_params.vel_cruise(0) = v;
		_params.vel_cruise(1) = v;
		/* using Z max up for now */
		param_get(_params_handles.z_vel_max_up, &v);
		_params.vel_cruise(2) = v;
		param_get(_params_handles.xy_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(0) = v;
		_params.vel_ff(1) = v;
		param_get(_params_handles.z_ff, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.vel_ff(2) = v;
		param_get(_params_handles.hold_xy_dz, &v);
		v = math::constrain(v, 0.0f, 1.0f);
		_params.hold_xy_dz = v;
		param_get(_params_handles.hold_max_xy, &v);
		_params.hold_max_xy = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.hold_max_z, &v);
		_params.hold_max_z = (v < 0.0f ? 0.0f : v);
		param_get(_params_handles.acc_hor_max, &v);
		_params.acc_hor_max = v;
		/*
		 * increase the maximum horizontal acceleration such that stopping
		 * within 1 s from full speed is feasible
		 */
		_params.acc_hor_max = math::max(_params.vel_cruise(0), _params.acc_hor_max);
		param_get(_params_handles.alt_mode, &v_i);
		_params.alt_mode = v_i;

		int i;
		param_get(_params_handles.opt_recover, &i);
		_params.opt_recover = i;

		_params.sp_offs_max = _params.vel_cruise.edivide(_params.pos_p) * 2.0f;

		/* mc attitude control parameters*/
		/* manual control scale */
		param_get(_params_handles.man_roll_max, &_params.man_roll_max);
		param_get(_params_handles.man_pitch_max, &_params.man_pitch_max);
		param_get(_params_handles.man_yaw_max, &_params.man_yaw_max);
		param_get(_params_handles.global_yaw_max, &_params.global_yaw_max);
		_params.man_roll_max = math::radians(_params.man_roll_max);
		_params.man_pitch_max = math::radians(_params.man_pitch_max);
		_params.man_yaw_max = math::radians(_params.man_yaw_max);
		_params.global_yaw_max = math::radians(_params.global_yaw_max);

		param_get(_params_handles.mc_att_yaw_p, &v);
		_params.mc_att_yaw_p = v;

		/* takeoff and land velocities should not exceed maximum */
		_params.tko_speed = fminf(_params.tko_speed, _params.vel_max_up);
		_params.land_speed = fminf(_params.land_speed, _params.vel_max_down);
	}

	return OK;
}

void
MulticopterPositionControl::poll_subscriptions()
{
	bool updated;

	orb_check(_vehicle_status_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_status), _vehicle_status_sub, &_vehicle_status);

		/* set correct uORB ID, depending on if vehicle is VTOL or not */
		if (!_attitude_setpoint_id) {
			if (_vehicle_status.is_vtol) {
				_attitude_setpoint_id = ORB_ID(mc_virtual_attitude_setpoint);

			} else {
				_attitude_setpoint_id = ORB_ID(vehicle_attitude_setpoint);
			}
		}
	}

	orb_check(_vehicle_land_detected_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);
	}

	orb_check(_ctrl_state_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &_ctrl_state);

		/* get current rotation matrix and euler angles from control state quaternions */
		math::Quaternion q_att(_ctrl_state.q[0], _ctrl_state.q[1], _ctrl_state.q[2], _ctrl_state.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_yaw = euler_angles(2);
	}

	orb_check(_att_sp_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp);
	}

	orb_check(_control_mode_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(vehicle_control_mode), _control_mode_sub, &_control_mode);
	}

	orb_check(_manual_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(manual_control_setpoint), _manual_sub, &_manual);
		//iffpc
		_manual.x=pos_helper.mx;
		_manual.y=pos_helper.my;
		_manual.r=pos_helper.mr;
	   // printf("1066 _manual.x=%.3f  \n",(double)_manual.x);
	}

	orb_check(_arming_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(actuator_armed), _arming_sub, &_arming);
	}

	//iffpc
	//orb_check(_ukf_localization_sub, &updated);

	//if (updated) {
	   orb_copy(ORB_ID(ukf_localization), _ukf_localization_sub, &ukf_localization);
	//}
	  // printf("U_x=%.3f\n",(double)ukf_localization.x);
	  // printf("U_Y=%.3f\n",(double)ukf_localization.y);

	orb_check(_local_pos_sub, &updated);

    if (updated) {
		orb_copy(ORB_ID(vehicle_local_position), _local_pos_sub, &_local_pos);
		//iffpc
		//_local_pos.vx=ukf_localization.vx;
		_local_pos.x=ukf_localization.x;
		//_local_pos.vy=ukf_localization.vy;
		_local_pos.y=ukf_localization.y;
	}

		//warnx("745 poshelper x =%.3f",(double)pos_helper.mx);
		//warnx("745 poshelper y=%.3f",(double)pos_helper.my);
		//warnx("745 poshelper z=%.3f",(double)pos_helper.mz);
		//warnx("745 poshelper r=%.3f",(double)pos_helper.mr);
	/*
	orb_check(_sonar_sub,&updated);

	if(updated)
	{
		orb_copy(ORB_ID(sonar_distance), _sonar_sub, &sonar);
	}	*/
}

float
MulticopterPositionControl::scale_control(float ctl, float end, float dz, float dy)
{
	if (ctl > dz) {
		return dy + (ctl - dz) * (1.0f - dy) / (end - dz);

	} else if (ctl < -dz) {
		return -dy + (ctl + dz) * (1.0f - dy) / (end - dz);

	} else {
		return ctl * (dy / dz);
	}
}

float
MulticopterPositionControl::throttle_curve(float ctl, float ctr)
{
	/* piecewise linear mapping: 0:ctr -> 0:0.5
	 * and ctr:1 -> 0.5:1 */
	if (ctl < 0.5f) {
		return 2 * ctl * ctr;

	} else {
		return ctr + 2 * (ctl - 0.5f) * (1.0f - ctr);
	}
}

void
MulticopterPositionControl::task_main_trampoline(int argc, char *argv[])
{
	pos_control::g_control->task_main();
}

void
MulticopterPositionControl::update_ref()
{
	if (_local_pos.ref_timestamp != _ref_timestamp) {
		double lat_sp, lon_sp;
		float alt_sp = 0.0f;

		if (_ref_timestamp != 0) {
			/* calculate current position setpoint in global frame */
			map_projection_reproject(&_ref_pos, _pos_sp(0), _pos_sp(1), &lat_sp, &lon_sp);
			alt_sp = _ref_alt - _pos_sp(2);
		}

		/* update local projection reference */
		map_projection_init(&_ref_pos, _local_pos.ref_lat, _local_pos.ref_lon);
		_ref_alt = _local_pos.ref_alt;

		if (_ref_timestamp != 0) {
			/* reproject position setpoint to new reference */
			map_projection_project(&_ref_pos, lat_sp, lon_sp, &_pos_sp.data[0], &_pos_sp.data[1]);
			_pos_sp(2) = -(alt_sp - _ref_alt);
		}

		_ref_timestamp = _local_pos.ref_timestamp;
	}
}

void
MulticopterPositionControl::reset_pos_sp()
{
	if (_reset_pos_sp) {
		_reset_pos_sp = false;

		// we have logic in the main function which chooses the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// position in a special way. In position control mode the position will be reset anyway until the vehicle has reduced speed.
		_pos_sp(0) = _pos(0);
		_pos_sp(1) = _pos(1);
	}
}

void
MulticopterPositionControl::reset_alt_sp()
{
	if (_reset_alt_sp) {
		_reset_alt_sp = false;

		// we have logic in the main function which choosed the velocity setpoint such that the attitude setpoint is
		// continuous when switching into velocity controlled mode, therefore, we don't need to bother about resetting
		// altitude in a special way
		_pos_sp(2) = _pos(2);
	}
}

void
MulticopterPositionControl::limit_pos_sp_offset()
{
	math::Vector<3> pos_sp_offs;
	pos_sp_offs.zero();

	if (_control_mode.flag_control_position_enabled) {
		pos_sp_offs(0) = (_pos_sp(0) - _pos(0)) / _params.sp_offs_max(0);
		pos_sp_offs(1) = (_pos_sp(1) - _pos(1)) / _params.sp_offs_max(1);
	}

	if (_control_mode.flag_control_altitude_enabled) {
		pos_sp_offs(2) = (_pos_sp(2) - _pos(2)) / _params.sp_offs_max(2);
	}

	float pos_sp_offs_norm = pos_sp_offs.length();

	if (pos_sp_offs_norm > 1.0f) {
		pos_sp_offs /= pos_sp_offs_norm;
		_pos_sp = _pos + pos_sp_offs.emult(_params.sp_offs_max);
	}
}

float
MulticopterPositionControl::caculate_thrust(float thrust)
{
	static float res,res_pre;
	static float error=0;
		static float height=0;
		static float Integral;
		static float error_pre=0;
		static float ref_height=0.4f;
		static bool flag_auto_first=true;		//first
		//static float current_thrust;
		//static float thrust_pre;
		static float u0 = 0.0f;
		static float h[5];
		static int x=0;
		//static int count = 0;
		//static bool ctrl_flag = false,alt_flag = false;
		float P=0,D=0;
		float z_v_p;
		float z_v_i=0;
		float z_v_d=0;
		 z_v_p=_params.pos_p(2);
		// P11=z_v_p;//_params.vel_p(2);
		 //z_v_i=_params.vel_i(2);
		// z_v_d=_params.vel_d(2);

		//bool updated;		// estimate
		bool updated1;		// sonar

		orb_check(_sonar_sub, &updated1);
		if(updated1)
		{
			//height_pre=height;
			time1=hrt_absolute_time()/1000;
			time_cha=time1-time2;
			time2=time1;
			count1++;
			orb_copy(ORB_ID(sonar_distance), _sonar_sub, &sonar);
			//printf("d=%.3f  \n",(double)sonar.distance_filter);
			if(sonar.distance_filter>=0)
			{
			if(x<4)
			{
					h[x]=sonar.distance_filter*0.01f;
					++x;
			}
			else
			{
				x=0;
				h[4]=sonar.distance_filter*0.01f;
			}}else{
			switch(x){
			case 1:
				h[1]=h[0]+(h[0]-h[4]);
				break;
			case 0:
				h[0]=h[4]+(h[4]-h[3]);
				break;
			default:
				h[x]=h[x-1]+(h[x-1]-h[x-2]);
			    break;
			}
			}

			for(int i=0;i<5;i++)
			{
				height=height+h[i];
			}
			height=height/5;

		}


		if(_vehicle_status.arming_state==1)		//manual control
		{
			//_manual.z =thrust ;
	//		if(*thrust>0.98f)
	//			*thrust=0.98f;
			//ref_height=0.5f;
			//ref_height = sonar.distance_filter*0.01f;
			//ref_height = alt.altitude;
			ref_height1 = height;		//debug
			//current_thrust = thrust;
			error_pre = 0;
			flag_auto_first = true;
			Integral = 0;
		}
		else			//auto altitude control
		{
			if(flag_auto_first)
			{
				//if((_local_pos.x<0.05f)&&(_local_pos.y<0.05f)&&(_local_pos.vy<0.05f)&&(_local_pos.vx<0.05f))
				//{
					flag_auto_first = false;
				//take_off=true;
				//}
				//*thrust = current_thrust;
				//thrust_pre = *thrust;
				//u0 = 0;
						//current_thrust;
			}
			else
			{
				//height=sonar.distance_filter*0.01f;_vehicle_status
				//height  = alt.altitude;
	//			if(ctrl_flag == true)
	//			{
	//				if(alt_flag == true)
	//				{
	//					height = alt.alt_with_sonar;
	//				}
	//				else
	//				{
	//					height  = alt.altitude;
	//				}
					//height1 = height;			//debug
					error=ref_height-height;
//ffpc
                    res_pre=res;
					if(_vehicle_status.arming_state==2)
					{
						Integral+=z_v_i*error;
						Integral=0.55f;
					}
					else
					{
						Integral=0;
					}
					if(Integral>0.515f)
						Integral = 0.56f;
					else if(Integral<-0.5f)
						Integral = 0.55f;
 //iffpc
					//Integral = 0.53f;
					P = z_v_p*error;
					D = z_v_d*(error-error_pre);

	//				if(D > 0.03f)
	//					D = 0.03f;
	//				else if(D < -0.03f)
	//					D = -0.03f;
					//*thrust = u0-_params.vel_p(2)*thrust_pre+_params.vel_i(2)*error-_params.vel_d(2)*error_pre+Integral;

					res = u0+P+Integral+D;

					//if(height<height_pre)
					//{
						if(res_pre<(res-0.005f))
						{
							res=res-0.005f;
						}

					//}
					//*thrust = _manual.z;
					if(res>0.98f)
						res = 0.98f;
					else if(res<0.5f)
						res = 0.55f;

					ref_height1=height;
					//if(x==1)
					error_pre = error;
					//thrust_pre = *thrust;

					//_att_sp.z_sp = ref_height;

	//			}
	//			else
	//			{
	//				*thrust = thrust_pre;
	//			}
			}
		}
		//thrust_test=_manual.z;IFFPC
		er=error;
		erp=error_pre;
		P11=P;
		D11=D;
		I11=Integral;
		height1=res;
		//iffpc
		//res=0.3f;
		return res;
	//	*thrust = _manual.z;IFFPC
}


void
MulticopterPositionControl::control_manual(float dt)
{


//	if (updatex) {
	orb_copy(ORB_ID(pos_helper), _pos_helper_sub, &pos_helper);
	//		}

	//warnx("1226 poshelper x =%.3f",(double)pos_helper.mx);
	//warnx("1226 poshelper y=%.3f",(double)pos_helper.my);
	//warnx("1226 poshelper z=%.3f",(double)pos_helper.mz);
	//warnx("1226 poshelper r=%.3f",(double)pos_helper.mr);


		//_manual.x=pos_helper.mx;
		//_manual.y=pos_helper.my;

	//	_manual.r=pos_helper.mr;
		//printf("1066 _manual.x=%.3f  \n",(double)_manual.x);
		//take_off=pos_helper.f1;
		//iffpc
	math::Vector<3> req_vel_sp; // X,Y in local frame and Z in global (D), in [-1,1] normalized range
	req_vel_sp.zero();

	if (_control_mode.flag_control_altitude_enabled) {
		/* set vertical velocity setpoint with throttle stick */
		req_vel_sp(2) = -scale_control(_manual.z - 0.5f, 0.5f, _params.alt_ctl_dz, _params.alt_ctl_dy); // D
	}

	if (_control_mode.flag_control_position_enabled) {
		/* set horizontal velocity setpoint with roll/pitch stick */
		req_vel_sp(0) = _manual.x;
		req_vel_sp(1) = _manual.y;

	}

	if (_control_mode.flag_control_altitude_enabled) {
		/* reset alt setpoint to current altitude if needed */
		reset_alt_sp();
	}

	if (_control_mode.flag_control_position_enabled) {
		/* reset position setpoint to current position if needed */
		reset_pos_sp();
	}

	/* limit velocity setpoint */
	float req_vel_sp_norm = req_vel_sp.length();

	if (req_vel_sp_norm > 1.0f) {
		req_vel_sp /= req_vel_sp_norm;
	}

	/* _req_vel_sp scaled to 0..1, scale it to max speed and rotate around yaw */
	math::Matrix<3, 3> R_yaw_sp;
	R_yaw_sp.from_euler(0.0f, 0.0f, _att_sp.yaw_body);
	math::Vector<3> req_vel_sp_scaled = R_yaw_sp * req_vel_sp.emult(
			_params.vel_cruise); // in NED and scaled to actual velocity

	/*
	 * assisted velocity mode: user controls velocity, but if	velocity is small enough, position
	 * hold is activated for the corresponding axis
	 */

	/* horizontal axes */
	if (_control_mode.flag_control_position_enabled) {
		/* check for pos. hold */
		if (fabsf(req_vel_sp(0)) < _params.hold_xy_dz && fabsf(req_vel_sp(1)) < _params.hold_xy_dz) {
			if (!_pos_hold_engaged) {
				if (_params.hold_max_xy < FLT_EPSILON || (fabsf(_vel(0)) < _params.hold_max_xy
						&& fabsf(_vel(1)) < _params.hold_max_xy)) {
					_pos_hold_engaged = true;

				} else {
					_pos_hold_engaged = false;
				}
			}

		} else {
			_pos_hold_engaged = false;
		}

		/* set requested velocity setpoint */
		//iffpc
		//warnx("_pos_hold_engaged=%.3f", (double)_pos_hold_engaged)  ==1;
		if (!_pos_hold_engaged) {
			_pos_sp(0) = _pos(0);
			_pos_sp(1) = _pos(1);
			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
			_vel_sp(0) = req_vel_sp_scaled(0);
			_vel_sp(1) = req_vel_sp_scaled(1);
		}
			//iffpc
	}

	//warnx("_pos_hold_engaged = %d",(int)_pos_hold_engaged);
//	printf("_vel_sp(0) = %.3f  \n",(double)_vel_sp(0));
	//warnx("_vel_sp(1) = %.3f",(double)_vel_sp(1));
	//warnx("_params.alt_ctl_dz = %.3f",(double)_params.alt_ctl_dz);
   // warnx("_params.alt_ctl_dy = %.3f",(double)_params.alt_ctl_dy);


	/* vertical axis */
	if (_control_mode.flag_control_altitude_enabled) {
		/* check for pos. hold */
		if (fabsf(req_vel_sp(2)) < FLT_EPSILON) {
			if (!_alt_hold_engaged) {
				if (_params.hold_max_z < FLT_EPSILON || fabsf(_vel(2)) < _params.hold_max_z) {
					_alt_hold_engaged = true;

				} else {
					_alt_hold_engaged = false;
				}
			}

		} else {
			_alt_hold_engaged = false;
			_pos_sp(2) = _pos(2);
		}

		/* set requested velocity setpoint */
		if (!_alt_hold_engaged) {
			_run_alt_control = false; /* request velocity setpoint to be used, instead of altitude setpoint */
			_vel_sp(2) = req_vel_sp_scaled(2);
		}
	}
	//iffpc
	//warnx("_vel_sp(2) = %.3f",(double)_vel_sp(2));
	//warnx("_alt_hold_engaged = %.3f",(double)_alt_hold_engaged);
	//warnx("FLT_EPSILON = %.3f",(double)FLT_EPSILON);
	//warnx("_params.hold_max_z = %.3f",(double)_params.hold_max_z);
	//warnx("FLT_EPSILON = %.3f",(double)FLT_EPSILON);
	//warnx("_params.hold_max_z = %.3f",(double)_params.hold_max_z);
}

void
MulticopterPositionControl::control_offboard(float dt)
{
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);
	}

	if (_pos_sp_triplet.current.valid) {
		if (_control_mode.flag_control_position_enabled && _pos_sp_triplet.current.position_valid) {
			/* control position */
			_pos_sp(0) = _pos_sp_triplet.current.x;
			_pos_sp(1) = _pos_sp_triplet.current.y;

		} else if (_control_mode.flag_control_velocity_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* control velocity */
			/* reset position setpoint to current position if needed */
			reset_pos_sp();

			/* set position setpoint move rate */
			_vel_sp(0) = _pos_sp_triplet.current.vx;
			_vel_sp(1) = _pos_sp_triplet.current.vy;

			_run_pos_control = false; /* request velocity setpoint to be used, instead of position setpoint */
		}

		if (_pos_sp_triplet.current.yaw_valid) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;

		} else if (_pos_sp_triplet.current.yawspeed_valid) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;
		}

		if (_control_mode.flag_control_altitude_enabled && _pos_sp_triplet.current.position_valid) {
			/* Control altitude */
			_pos_sp(2) = _pos_sp_triplet.current.z;

		} else if (_control_mode.flag_control_climb_rate_enabled && _pos_sp_triplet.current.velocity_valid) {
			/* reset alt setpoint to current altitude if needed */
			reset_alt_sp();

			/* set altitude setpoint move rate */
			_vel_sp(2) = _pos_sp_triplet.current.vz;

			_run_alt_control = false; /* request velocity setpoint to be used, instead of position setpoint */
		}

	} else {
		reset_pos_sp();
		reset_alt_sp();
	}
}

bool
MulticopterPositionControl::cross_sphere_line(const math::Vector<3> &sphere_c, float sphere_r,
		const math::Vector<3> line_a, const math::Vector<3> line_b, math::Vector<3> &res)
{
	/* project center of sphere on line */
	/* normalized AB */
	math::Vector<3> ab_norm = line_b - line_a;
	ab_norm.normalize();
	math::Vector<3> d = line_a + ab_norm * ((sphere_c - line_a) * ab_norm);
	float cd_len = (sphere_c - d).length();

	if (sphere_r > cd_len) {
		/* we have triangle CDX with known CD and CX = R, find DX */
		float dx_len = sqrtf(sphere_r * sphere_r - cd_len * cd_len);

		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			/* target waypoint is already behind us */
			res = line_b;

		} else {
			/* target is in front of us */
			res = d + ab_norm * dx_len; // vector A->B on line
		}

		return true;

	} else {
		/* have no roots, return D */
		res = d; /* go directly to line */

		/* previous waypoint is still in front of us */
		if ((sphere_c - line_a) * ab_norm < 0.0f) {
			res = line_a;
		}

		/* target waypoint is already behind us */
		if ((sphere_c - line_b) * ab_norm > 0.0f) {
			res = line_b;
		}

		return false;
	}
}

void MulticopterPositionControl::control_auto(float dt)
{
	/* reset position setpoint on AUTO mode activation or if we are not in MC mode */
	if (!_mode_auto || !_vehicle_status.is_rotary_wing) {
		if (!_mode_auto) {
			_mode_auto = true;
		}

		_reset_pos_sp = true;
		_reset_alt_sp = true;

		reset_pos_sp();
		reset_alt_sp();
	}

	//Poll position setpoint
	bool updated;
	orb_check(_pos_sp_triplet_sub, &updated);

	if (updated) {
		orb_copy(ORB_ID(position_setpoint_triplet), _pos_sp_triplet_sub, &_pos_sp_triplet);

		//Make sure that the position setpoint is valid
		if (!PX4_ISFINITE(_pos_sp_triplet.current.lat) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.lon) ||
		    !PX4_ISFINITE(_pos_sp_triplet.current.alt)) {
			_pos_sp_triplet.current.valid = false;
		}
	}

	bool current_setpoint_valid = false;
	bool previous_setpoint_valid = false;

	math::Vector<3> prev_sp;
	math::Vector<3> curr_sp;

	if (_pos_sp_triplet.current.valid) {

		/* project setpoint (JIN WEI DU)to local frame */
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.current.lat, _pos_sp_triplet.current.lon,
				       &curr_sp.data[0], &curr_sp.data[1]);
		curr_sp(2) = -(_pos_sp_triplet.current.alt - _ref_alt);

		if (PX4_ISFINITE(curr_sp(0)) &&
		    PX4_ISFINITE(curr_sp(1)) &&
		    PX4_ISFINITE(curr_sp(2))) {
			current_setpoint_valid = true;
		}
	}

	if (_pos_sp_triplet.previous.valid) {
		map_projection_project(&_ref_pos,
				       _pos_sp_triplet.previous.lat, _pos_sp_triplet.previous.lon,
				       &prev_sp.data[0], &prev_sp.data[1]);
		prev_sp(2) = -(_pos_sp_triplet.previous.alt - _ref_alt);

		if (PX4_ISFINITE(prev_sp(0)) &&
		    PX4_ISFINITE(prev_sp(1)) &&
		    PX4_ISFINITE(prev_sp(2))) {
			previous_setpoint_valid = true;
		}
	}

	if (current_setpoint_valid) {
		/* scaled space: 1 == position error resulting max allowed speed */

		math::Vector<3> cruising_speed = _params.vel_cruise;

		if (PX4_ISFINITE(_pos_sp_triplet.current.cruising_speed) &&
		    _pos_sp_triplet.current.cruising_speed > 0.1f) {
			cruising_speed(0) = _pos_sp_triplet.current.cruising_speed;
			cruising_speed(1) = _pos_sp_triplet.current.cruising_speed;
		}

		math::Vector<3> scale = _params.pos_p.edivide(cruising_speed);

		/* convert current setpoint to scaled space */
		math::Vector<3> curr_sp_s = curr_sp.emult(scale);

		/* by default use current setpoint as is */
		math::Vector<3> pos_sp_s = curr_sp_s;

		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION  ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) &&
		    previous_setpoint_valid) {

			/* follow "previous - current" line */

			if ((curr_sp - prev_sp).length() > MIN_DIST) {

				/* find X - cross point of unit sphere and trajectory */
				math::Vector<3> pos_s = _pos.emult(scale);
				math::Vector<3> prev_sp_s = prev_sp.emult(scale);
				math::Vector<3> prev_curr_s = curr_sp_s - prev_sp_s;
				math::Vector<3> curr_pos_s = pos_s - curr_sp_s;
				float curr_pos_s_len = curr_pos_s.length();

				if (curr_pos_s_len < 1.0f) {
					/* copter is closer to waypoint than unit radius */
					/* check next waypoint and use it to avoid slowing down when passing via waypoint */
					if (_pos_sp_triplet.next.valid) {
						math::Vector<3> next_sp;
						map_projection_project(&_ref_pos,
								       _pos_sp_triplet.next.lat, _pos_sp_triplet.next.lon,
								       &next_sp.data[0], &next_sp.data[1]);
						next_sp(2) = -(_pos_sp_triplet.next.alt - _ref_alt);

						if ((next_sp - curr_sp).length() > MIN_DIST) {
							math::Vector<3> next_sp_s = next_sp.emult(scale);

							/* calculate angle prev - curr - next */
							math::Vector<3> curr_next_s = next_sp_s - curr_sp_s;
							math::Vector<3> prev_curr_s_norm = prev_curr_s.normalized();

							/* cos(a) * curr_next, a = angle between current and next trajectory segments */
							float cos_a_curr_next = prev_curr_s_norm * curr_next_s;

							/* cos(b), b = angle pos - curr_sp - prev_sp */
							float cos_b = -curr_pos_s * prev_curr_s_norm / curr_pos_s_len;

							if (cos_a_curr_next > 0.0f && cos_b > 0.0f) {
								float curr_next_s_len = curr_next_s.length();

								/* if curr - next distance is larger than unit radius, limit it */
								if (curr_next_s_len > 1.0f) {
									cos_a_curr_next /= curr_next_s_len;
								}

								/* feed forward position setpoint offset */
								math::Vector<3> pos_ff = prev_curr_s_norm *
											 cos_a_curr_next * cos_b * cos_b * (1.0f - curr_pos_s_len) *
											 (1.0f - expf(-curr_pos_s_len * curr_pos_s_len * 20.0f));
								pos_sp_s += pos_ff;
							}
						}
					}

				} else {
					bool near = cross_sphere_line(pos_s, 1.0f, prev_sp_s, curr_sp_s, pos_sp_s);

					if (!near) {
						/* we're far away from trajectory, pos_sp_s is set to the nearest point on the trajectory */
						pos_sp_s = pos_s + (pos_sp_s - pos_s).normalized();
					}
				}
			}
		}

		/* move setpoint not faster than max allowed speed */
		math::Vector<3> pos_sp_old_s = _pos_sp.emult(scale);

		/* difference between current and desired position setpoints, 1 = max speed */
		math::Vector<3> d_pos_m = (pos_sp_s - pos_sp_old_s).edivide(_params.pos_p);
		float d_pos_m_len = d_pos_m.length();

		if (d_pos_m_len > dt) {
			pos_sp_s = pos_sp_old_s + (d_pos_m / d_pos_m_len * dt).emult(_params.pos_p);
		}

		/* scale result back to normal space */
		_pos_sp = pos_sp_s.edivide(scale);

		/* update yaw setpoint if needed */

		if (_pos_sp_triplet.current.yawspeed_valid
		    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET) {
			_att_sp.yaw_body = _att_sp.yaw_body + _pos_sp_triplet.current.yawspeed * dt;

		} else if (PX4_ISFINITE(_pos_sp_triplet.current.yaw)) {
			_att_sp.yaw_body = _pos_sp_triplet.current.yaw;
		}

		/*
		 * if we're already near the current takeoff setpoint don't reset in case we switch back to posctl.
		 * this makes the takeoff finish smoothly.
		 */
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
		     || _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER)
		    && _pos_sp_triplet.current.acceptance_radius > 0.0f
		    /* need to detect we're close a bit before the navigator switches from takeoff to next waypoint */
		    && (_pos - _pos_sp).length() < _pos_sp_triplet.current.acceptance_radius * 1.2f) {
			_reset_pos_sp = false;
			_reset_alt_sp = false;

			/* otherwise: in case of interrupted mission don't go to waypoint but stay at current position */

		} else {
			_reset_pos_sp = true;
			_reset_alt_sp = true;
		}

		// During a mission or in loiter it's safe to retract the landing gear.
		if ((_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_POSITION ||
		     _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LOITER) &&
		    !_vehicle_land_detected.landed) {
			_att_sp.landing_gear = 1.0f;

			// During takeoff and landing, we better put it down again.

		} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF ||
			   _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
			_att_sp.landing_gear = -1.0f;

		} else {
			// For the rest of the setpoint types, just leave it as is.
		}

	} else {
		/* no waypoint, do nothing, setpoint was already reset */
	}
}

void
MulticopterPositionControl::task_main()
{

	//float alt_count=0.5f;

	/*
	 * do subscriptions
	 */
	_vehicle_status_sub = orb_subscribe(ORB_ID(vehicle_status));
	_vehicle_land_detected_sub = orb_subscribe(ORB_ID(vehicle_land_detected));
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
	_control_mode_sub = orb_subscribe(ORB_ID(vehicle_control_mode));
	_params_sub = orb_subscribe(ORB_ID(parameter_update));
	_manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
	_arming_sub = orb_subscribe(ORB_ID(actuator_armed));
	_local_pos_sub = orb_subscribe(ORB_ID(vehicle_local_position));
	_pos_sp_triplet_sub = orb_subscribe(ORB_ID(position_setpoint_triplet));
	_local_pos_sp_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
	_global_vel_sp_sub = orb_subscribe(ORB_ID(vehicle_global_velocity_setpoint));
	_sonar_sub = orb_subscribe(ORB_ID(sonar_distance));
	_alt_estimate_sub = orb_subscribe(ORB_ID(alt_estimate));
    //iffpc
	_pos_helper_sub = orb_subscribe(ORB_ID(pos_helper));
	_ukf_localization_sub= orb_subscribe(ORB_ID(ukf_localization));

	parameters_update(true);

	/* initialize values of critical structs until first regular update */
	_arming.armed = false;

	/* get an initial update for all sensor and status data */
	poll_subscriptions();
	orb_copy(ORB_ID(ukf_localization), _ukf_localization_sub, &ukf_localization);
	ukf_localization.x=ukf_localization.x-u_pos_x0;
	ukf_localization.y=ukf_localization.y-u_pos_y0;
	_local_pos.x=ukf_localization.x;
	_local_pos.y=ukf_localization.y;

	/* We really need to know from the beginning if we're landed or in-air. */
	orb_copy(ORB_ID(vehicle_land_detected), _vehicle_land_detected_sub, &_vehicle_land_detected);

	bool reset_int_z = true;
	bool reset_int_z_manual = false;
	bool reset_int_xy = true;
	bool reset_yaw_sp = true;
	bool was_armed = false;

	hrt_abstime t_prev = 0;
	math::Vector<3> thrust_int;
	thrust_int.zero();

	// Let's be safe and have the landing gear down by default
	_att_sp.landing_gear = -1.0f;


	matrix::Dcmf R;
	R.identity();

	/* wakeup source */
	px4_pollfd_struct_t fds[1];

	fds[0].fd = _local_pos_sub;
	fds[0].events = POLLIN;//??????????????????????????????_local_pos_sub


	//struct alt_ctrl_s    alt_ctrl_control;
	//memset(&alt_ctrl_control, 0, sizeof(alt_ctrl_control));
	//orb_advert_t  _alt_ctrl_pub = orb_advertise(ORB_ID(alt_ctrl), &alt_ctrl_control);		/*publish altitude control data*/

	while (!_task_should_exit) {
		/* wait for up to 20ms for data */
		int pret = px4_poll(&fds[0], (sizeof(fds) / sizeof(fds[0])), 20);

		/* timed out - periodic check for _task_should_exit */
		if (pret == 0) {
			// Go through the loop anyway to copy manual input at 50 Hz.
		}

		/* this is undesirable but not much we can do */
		if (pret < 0) {
			warn("poll error %d, %d", pret, errno);
			continue;
		}

		poll_subscriptions();
		orb_copy(ORB_ID(ukf_localization), _ukf_localization_sub, &ukf_localization);
		//iffpc
		ukf_localization.x=ukf_localization.x-u_pos_x0;
		ukf_localization.y=ukf_localization.y-u_pos_y0;
		_local_pos.x=ukf_localization.x;
		_local_pos.y=ukf_localization.y;

		parameters_update(false);

		hrt_abstime t = hrt_absolute_time();
		float dt = t_prev != 0 ? (t - t_prev) * 0.000001f : 0.0f;
		t_prev = t;

		// set dt for control blocks
		setDt(dt);

		if (_control_mode.flag_armed && !was_armed) {
			/* reset setpoints and integrals on arming */
			_reset_pos_sp = true;
			_reset_alt_sp = true;
			_vel_sp_prev.zero();
			reset_int_z = true;
			reset_int_xy = true;
			reset_yaw_sp = true;
		}

		/* reset yaw and altitude setpoint for VTOL which are in fw mode */
		if (_vehicle_status.is_vtol) {
			if (!_vehicle_status.is_rotary_wing) {
				reset_yaw_sp = true;
				_reset_alt_sp = true;
			}
		}

		//Update previous arming state
		was_armed = _control_mode.flag_armed;

		update_ref();

		/* Update velocity derivative,
		 * independent of the current flight mode
		 */
		if (_local_pos.timestamp > 0) {

			if (PX4_ISFINITE(_local_pos.x) &&
			    PX4_ISFINITE(_local_pos.y) &&
			    PX4_ISFINITE(_local_pos.z)) {

				_pos(0) = _local_pos.x;
				_pos(1) = _local_pos.y;

				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
					_pos(2) = -_local_pos.dist_bottom;

				} else {
					_pos(2) = _local_pos.z;
				}
			}

			if (PX4_ISFINITE(_local_pos.vx) &&
			    PX4_ISFINITE(_local_pos.vy) &&
			    PX4_ISFINITE(_local_pos.vz)) {

				_vel(0) = _local_pos.vx;
				_vel(1) = _local_pos.vy;

				if (_params.alt_mode == 1 && _local_pos.dist_bottom_valid) {
					_vel(2) = -_local_pos.dist_bottom_rate;

				} else {
					_vel(2) = _local_pos.vz;
				}
			}

			_vel_err_d(0) = _vel_x_deriv.update(-_vel(0));
			_vel_err_d(1) = _vel_y_deriv.update(-_vel(1));
			_vel_err_d(2) = _vel_z_deriv.update(-_vel(2));
		}

		// reset the horizontal and vertical position hold flags for non-manual modes
		// or if position / altitude is not controlled
		if (!_control_mode.flag_control_position_enabled || !_control_mode.flag_control_manual_enabled) {
			_pos_hold_engaged = false;
		}

		if (!_control_mode.flag_control_altitude_enabled || !_control_mode.flag_control_manual_enabled) {
			_alt_hold_engaged = false;
		}

		if (_control_mode.flag_control_altitude_enabled ||
		    _control_mode.flag_control_position_enabled ||
		    _control_mode.flag_control_climb_rate_enabled ||
		    _control_mode.flag_control_velocity_enabled ||
		    _control_mode.flag_control_acceleration_enabled) {

			_vel_ff.zero();

			/* by default, run position/altitude controller. the control_* functions
			 * can disable this and run velocity controllers directly in this cycle */
			_run_pos_control = true;
			_run_alt_control = true;

			/* select control source */
			if (_control_mode.flag_control_manual_enabled) {
				/* manual control */
				control_manual(dt);
				_mode_auto = false;

			} else if (_control_mode.flag_control_offboard_enabled) {
				/* offboard control */
				control_offboard(dt);
				_mode_auto = false;

			} else {
				/* AUTO */
				control_auto(dt);
			}

			/* weather-vane mode for vtol: disable yaw control */
			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.disable_mc_yaw_control == true) {
				_att_sp.disable_mc_yaw_control = true;

			} else {
				/* reset in case of setpoint updates */
				_att_sp.disable_mc_yaw_control = false;
			}
            //iffpc
		//	printf(" 1733 U_x=%.3f\n",(double)ukf_localization.x);
		//	printf("1733 U_Y=%.3f\n",(double)ukf_localization.y);
			take_off=(_local_pos.vy<0.06f)&&(_local_pos.vx<0.06f)&&((_local_pos.x<5.0f)&&(_local_pos.x>-5.0f))&&((_local_pos.y<5.0f)&&(_local_pos.y>-5.0f));//(_local_pos.x<0.05f)&&(_local_pos.y<0.05f)&&;
			if((_vehicle_status.arming_state==2)&&(take_off==true))
			{
				_vehicle_land_detected.landed=0;
				u_pos_x0=ukf_localization.x;
				u_pos_y0=ukf_localization.y;
	    	}//iffpc
			if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
			    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_IDLE) {
				/* idle state, don't run controller and set zero thrust */
				R.identity();
				matrix::Quatf qd = R;
				memcpy(&_att_sp.q_d[0], qd.data(), sizeof(_att_sp.q_d));
				_att_sp.q_d_valid = true;

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

			} else if (_control_mode.flag_control_manual_enabled
				   && _vehicle_land_detected.landed) {
				//iffpc
				//warnx("1717 ");
				/* don't run controller when landed */
				_reset_pos_sp = true;
				_reset_alt_sp = true;
				_mode_auto = false;
				reset_int_z = true;
				reset_int_xy = true;

				R.identity();

				_att_sp.roll_body = 0.0f;
				_att_sp.pitch_body = 0.0f;
				_att_sp.yaw_body = _yaw;
				_att_sp.thrust = 0.0f;

				_att_sp.timestamp = hrt_absolute_time();

				/* publish attitude setpoint */
				if (_att_sp_pub != nullptr) {
					orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);

				} else if (_attitude_setpoint_id) {
					_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
				}

			} else {
				//iffpc
								//	printf("1743 \n");
				/* run position & altitude controllers, if enabled (otherwise use already computed velocity setpoints) */
				if (_run_pos_control) {
					_vel_sp(0) = (_pos_sp(0) - _pos(0)) * _params.pos_p(0);
					//iffpc
					//printf(" 1744 Vxsp = %.3f \n",(double)_vel_sp(0));
					_vel_sp(1) = (_pos_sp(1) - _pos(1)) * _params.pos_p(1);
				}

				// guard against any bad velocity values

				bool velocity_valid = PX4_ISFINITE(_pos_sp_triplet.current.vx) &&
						      PX4_ISFINITE(_pos_sp_triplet.current.vy) &&
						      _pos_sp_triplet.current.velocity_valid;

				// do not go slower than the follow target velocity when position tracking is active (set to valid)

				if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
				    velocity_valid &&
				    _pos_sp_triplet.current.position_valid) {

					math::Vector<3> ft_vel(_pos_sp_triplet.current.vx, _pos_sp_triplet.current.vy, 0);

					float cos_ratio = (ft_vel * _vel_sp) / (ft_vel.length() * _vel_sp.length());

					// only override velocity set points when uav is traveling in same direction as target and vector component
					// is greater than calculated position set point velocity component

					if (cos_ratio > 0) {
						ft_vel *= (cos_ratio);
						// min speed a little faster than target vel
						ft_vel += ft_vel.normalized() * 1.5f;

					} else {
						ft_vel.zero();
					}

					_vel_sp(0) = fabs(ft_vel(0)) > fabs(_vel_sp(0)) ? ft_vel(0) : _vel_sp(0);
					_vel_sp(1) = fabs(ft_vel(1)) > fabs(_vel_sp(1)) ? ft_vel(1) : _vel_sp(1);

					// track target using velocity only

				} else if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_FOLLOW_TARGET &&
					   velocity_valid) {

					_vel_sp(0) = _pos_sp_triplet.current.vx;
					_vel_sp(1) = _pos_sp_triplet.current.vy;
				}

				if (_run_alt_control) {
					_vel_sp(2) = (_pos_sp(2) - _pos(2)) * _params.pos_p(2);
				}

				/* make sure velocity setpoint is saturated in xy*/
				float vel_norm_xy = sqrtf(_vel_sp(0) * _vel_sp(0) +
							  _vel_sp(1) * _vel_sp(1));

				if (vel_norm_xy > _params.vel_max(0)) {
					/* note assumes vel_max(0) == vel_max(1) */
					_vel_sp(0) = _vel_sp(0) * _params.vel_max(0) / vel_norm_xy;
					_vel_sp(1) = _vel_sp(1) * _params.vel_max(1) / vel_norm_xy;
				}

				/* make sure velocity setpoint is saturated in z*/
				if (_vel_sp(2) < -1.0f * _params.vel_max_up) {
					_vel_sp(2) = -1.0f * _params.vel_max_up;
				}

				if (_vel_sp(2) >  _params.vel_max_down) {
					_vel_sp(2) = _params.vel_max_down;
				}

				if (!_control_mode.flag_control_position_enabled) {
					_reset_pos_sp = true;
				}

				if (!_control_mode.flag_control_altitude_enabled) {
					_reset_alt_sp = true;
				}

				if (!_control_mode.flag_control_velocity_enabled) {
					_vel_sp_prev(0) = _vel(0);
					_vel_sp_prev(1) = _vel(1);
					_vel_sp(0) = 0.0f;
					_vel_sp(1) = 0.0f;
					control_vel_enabled_prev = false;
				}

				if (!_control_mode.flag_control_climb_rate_enabled) {
					_vel_sp(2) = 0.0f;
				}

				/* use constant descend rate when landing, ignore altitude setpoint */
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
					_vel_sp(2) = _params.land_speed;
				}

				/* special thrust setpoint generation for takeoff from ground */
				if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid
				    && _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
				    && _control_mode.flag_armed) {

					// check if we are not already in air.
					// if yes then we don't need a jumped takeoff anymore
					if (!_takeoff_jumped && !_vehicle_land_detected.landed && fabsf(_takeoff_thrust_sp) < FLT_EPSILON) {
						_takeoff_jumped = true;
					}
					//iffpc
					//printf("1824 _takeoff_jumped = %.3f",(double)_takeoff_jumped);
					if (!_takeoff_jumped) {
						// ramp thrust setpoint up
						if (_vel(2) > -(_params.tko_speed / 2.0f)) {
							_takeoff_thrust_sp += 0.5f * dt;
							_vel_sp.zero();
							_vel_prev.zero();

						} else {
							// copter has reached our takeoff speed. split the thrust setpoint up
							// into an integral part and into a P part
							thrust_int(2) = _takeoff_thrust_sp - _params.vel_p(2) * fabsf(_vel(2));
							thrust_int(2) = -math::constrain(thrust_int(2), _params.thr_min, _params.thr_max);
							_vel_sp_prev(2) = -_params.tko_speed;
							_takeoff_jumped = true;
							reset_int_z = false;
						}
					}

					if (_takeoff_jumped) {
						_vel_sp(2) = -_params.tko_speed;
					}

				} else {
					_takeoff_jumped = false;
					_takeoff_thrust_sp = 0.0f;
				}

				// limit total horizontal acceleration
				math::Vector<2> acc_hor;
				acc_hor(0) = (_vel_sp(0) - _vel_sp_prev(0)) / dt;
				acc_hor(1) = (_vel_sp(1) - _vel_sp_prev(1)) / dt;

				if (acc_hor.length() > _params.acc_hor_max) {
					acc_hor.normalize();
					acc_hor *= _params.acc_hor_max;
					math::Vector<2> vel_sp_hor_prev(_vel_sp_prev(0), _vel_sp_prev(1));
					math::Vector<2> vel_sp_hor = acc_hor * dt + vel_sp_hor_prev;
					_vel_sp(0) = vel_sp_hor(0);
					_vel_sp(1) = vel_sp_hor(1);
				}

				// limit vertical acceleration
				float acc_v = (_vel_sp(2) - _vel_sp_prev(2)) / dt;

				if (fabsf(acc_v) > 2 * _params.acc_hor_max) {
					acc_v /= fabsf(acc_v);
					_vel_sp(2) = acc_v * 2 * _params.acc_hor_max * dt + _vel_sp_prev(2);
				}

				_vel_sp_prev = _vel_sp;

				_global_vel_sp.vx = _vel_sp(0);
				_global_vel_sp.vy = _vel_sp(1);
				_global_vel_sp.vz = _vel_sp(2);

				/* publish velocity setpoint */
				if (_global_vel_sp_pub != nullptr) {
					orb_publish(ORB_ID(vehicle_global_velocity_setpoint), _global_vel_sp_pub, &_global_vel_sp);

				} else {
					_global_vel_sp_pub = orb_advertise(ORB_ID(vehicle_global_velocity_setpoint), &_global_vel_sp);
				}
				//iffpc
							//		printf("1913 \n");
				if (_control_mode.flag_control_climb_rate_enabled || _control_mode.flag_control_velocity_enabled ||
				    _control_mode.flag_control_acceleration_enabled) {
					//iffpc
				//	printf("1915 \n");
					/* reset integrals if needed */
					if (_control_mode.flag_control_climb_rate_enabled) {
						if (reset_int_z) {
							reset_int_z = false;
							float i = _params.thr_min;

							if (reset_int_z_manual) {
								i = _params.thr_hover;

								if (i < _params.thr_min) {
									i = _params.thr_min;

								} else if (i > _params.thr_max) {
									i = _params.thr_max;
								}
							}

							thrust_int(2) = -i;
						}

					} else {
						reset_int_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled) {
						if (reset_int_xy) {
							reset_int_xy = false;
							thrust_int(0) = 0.0f;
							thrust_int(1) = 0.0f;
						}

					} else {
						reset_int_xy = true;
					}

					/* velocity error */
					math::Vector<3> vel_err = _vel_sp - _vel;

					// check if we have switched from a non-velocity controlled mode into a velocity controlled mode
					// if yes, then correct xy velocity setpoint such that the attitude setpoint is continuous
					if (!control_vel_enabled_prev && _control_mode.flag_control_velocity_enabled) {

						matrix::Dcmf Rb = matrix::Quatf(_att_sp.q_d[0], _att_sp.q_d[1], _att_sp.q_d[2], _att_sp.q_d[3]);

						// choose velocity xyz setpoint such that the resulting thrust setpoint has the direction
						// given by the last attitude setpoint
						_vel_sp(0) = _vel(0) + (-Rb(0,
									    2) * _att_sp.thrust - thrust_int(0) - _vel_err_d(0) * _params.vel_d(0)) / _params.vel_p(0);
						_vel_sp(1) = _vel(1) + (-Rb(1,
									    2) * _att_sp.thrust - thrust_int(1) - _vel_err_d(1) * _params.vel_d(1)) / _params.vel_p(1);
						_vel_sp(2) = _vel(2) + (-Rb(2,
									    2) * _att_sp.thrust - thrust_int(2) - _vel_err_d(2) * _params.vel_d(2)) / _params.vel_p(2);
						_vel_sp_prev(0) = _vel_sp(0);
						_vel_sp_prev(1) = _vel_sp(1);
						_vel_sp_prev(2) = _vel_sp(2);
						control_vel_enabled_prev = true;

						// compute updated velocity error
						vel_err = _vel_sp - _vel;
					}

					/* thrust vector in NED frame */
					math::Vector<3> thrust_sp;
					//iffpc
				//	printf("1980 \n");
					if (_control_mode.flag_control_acceleration_enabled && _pos_sp_triplet.current.acceleration_valid) {
						thrust_sp = math::Vector<3>(_pos_sp_triplet.current.a_x, _pos_sp_triplet.current.a_y, _pos_sp_triplet.current.a_z);

					} else {
						thrust_sp = vel_err.emult(_params.vel_p) + _vel_err_d.emult(_params.vel_d) + thrust_int;
					}

					if (_pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_TAKEOFF
					    && !_takeoff_jumped && !_control_mode.flag_control_manual_enabled) {
						// for jumped takeoffs use special thrust setpoint calculated above
						thrust_sp.zero();
						thrust_sp(2) = -_takeoff_thrust_sp;
					}

					if (!_control_mode.flag_control_velocity_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(0) = 0.0f;
						thrust_sp(1) = 0.0f;
					}

					if (!_control_mode.flag_control_climb_rate_enabled && !_control_mode.flag_control_acceleration_enabled) {
						thrust_sp(2) = 0.0f;
					}
                     //iffpc
				//	printf("2005 \n");
					thrust_sp(2)=-caculate_thrust(0.5f);
					//printf("thrust=%.3f \n",(double)thrust_sp(2));
					/* limit thrust vector and check for saturation */
					bool saturation_xy = false;
					bool saturation_z = false;

					/* limit min lift */
					float thr_min = _params.thr_min;

					if (!_control_mode.flag_control_velocity_enabled && thr_min < 0.0f) {
						/* don't allow downside thrust direction in manual attitude mode */
						thr_min = 0.0f;
					}
//iffpc
					float thrust_abs = thrust_sp.length();
					float tilt_max = _params.tilt_max_air;
					float thr_max = _params.thr_max;
					/* filter vel_z over 1/8sec */
					_vel_z_lp = _vel_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * _vel(2);
					/* filter vel_z change over 1/8sec */
					float vel_z_change = (_vel(2) - _vel_prev(2)) / dt;
					_acc_z_lp = _acc_z_lp * (1.0f - dt * 8.0f) + dt * 8.0f * vel_z_change;

					/* adjust limits for landing mode */
					if (!_control_mode.flag_control_manual_enabled && _pos_sp_triplet.current.valid &&
					    _pos_sp_triplet.current.type == position_setpoint_s::SETPOINT_TYPE_LAND) {
						/* limit max tilt and min lift when landing */
						tilt_max = _params.tilt_max_land;

						if (thr_min < 0.0f) {
							thr_min = 0.0f;
						}

						/* descend stabilized, we're landing */
						if (!_in_landing && !_lnd_reached_ground
						    && (float)fabs(_acc_z_lp) < 0.1f
						    && _vel_z_lp > 0.5f * _params.land_speed) {
							_in_landing = true;
						}

						/* assume ground, cut thrust */
						if (_in_landing
						    && _vel_z_lp < 0.1f) {
							thr_max = 0.0f;
							_in_landing = false;
							_lnd_reached_ground = true;
						}

						/* once we assumed to have reached the ground always cut the thrust.
							Only free fall detection below can revoke this
						*/
						if (!_in_landing && _lnd_reached_ground) {
							thr_max = 0.0f;
						}

						/* if we suddenly fall, reset landing logic and remove thrust limit */
						if (_lnd_reached_ground
						    /* XXX: magic value, assuming free fall above 4m/s2 acceleration */
						    && (_acc_z_lp > 4.0f
							|| _vel_z_lp > 2.0f * _params.land_speed)) {
							thr_max = _params.thr_max;
							_in_landing = false;
							_lnd_reached_ground = false;
						}

					} else {
						_in_landing = false;
						_lnd_reached_ground = false;
					}

					/* limit min lift */
					if (-thrust_sp(2) < thr_min) {
						thrust_sp(2) = -thr_min;
						saturation_z = true;
					}

					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {

						/* limit max tilt */
						if (thr_min >= 0.0f && tilt_max < M_PI_F / 2 - 0.05f) {
							/* absolute horizontal thrust */
							float thrust_sp_xy_len = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();

							if (thrust_sp_xy_len > 0.01f) {
								/* max horizontal thrust for given vertical thrust*/
								float thrust_xy_max = -thrust_sp(2) * tanf(tilt_max);

								if (thrust_sp_xy_len > thrust_xy_max) {
									float k = thrust_xy_max / thrust_sp_xy_len;
									thrust_sp(0) *= k;
									thrust_sp(1) *= k;
									saturation_xy = true;
								}
							}
						}
					}

					if (_control_mode.flag_control_altitude_enabled) {
						/* thrust compensation for altitude only control modes */
						float att_comp;

						if (_R(2, 2) > TILT_COS_MAX) {
							att_comp = 1.0f / _R(2, 2);

						} else if (_R(2, 2) > 0.0f) {
							att_comp = ((1.0f / TILT_COS_MAX - 1.0f) / TILT_COS_MAX) * _R(2, 2) + 1.0f;
							saturation_z = true;

						} else {
							att_comp = 1.0f;
							saturation_z = true;
						}

						thrust_sp(2) *= att_comp;
					}

					/* limit max thrust */
					thrust_abs = thrust_sp.length(); /* recalculate because it might have changed */

					if (thrust_abs > thr_max) {
						if (thrust_sp(2) < 0.0f) {
							if (-thrust_sp(2) > thr_max) {
								/* thrust Z component is too large, limit it */
								thrust_sp(0) = 0.0f;
								thrust_sp(1) = 0.0f;
								thrust_sp(2) = -thr_max;
								saturation_xy = true;
								saturation_z = true;

							} else {
								/* preserve thrust Z component and lower XY, keeping altitude is more important than position */
								float thrust_xy_max = sqrtf(thr_max * thr_max - thrust_sp(2) * thrust_sp(2));
								float thrust_xy_abs = math::Vector<2>(thrust_sp(0), thrust_sp(1)).length();
								float k = thrust_xy_max / thrust_xy_abs;
								thrust_sp(0) *= k;
								thrust_sp(1) *= k;
								saturation_xy = true;
							}

						} else {
							/* Z component is negative, going down, simply limit thrust vector */
							float k = thr_max / thrust_abs;
							thrust_sp *= k;
							saturation_xy = true;
							saturation_z = true;
						}

						thrust_abs = thr_max;
					}

					/* update integrals */
					if (_control_mode.flag_control_velocity_enabled && !saturation_xy) {
						thrust_int(0) += vel_err(0) * _params.vel_i(0) * dt;
						thrust_int(1) += vel_err(1) * _params.vel_i(1) * dt;
					}

					if (_control_mode.flag_control_climb_rate_enabled && !saturation_z) {
						thrust_int(2) += vel_err(2) * _params.vel_i(2) * dt;

						/* protection against flipping on ground when landing */
						if (thrust_int(2) > 0.0f) {
							thrust_int(2) = 0.0f;
						}
					}

					/* calculate attitude setpoint from thrust vector */
					if (_control_mode.flag_control_velocity_enabled || _control_mode.flag_control_acceleration_enabled) {
						/* desired body_z axis = -normalize(thrust_vector) */
						math::Vector<3> body_x;
						math::Vector<3> body_y;
						math::Vector<3> body_z;

						if (thrust_abs > SIGMA) {
							body_z = -thrust_sp / thrust_abs;

						} else {
							/* no thrust, set Z axis to safe value */
							body_z.zero();
							body_z(2) = 1.0f;
						}

						/* vector of desired yaw direction in XY plane, rotated by PI/2 */
						math::Vector<3> y_C(-sinf(_att_sp.yaw_body), cosf(_att_sp.yaw_body), 0.0f);
						//iffpc
						//printf(" 2200 pitch_sp=%.3f  \n",(double)_att_sp.pitch_body);
						//printf(" 2201 pitch=%.3f  \n",(double)_att_sp.pitch_body);
					//	printf(" 2202  roll_sp=%.3f  \n",(double)_att_sp.roll_body);
						if (fabsf(body_z(2)) > SIGMA) {
							/* desired body_x axis, orthogonal to body_z */
							body_x = y_C % body_z;

							/* keep nose to front while inverted upside down */
							if (body_z(2) < 0.0f) {
								body_x = -body_x;
							}

							body_x.normalize();

						} else {
							/* desired thrust is in XY plane, set X downside to construct correct matrix,
							 * but yaw component will not be used actually */
							body_x.zero();
							body_x(2) = 1.0f;
						}

						/* desired body_y axis */
						body_y = body_z % body_x;

						/* fill rotation matrix */
						for (int i = 0; i < 3; i++) {
							R(i, 0) = body_x(i);
							R(i, 1) = body_y(i);
							R(i, 2) = body_z(i);
						}

						/* copy quaternion setpoint to attitude setpoint topic */
						matrix::Quatf q_sp = R;
						memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
						_att_sp.q_d_valid = true;

						/* calculate euler angles, for logging only, must not be used for control */
						matrix::Eulerf euler = R;
						_att_sp.roll_body = euler(0);
						_att_sp.pitch_body = euler(1);
						/* yaw already used to construct rot matrix, but actual rotation matrix can have different yaw near singularity *///iffpc
					//	printf("pitch=%.3f",(double)_att_sp.pitch_body);
					} else if (!_control_mode.flag_control_manual_enabled) {
						/* autonomous altitude control without position control (failsafe landing),
						 * force level attitude, don't change yaw */
						R = matrix::Eulerf(0.0f, 0.0f, _att_sp.yaw_body);

						/* copy quaternion setpoint to attitude setpoint topic */
						matrix::Quatf q_sp = R;
						memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
						_att_sp.q_d_valid = true;

						_att_sp.roll_body = 0.0f;
						_att_sp.pitch_body = 0.0f;
					}
					//warnx(" 2380 2380 thrust =%.3f",(double)_att_sp.thrust);
					_att_sp.thrust = thrust_abs;

					/* save thrust setpoint for logging */
					_local_pos_sp.acc_x = thrust_sp(0) * ONE_G;
					_local_pos_sp.acc_y = thrust_sp(1) * ONE_G;
					_local_pos_sp.acc_z = thrust_sp(2) * ONE_G;

					_att_sp.timestamp = hrt_absolute_time();


				} else {
					reset_int_z = true;
				}
			}

			/* fill local position, velocity and thrust setpoint */
			_local_pos_sp.timestamp = hrt_absolute_time();
			//iffpc
			_local_pos_sp.x = _pos_sp(0);
			_local_pos_sp.y = _pos_sp(1);
			//_local_pos_sp.x = 0.0;
			//			_local_pos_sp.y = 0.0;
			_local_pos_sp.z = _pos_sp(2);
			_local_pos_sp.yaw = _att_sp.yaw_body;
			//iffpc
			_local_pos_sp.vx = _vel_sp(0);
			_local_pos_sp.vy = _vel_sp(1);
			//_local_pos_sp.vx = 0.0 ;
			//_local_pos_sp.vy = 0.0 ;
			_local_pos_sp.vz = _vel_sp(2);
  //iffpc
			//_local_pos_sp.timestamp = hrt_absolute_time();
			//_local_pos_sp.x = _pos_sp(0);
			//_local_pos_sp.y = _pos_sp(1);
			//_local_pos_sp.z = _pos_sp(2);
			//_local_pos_sp.yaw = _att_sp.yaw_body;
			//_local_pos_sp.vx = 0.5f;//_vel_sp(0);
			//_local_pos_sp.vy = 0;    //_vel_sp(1);
			//_local_pos_sp.vz = 0.1f; //_vel_sp(2);


			/* publish local position setpoint */
			if (_local_pos_sp_pub != nullptr) {
				orb_publish(ORB_ID(vehicle_local_position_setpoint), _local_pos_sp_pub, &_local_pos_sp);
				//iffpc
				//printf("zsp = %.3f \n",(double)_local_pos_sp.z);
				//printf("vzsp = %.3f \n",(double)_local_pos_sp.vz);
				//printf("x = %.3f \n",(double)_pos(0));
				//printf("xsp = %.3f \n",(double)_local_pos_sp.x);
			//	printf(" 2280 Vxsp = %.3f \n",(double)_local_pos_sp.vx);
				//printf("ysp = %.3f \n",(double)_local_pos_sp.y);
				//printf(" 2280 Vysp = %.3f \n",(double)_local_pos_sp.vy);

			} else {
				_local_pos_sp_pub = orb_advertise(ORB_ID(vehicle_local_position_setpoint), &_local_pos_sp);
			}

		} else {
			/* position controller disabled, reset setpoints */
			_reset_alt_sp = true;
			_reset_pos_sp = true;
			_mode_auto = false;
			reset_int_z = true;
			reset_int_xy = true;
			control_vel_enabled_prev = false;

			/* store last velocity in case a mode switch to position control occurs */
			_vel_sp_prev = _vel;
		}

		/* generate attitude setpoint from manual controls */
		if (_control_mode.flag_control_manual_enabled && _control_mode.flag_control_attitude_enabled) {
		//	warnx(" 2454 2454 _vehicle_land_detected.landed =%.3f",(double)_vehicle_land_detected.landed);
			//iffpc
		//	_vehicle_land_detected.landed=0;
			//warnx(" 2476 2476 yaw =%.3f",(double)_yaw);
			//iffpc
			/* reset yaw setpoint to current position if needed */
			if (reset_yaw_sp) {
				reset_yaw_sp = false;
				_att_sp.yaw_body = _yaw;
			}
			/* do not move yaw while sitting on the ground */
			else if (!_vehicle_land_detected.landed &&
				 !(!_control_mode.flag_control_altitude_enabled && _manual.z < 0.1f)) {
				//iffpc
			//	_manual.r=0.2f;
				//iffpc
				/* we want to know the real constraint, and global overrides manual */
				const float yaw_rate_max = (_params.man_yaw_max < _params.global_yaw_max) ? _params.man_yaw_max :
							   _params.global_yaw_max;
				const float yaw_offset_max = yaw_rate_max / _params.mc_att_yaw_p;

				_att_sp.yaw_sp_move_rate = _manual.r * yaw_rate_max;
				float yaw_target = _wrap_pi(_att_sp.yaw_body + _att_sp.yaw_sp_move_rate * dt);
				float yaw_offs = _wrap_pi(yaw_target - _yaw);

				// If the yaw offset became too big for the system to track stop
				// shifting it, only allow if it would make the offset smaller again.
				if (fabsf(yaw_offs) < yaw_offset_max ||
				    (_att_sp.yaw_sp_move_rate > 0 && yaw_offs < 0) ||
				    (_att_sp.yaw_sp_move_rate < 0 && yaw_offs > 0)) {
					_att_sp.yaw_body = yaw_target;
				}
			}

			/* control throttle directly if no climb rate controller is active */
			if (!_control_mode.flag_control_climb_rate_enabled) {
				float thr_val = throttle_curve(_manual.z, _params.thr_hover);
				_att_sp.thrust = math::min(thr_val, _manual_thr_max.get());
				//warnx(" 2472 2472_att_sp thrust =%.3f",(double)_att_sp.thrust);
				//warnx(" 2472 2472_params.thr_hover thrust =%.3f",(double)_params.thr_hover);
				//iffpc
				//float getmaxz=_manual_thr_max.get();
				//warnx(" 2472 _manual_thr_max 2472 thrust =%.3f",(double)getmaxz);
				/* enforce minimum throttle if not landed */
				if (!_vehicle_land_detected.landed) {
					_att_sp.thrust = math::max(_att_sp.thrust, _manual_thr_min.get());
					//warnx("2476 2476 thrust =%.3f",(double)_att_sp.thrust);
				}
			}

			/* control roll and pitch directly if no aiding velocity controller is active */
			if (!_control_mode.flag_control_velocity_enabled) {
				_att_sp.roll_body = _manual.y * _params.man_roll_max;
				_att_sp.pitch_body = -_manual.x * _params.man_pitch_max;

				/* only if optimal recovery is not used, modify roll/pitch */
				if (_params.opt_recover <= 0) {
					// construct attitude setpoint rotation matrix. modify the setpoints for roll
					// and pitch such that they reflect the user's intention even if a yaw error
					// (yaw_sp - yaw) is present. In the presence of a yaw error constructing a rotation matrix
					// from the pure euler angle setpoints will lead to unexpected attitude behaviour from
					// the user's view as the euler angle sequence uses the  yaw setpoint and not the current
					// heading of the vehicle.

					// calculate our current yaw error
					float yaw_error = _wrap_pi(_att_sp.yaw_body - _yaw);

					// compute the vector obtained by rotating a z unit vector by the rotation
					// given by the roll and pitch commands of the user
					math::Vector<3> zB = {0, 0, 1};
					math::Matrix<3, 3> R_sp_roll_pitch;
					R_sp_roll_pitch.from_euler(_att_sp.roll_body, _att_sp.pitch_body, 0);
					math::Vector<3> z_roll_pitch_sp = R_sp_roll_pitch * zB;


					// transform the vector into a new frame which is rotated around the z axis
					// by the current yaw error. this vector defines the desired tilt when we look
					// into the direction of the desired heading
					math::Matrix<3, 3> R_yaw_correction;
					R_yaw_correction.from_euler(0.0f, 0.0f, -yaw_error);
					z_roll_pitch_sp = R_yaw_correction * z_roll_pitch_sp;

					// use the formula z_roll_pitch_sp = R_tilt * [0;0;1]
					// to calculate the new desired roll and pitch angles
					// R_tilt can be written as a function of the new desired roll and pitch
					// angles. we get three equations and have to solve for 2 unknowns
					_att_sp.pitch_body = asinf(z_roll_pitch_sp(0));
					_att_sp.roll_body = -atan2f(z_roll_pitch_sp(1), z_roll_pitch_sp(2));
				}

				/* copy quaternion setpoint to attitude setpoint topic */
				matrix::Quatf q_sp = matrix::Eulerf(_att_sp.roll_body, _att_sp.pitch_body, _att_sp.yaw_body);
				memcpy(&_att_sp.q_d[0], q_sp.data(), sizeof(_att_sp.q_d));
				_att_sp.q_d_valid = true;
			}

			if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_ON &&
			    !_vehicle_land_detected.landed) {
				_att_sp.landing_gear = 1.0f;

			} else if (_manual.gear_switch == manual_control_setpoint_s::SWITCH_POS_OFF) {
				_att_sp.landing_gear = -1.0f;
			}


			_att_sp.timestamp = hrt_absolute_time();

		} else {
			reset_yaw_sp = true;
			_att_sp.yaw_sp_move_rate = 0.0f;
		}

		/* update previous velocity for velocity controller D part */
		_vel_prev = _vel;

		/* publish attitude setpoint
		 * Do not publish if offboard is enabled but position/velocity/accel control is disabled,
		 * in this case the attitude setpoint is published by the mavlink app. Also do not publish
		 * if the vehicle is a VTOL and it's just doing a transition (the VTOL attitude control module will generate
		 * attitude setpoints for the transition).
		 */
		if (!(_control_mode.flag_control_offboard_enabled &&
		      !(_control_mode.flag_control_position_enabled ||
			_control_mode.flag_control_velocity_enabled ||
			_control_mode.flag_control_acceleration_enabled))) {

	/*		alt_ctrl_control.alt_measure = 0.2f+alt_count;
			alt_ctrl_control.alt_sp = 0.3f+alt_count;
			alt_ctrl_control.thrust = 0.4f+alt_count;
			alt_count+=0.5f;

			thrust1 = alt_ctrl_control.thrust;
			alt_sp1 = alt_ctrl_control.alt_sp;
			alt_now1 = alt_ctrl_control.alt_measure;

			if(_alt_ctrl_pub!=nullptr)
			{
				publish_count++;
				orb_publish(ORB_ID(alt_ctrl),&_alt_ctrl_pub, &alt_ctrl_control);
			}
			else
			{
				_alt_ctrl_pub=orb_advertise(ORB_ID(alt_ctrl),&alt_ctrl_control);
			}	*/

			if (_att_sp_pub != nullptr) {
				//caculate_thrust(&_att_sp.thrust);  //IFFPC
				//alt_ctrl_s.thrust=_att_sp.thrust;
				//orb_publish(ORB_ID(alt_ctrl),&_alt_ctrl_pub, &alt_ctrl_s);
				//iffpc
				//_att_sp.thrust=_manual.z;
				//_att_sp.thrust=0.8f;
				//iffpc
			//	if( _manual.z<=0.1f )
		///	{_att_sp.thrust=0;}
				//_att_sp.yaw_body=0.2f;
			//	if (_att_sp_pub != nullptr) {
								//caculate_thrust(&_att_sp.thrust);
								//alt_ctrl_s.thrust=_att_sp.thrust;
								//orb_publish(ORB_ID(alt_ctrl),&_alt_ctrl_pub, &alt_ctrl_s);
								//_att_sp.thrust=0.4f;
								//iffpc
								//if(take_off)
									//{
									//_att_sp.roll_body=0.0f;
									//_att_sp.pitch_body=0.0f;
									//_att_sp.yaw_body=0.5f;
									//iffpc
									 // yx=_yaw;
									//}
								//iffpc
			//	warnx("flag_armed=%d",_control_mode.flag_armed);
			//	warnx("flag_external_manual_override_ok=%d",_control_mode.flag_external_manual_override_ok);
			//	warnx("flag_system_hil_enabled=%d",_control_mode.flag_system_hil_enabled);
			//	warnx("flag_control_manual_enabled=%d",_control_mode.flag_control_manual_enabled);
			//	warnx("flag_control_auto_enabled=%d",_control_mode.flag_control_auto_enabled);
			//	warnx("flag_control_offboard_enabled=%d",_control_mode.flag_control_offboard_enabled);
			//	warnx("flag_control_rates_enabled=%d",_control_mode.flag_control_rates_enabled);
			//	warnx("attitude_enabled=%d",_control_mode.flag_control_rattitude_enabled);
			//	warnx("flag_control_force_enabled=%d",_control_mode.flag_control_force_enabled);
			//	warnx("flag_control_acceleration_enabled=%d",_control_mode.flag_control_acceleration_enabled);
			//	warnx("flag_control_velocity_enabled=%d",_control_mode.flag_control_velocity_enabled);
			//	warnx("flag_control_position_enabled=%d",_control_mode.flag_control_position_enabled);
			//	warnx("flag_control_altitude_enabled=%d",_control_mode.flag_control_altitude_enabled);
			//	warnx("flag_control_climb_rate_enabled=%d",_control_mode.flag_control_climb_rate_enabled);
			//	warnx("flag_control_termination_enabled=%d",_control_mode.flag_control_termination_enabled);
                 //iffpc
				//warnx("flag_armed=%d",_control_mode.flag_armed);
				orb_publish(_attitude_setpoint_id, _att_sp_pub, &_att_sp);
				//iffpc
			//	orb_copy(ORB_ID(vehicle_attitude_setpoint), _att_sp_sub, &_att_sp1);
			//	warnx("roll =%??????3f",(double)_att_sp.roll_body);
			//	warnx("pitch =%??????3f",(double)_att_sp.pitch_body);
			//	warnx("yaw =%??????3f",(double)_att_sp.yaw_body);
				//printf(" 2520 true  thrust =%.3f",(double)_att_sp.thrust);
				//printf(" 2521 true  pitch =%.3f",(double)_att_sp.pitch_body);
			//	printf(" 2522 true  roll =%.3f",(double)_att_sp.roll_body);
				//iffpc
			} else if (_attitude_setpoint_id) {
				_att_sp_pub = orb_advertise(_attitude_setpoint_id, &_att_sp);
			}
		}

		/* reset altitude controller integral (hovering throttle) to manual throttle after manual throttle control */
		reset_int_z_manual = _control_mode.flag_armed && _control_mode.flag_control_manual_enabled
				     && !_control_mode.flag_control_climb_rate_enabled;
	}

	mavlink_log_info(&_mavlink_log_pub, "[mpc] stopped");

	_control_task = -1;
}

int
MulticopterPositionControl::start()
{
	ASSERT(_control_task == -1);

	/* start the task */
	_control_task = px4_task_spawn_cmd("mc_pos_control",
					   SCHED_DEFAULT,
					   SCHED_PRIORITY_MAX - 5,
					   1900,
					   (px4_main_t)&MulticopterPositionControl::task_main_trampoline,
					   nullptr);

	if (_control_task < 0) {
		warn("task start failed");
		return -errno;
	}

	return OK;
}

int mc_pos_control_main(int argc, char *argv[])
{
	if (argc < 2) {
		warnx("usage: mc_pos_control {start|stop|status}");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (pos_control::g_control != nullptr) {
			warnx("already running");
			return 1;
		}

		pos_control::g_control = new MulticopterPositionControl;

		if (pos_control::g_control == nullptr) {
			warnx("alloc failed");
			return 1;
		}

		if (OK != pos_control::g_control->start()) {
			delete pos_control::g_control;
			pos_control::g_control = nullptr;
			warnx("start failed");
			return 1;
		}

		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		if (pos_control::g_control == nullptr) {
			warnx("not running");
			return 1;
		}

		delete pos_control::g_control;
		pos_control::g_control = nullptr;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (pos_control::g_control) {
			warnx("running");

			//int error_counter=0;
			int hp_sub=orb_subscribe(ORB_ID(pos_helper));
			struct pos_helper_s hp;

			int position_sub=orb_subscribe(ORB_ID(vehicle_local_position));
			struct vehicle_local_position_s position;

			int v_att_sp_sub = orb_subscribe(ORB_ID(vehicle_attitude_setpoint));
			struct vehicle_attitude_setpoint_s v_att_sp;

			int manual_sub = orb_subscribe(ORB_ID(manual_control_setpoint));
			struct manual_control_setpoint_s manual1;

			int v_rates_sp_sub = orb_subscribe(ORB_ID(vehicle_rates_setpoint));
			struct vehicle_rates_setpoint_s v_rates_sp;

			int v_ukf_localization_sub = orb_subscribe(ORB_ID(ukf_localization));
			struct ukf_localization_s v_ukf_localization;

			//int v_vehicle_local_position_setpoint_sub = orb_subscribe(ORB_ID(vehicle_local_position_setpoint));
			//struct vehicle_local_position_setpoint_s v_vehicle_local_position_setpoint_sp;

			//px4_pollfd_struct_t fds[] = { ukf_localization
			//		{ .fd=manual_sub,	.events=POLLIN },
			//};

			while(1)
			{
				/*int poll_ret = px4_poll(fds,1,200);

				if(poll_ret==0)
				{
					PX4_ERR("Got no data within a second");
				}
				else if(poll_ret<0)
				{
					if (error_counter < 10 || error_counter % 50 == 0)
					{
						PX4_ERR("ERROR return value from poll(): %d", poll_ret);
					}
					error_counter++;
				}
				else
				{
					if(fds[0].revents & POLLIN)
					{	*/
				        orb_copy(ORB_ID(pos_helper), hp_sub, &hp);
						orb_copy(ORB_ID(manual_control_setpoint),manual_sub,&manual1);
						orb_copy(ORB_ID(vehicle_local_position),position_sub,&position);
						orb_copy(ORB_ID(vehicle_attitude_setpoint), v_att_sp_sub, &v_att_sp);
						orb_copy(ORB_ID(vehicle_rates_setpoint), v_rates_sp_sub, &v_rates_sp);
						orb_copy(ORB_ID(ukf_localization), v_ukf_localization_sub, &v_ukf_localization);
						//orb_copy(ORB_ID(vehicle_local_position_setpoint), v_vehicle_local_position_setpoint_sub, &v_vehicle_local_position_setpoint_sp);

						warnx("PID: [P=%.3f,I=%.3f,D=%.3f]", (double)P11,(double)I11,(double)D11);
						warnx("PID: [er=%.3f,erp=%.3f,ercha=%.3f]", (double)er,(double)erp,(double)(er-erp));
						//warnx("Debug:time=%lld count=%d",time_cha,count1);
						printf("hpx=%.3f\n",(double)hp.mx);
						printf("take_off=%.3f\n",(double)take_off);
						//warnx("thrust=%.2f alt_sp=%.2f alt_now=%.2f",(double)thrust1,(double)alt_sp1,(double)alt_now1);
						warnx("Alt:ref_height=%.2f,res=%.2f",(double)ref_height1,(double)height1);
						warnx("Manual:x=%.2f,y=%.2f,z=%.2f",(double)manual1.x,(double)manual1.y,(double)manual1.z);
						//warnx("_att_sp.rollbody=%.2f,_att_sp.pitchbody=%.2f",(double)v_att_sp.roll_body,(double)v_att_sp.pitch_body);
						warnx("Position:vx=%.3f,vy=%.3f,vz=%.3f",(double)position.vx,(double)position.vy,(double)position.vz);
						//warnx("Position:vxsp=%.3f,vysp=%.3f,vzsp=%.3f",(double)v_vehicle_local_position_setpoint_sp.vx,(double)v_vehicle_local_position_setpoint_sp.vy,(double)v_vehicle_local_position_setpoint_sp.vz);
						warnx("yaw_now:_yaw=%.3f,target =%.3f",(double)position.yaw,(double)v_att_sp.yaw_body);
						warnx("Position:x=%.3f,y=%.3f,z=%.3f",(double)position.x,(double)position.y,(double)position.z);
						//warnx("Position:xsp=%.3f,ysp=%.3f,zsp=%.3f",(double)v_vehicle_local_position_setpoint_sp.x,(double)v_vehicle_local_position_setpoint_sp.y,(double)v_vehicle_local_position_setpoint_sp.z);
						warnx("yaw_rate:=%.2f",(double)v_rates_sp.yaw);
						warnx("att_sp:thrust=%.3f ",(double)v_att_sp.thrust);
						warnx("Position:vx_valid=%.3f,x_valid=%.3f",(double)position.v_xy_valid,(double)position.xy_valid);
						//warnx("Position:U_x=%.3f,U_y=%.3f",(double)v_ukf_localization.x,(double)v_ukf_localization.y);
						//printf("x_sp:x=%.3f ",(double)v_vehicle_local_position_setpoint_sp.x);
						//printf("flag_auto=%d\n",flag_auto);
				//
				//}
				warnx("============press CTRL+C to abort============");

				char c;
				struct pollfd fds;
				int ret;
				fds.fd=0;
				fds.events=POLLIN;
				ret=poll(&fds,1,0);
				if(ret>0)
				{
					read(0,&c,1);
					if(c==0x03||c==0x63||c=='q')
					{
						warnx("User abort\n");
						break;
					}
				}
				usleep(800000);		//500ms
			}

			return 0;

		} else {
			warnx("not running");
			return 1;
		}
	}

	warnx("unrecognized command");
	return 1;
}
