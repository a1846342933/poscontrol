
 /****************************************************************************
 *
 *   Copyright (c) 2012-2015 PX4 Development Team. All rights reserved.
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
 * @file px4_daemon_app.c
 * daemon application example for PX4 autopilot
 *
 * @author Example User <mail@example.com>//0102
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>
#include <poll.h>

#include <px4_config.h>
#include <nuttx/sched.h>

#include <systemlib/systemlib.h>
#include <systemlib/err.h>

#include <uORB/uORB.h>
#include <uORB/topics/pos_helper.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/sonar_distance.h>
#include <uORB/topics/ukf_localization.h>
#include <uORB/topics/vehicle_status.h>
//
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/sensor_accel.h>
#include <uORB/topics/vehicle_local_position.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_pwm_output.h>

static bool thread_should_exit = false;		/**< daemon exit flag */
static bool thread_running = false;		/**< daemon status flag */
static int daemon_task;				/**< Handle of daemon task / thread */
//int h_ctrl_state_sub;
int h_sonar_sub;
int h_ukf_status_sub;
int h_vehicle_local_position_sub;

int h_x;
float h_height;
int h_vehicle_status_sub;
hrt_abstime t, t_prev;
float dt;
int h_h[10];
//float xacc=0,yacc=0;
//float x_est[2] = { 0.0f, 0.0f };	// pos, vel
//float y_est[2] = { 0.0f, 0.0f };	// pos, vel


orb_advert_t	pos_helper_pub;
    //int ix;
struct pos_helper_s manual;
//struct control_state_s	h_ctrl_state;
struct sonar_distance_s 	h_sonar;
struct ukf_localization_s h_ukf_status;
struct vehicle_local_position_s h_vehicle_local_position;
struct vehicle_status_s h_vehicle_status;
/**
 * daemon management function.
 */
__EXPORT int mc_pos_helper_main(int argc, char *argv[]);

/**
 * Mainloop of daemon.
 */
int mc_pos_helper_thread_main(int argc, char *argv[]);


/**
 * Print the correct usage.
 */
static void usage(const char *reason);

void publish_manual_control_setpoint(void);
//void h_inertial_filter_predict(float dt1, float x[2], float acc);

static void
usage(const char *reason)
{
	if (reason) {
		warnx("%s\n", reason);
	}

	warnx("usage: daemon {start|stop|status} [-p <additional params>]\n\n");
}

void publish_manual_control_setpoint(void)
{

     //   if((manual.mx<=manual_prev.mx+0.01f||manual.mx>=manual_prev.mx-0.01f)&&(manual.my<=manual_prev.my+0.01f||manual.my>=manual_prev.my-0.01f)&&(manual.mz<=manual_prev.mz+0.01f||manual.mz>=manual_prev.mz-0.01f))
      //  {
      // 	return;
      //  }
        /* lazily publish the position setpoint triplet only once available */
       if (pos_helper_pub!= NULL) {
        	    manual.timestamp = hrt_absolute_time();
                orb_publish(ORB_ID(pos_helper), pos_helper_pub, &manual);
                //warnx("pos mx =%.3f",(double)manual.mz);
        } else {
        	pos_helper_pub = orb_advertise(ORB_ID(pos_helper), &manual);
        }
}


//void h_inertial_filter_predict(float dt1, float x[2], float acc)
//{


//		x[0] += x[1] * dt1 + acc * dt1 * dt1 / 2.0f;
//		x[1] += acc * dt1;
//	}
/*
hrt_abstime t = hrt_absolute_time();
float dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
dt = fmaxf(fminf(0.02, dt), 0.0002);		// constrain dt from 0.2 to 20 ms
t_prev = t;

float x_est[2] = { 0.0f, 0.0f };	// pos, vel
float y_est[2] = { 0.0f, 0.0f };	// pos, vel

float acc[] = { 0.0f, 0.0f, 0.0f };	// N E D
float acc_bias[] = { 0.0f, 0.0f, 0.0f };	// body frame

 //inertial filter prediction for position
			inertial_filter_predict(dt, x_est, acc[0]);
			inertial_filter_predict(dt, y_est, acc[1]);
*/

/**
 * The daemon app only briefly exists to start
 * the background job. The stack size assigned in the
 * Makefile does only apply to this management task.
 *
 * The actual stack size should be set in the call
 * to task_create().
 */
int mc_pos_helper_main(int argc, char *argv[])
{
	if (argc < 2) {
		usage("missing command");
		return 1;
	}

	if (!strcmp(argv[1], "start")) {

		if (thread_running) {

			//warnx("daemon already running\n");

			/* this is not an error */
			return 0;
		}

		thread_should_exit = false;
		daemon_task = px4_task_spawn_cmd("mc_pos_helper",
						 SCHED_DEFAULT,
						 SCHED_PRIORITY_DEFAULT,
						 2000,
                          mc_pos_helper_thread_main,
						 (argv) ? (char *const *)&argv[2] : (char *const *)NULL);
		return 0;
	}

	if (!strcmp(argv[1], "stop")) {
		thread_should_exit = true;
		return 0;
	}

	if (!strcmp(argv[1], "status")) {
		if (thread_running) {
			warnx("\trunning\n");


					//	int manual_sub=orb_subscribe(ORB_ID(manual_control_setpoint));
					//	struct manual_control_setpoint_s manual;
			         //   int h_sensor_sub=orb_subscribe(ORB_ID(sensor_combined));
			         //   struct sensor_combined_s h_sensor;
					//	int vv_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
					//	struct control_state_s vv_ctrl_state;
			       //     int h_sensor_sub1=orb_subscribe(ORB_ID(sensor_accel));
			      //      struct sensor_accel_s h_sensor1;
			           // sensor_accel
			            int local_positon_sub1=orb_subscribe(ORB_ID(vehicle_local_position));
			            struct vehicle_local_position_s vehicle_local_position1;
						while(1)
						{
							//up_pwm_servo_set( 0, 1800);
								//	orb_copy(ORB_ID(sensor_combined),h_sensor_sub,&h_sensor);
								//	orb_copy(ORB_ID(sensor_accel),h_sensor_sub1,&h_sensor1);
									//orb_copy(ORB_ID(control_state), vv_ctrl_state_sub, &vv_ctrl_state);
									orb_copy(ORB_ID(vehicle_local_position),local_positon_sub1,&vehicle_local_position1);
								//	warnx("1226 acc_x =%.3f",(double)h_sensor.accelerometer_m_s2[0]);
								//	warnx("1226 acc_y =%.3f",(double)h_sensor.accelerometer_m_s2[1]);
								//	warnx("1226 acc_y1 =%.3f",(double)h_sensor1.x);
									//warnx("1226 acc_x3 =%.3f",(double)xacc);
									//warnx("1226 acc_y3 =%.3f",(double)yacc);
									warnx("manual.mx =%.3f",(double)manual.mx);
									warnx("manual.mz =%.3f",(double)manual.mz);
									warnx("thread_should_exit=%.3f",(double)thread_should_exit);
									//warnx("yp=%.3f yv=%.3f",(double)y_est[0],(double)y_est[1]);
									warnx("1226 lvy =%.3f lpy =%.3f",(double)vehicle_local_position1.vy,(double)vehicle_local_position1.y);
									//warnx("slope =%.3f",(double)slope);
									//t = hrt_absolute_time()

		                           // dt
		                            //slope
									//warnx("ctrl_state.x_pos=%.2f,ctrl_state.y_pos=mz=%.2f",(double)vv_ctrl_state.x_pos,(double)vv_ctrl_state.y_pos);
							//	}
							//}
							warnx("============press CTRL+C to abort============");

							char h_c;
							struct pollfd fds;
							int ret;
							fds.fd=0;
							fds.events=POLLIN;
							ret=poll(&fds,1,0);
							if(ret>0)
							{
								read(0,&h_c,1);
								if(h_c==0x03||h_c==0x63||h_c=='q')
								{
									warnx("User abort\n");
									break;
								}
							}
							usleep(800000);		//500ms
						}

						return 0;

			/*
			 * struct sensor_combined_s h_sensor;
	           //memset(&sensor, 0, sizeof(sensor));
	         *
			 */
		} else {
			warnx("\tnot started\n");
		}

		return 0;
	}

	usage("unrecognized command");
	return 1;
}

int mc_pos_helper_thread_main(int argc, char *argv[])
{

	warnx("[daemon] starting\n");

	thread_running = true;

	//int pos_helper_sub = orb_subscribe(ORB_ID(pos_helper));
	memset(&manual, 0, sizeof(manual));
//	h_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
//	memset(&h_ctrl_state, 0, sizeof(h_ctrl_state));
	h_sonar_sub=orb_subscribe(ORB_ID(sonar_distance));
	memset(&h_sonar, 0, sizeof(h_sonar));
	h_ukf_status_sub=orb_subscribe(ORB_ID(ukf_localization));
	memset(&h_ukf_status, 0, sizeof(h_ukf_status));
	h_vehicle_local_position_sub=orb_subscribe(ORB_ID(vehicle_local_position));
	memset(&h_vehicle_local_position,0,sizeof(h_vehicle_local_position));
	h_vehicle_status_sub=orb_subscribe(ORB_ID(vehicle_status));
    memset(&h_vehicle_status, 0, sizeof(h_vehicle_status));
	//int h_sensor_sub2=orb_subscribe(ORB_ID(sensor_combined));
	//struct sensor_combined_s h_sensor2;

	//manual.timestamp = hrt_absolute_time();

	//orb_publish(ORB_ID(pos_helper), pos_helper_pub, &manual);
	//orb_copy(ORB_ID(pos_helper), pos_helper_sub, &manual);

	//orb_copy(ORB_ID(ukf_localization), h_ukf_status_sub, &h_ukf_status);
	//orb_copy(ORB_ID(control_state), h_ctrl_state_sub, &h_ctrl_state);
	manual.f1=true;
	manual.f2=false;
	manual.mx=0.0f;
	manual.mz=0.5f;
	manual.my=0.0f;
	//float slope=0.1f;

	t = hrt_absolute_time();
	t_prev = hrt_absolute_time();

	while (!thread_should_exit) {


		orb_copy(ORB_ID(vehicle_status), h_vehicle_status_sub, &h_vehicle_status);//wei jiesuo1 jiesuo2
		orb_copy(ORB_ID(vehicle_local_position),h_vehicle_local_position_sub,&h_vehicle_local_position);
		orb_copy(ORB_ID(sonar_distance), h_sonar_sub, &h_sonar);
		//if(h_vehicle_status.arming_state==2)
		//{
		 t = hrt_absolute_time();
		 dt = t_prev > 0 ? (t - t_prev) / 1000000.0f : 0.0f;
		dt = fmaxf(fminf(0.02, dt), 0.0002);		// constrain dt from 0.2 to 20 ms
		t_prev = t;
		//orb_copy(ORB_ID(sonar_distance), h_sonar_sub, &h_sonar);
					if(h_sonar.distance_filter>=0)
					{
					if(h_x<9)
					{
							h_h[h_x]=h_sonar.distance_filter*0.01f;
							++h_x;
					}
					else
					{
						h_x=1;
						h_h[0]=h_sonar.distance_filter*0.01f;
					}

					for(int i=0;i<10;i++)
					{
						h_height=h_height+h_h[i];
					}
					h_height=h_height/10;}


        //warnx("h_vehicle_local_position.v_xy_valid =%.3f",(double)h_vehicle_local_position.v_xy_valid);
		//warnx("h_vehicle_local_position.x =%.3f",(double)h_vehicle_local_position.x);
	    if((h_sonar.distance_filter*0.01f)>0.1f)
	    {
	    	manual.f2=true;
	    }
        if((h_vehicle_local_position.v_xy_valid==true)&&(h_vehicle_local_position.x<0.2f))
        {
       // manual.f1=false;
        if(manual.f2&&(h_vehicle_status.arming_state==2))
        {manual.my=0.0f;}

        }else
        {manual.mx=0.0f;}
		//if(manual.mz>=0.4f)
		//{
		//	manual.mz=0.4f;
		//	slope=-0.1f;
		//}
		//if(manual.mz<=0.01f)
	//	{
	//				manual.mz=0.015f;
	//				slope=0.1f;
	//	}

		publish_manual_control_setpoint();
		//}
		warnx("actuator !\n");
		for(int i=0;i<4;i++){

		up_pwm_servo_set( i, 50);
		//up_pwm_servo_set( 1, 100);
		//up_pwm_servo_set( 2, 50);
		//up_pwm_servo_set( 3, 80);}
		sleep(2);
		//if(i==3){i=0;}
		}
	}

	warnx("[daemon] exiting.\n");

	thread_running = false;

	return 0;
}



