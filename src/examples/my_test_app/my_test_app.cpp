#include <px4_config.h>
#include <px4_tasks.h>
#include <px4_posix.h>
#include <unistd.h>
#include <stdio.h>
#include <poll.h>
#include <string.h>

#include <math.h>
#include <mathlib/mathlib.h>

#include <drivers/drv_hrt.h>

#include <uORB/uORB.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/rc_channels.h>
#include <uORB/topics/control_state.h>
#include <uORB/topics/ukf_localization.h>

#include <drivers/drv_hrt.h>

#define PI 3.1415926
int time_interval_IMU;
int time_interval_dist;
int time_temp_IMU=0;
int time_temp_dist=0;

extern "C" int my_test_app_main(int argc, char *argv[]);

int my_test_app_main(int argc, char *argv[])
{
	//====================test 5 测试IMU的更新频率=====================================
//		int _ctrl_state_sub=orb_subscribe(ORB_ID(control_state));
//		int _distance_sensor_sub=orb_subscribe(ORB_ID(distance_sensor));
//	bool updated=true;
//	while(1)
//	{
//	orb_check(_distance_sensor_sub, &updated);
//	if (updated) {
//		time_interval_dist=hrt_absolute_time()-time_temp_dist;
//		time_temp_dist=hrt_absolute_time();
////		orb_copy(ORB_ID(distance_sensor), _distance_sensor_sub, &distance);
//		PX4_INFO("time interval of distance sensor is = %d",(int)time_interval_dist);
//		usleep(80000);
//		}
//
//	}
//	orb_check(_distance_sensor_sub, &updated);
//	if (updated) {
//		time_interval_dist=hrt_absolute_time()-time_temp_dist;
//	}
	//===================test 4读取加速度计和陀螺仪的数据================
//	struct control_state_s inertial;
//	struct distance_sensor_s distance;

//		int _ctrl_state_sub=orb_subscribe(ORB_ID(control_state));
	//	int _distance_sensor_sub=orb_subscribe(ORB_ID(distance_sensor));
		int ukf_state_sub=orb_subscribe(ORB_ID(ukf_localization));
		uint64_t time_interval;
		uint64_t time_last=0;
		uint64_t time;
	while(1)
	{
		orb_stat(ukf_state_sub, &time);
		time_interval=time-time_last;
		time_last=time;
//	math::Quaternion q_att(inertial.q[0], inertial.q[1], inertial.q[2],inertial.q[3]);
//	math::Matrix<3, 3> _R;
//	_R = q_att.to_dcm();
//	math::Vector<3> euler_angles;
//	euler_angles = _R.to_euler();
	PX4_INFO("time interval of control state is:%d",(int)time_interval);
//	PX4_INFO("degree:cos90=%.4f  \tsin90=%.4f",(double)cos(90),(double)sin(90));
//	PX4_INFO("rad:cos(pi/2)=%.4f  \tsin(pi/2)=%.4f",(double)cos(PI/2),(double)sin(PI/2));
//	//	PX4_INFO("distacen1 = %f\tdistacen2= %f",(double)distance.distance[0],(double)distance.distance[1]);
////	PX4_INFO("distacen3 = %f\tdistacen4= %f",(double)distance.distance[2],(double)distance.distance[3]);
//	PX4_INFO("x_acc = %.4f\ty_acc= %.4f\tz_acc=%.4f",(double)inertial.x_acc,(double)inertial.y_acc,(double)inertial.z_acc);
//	PX4_INFO("roll_rate = %.4f\tpitch_rate= %.4f\tyaw_rate=%.4f",(double)inertial.roll_rate,(double)inertial.pitch_rate,(double)inertial.yaw_rate);
//	PX4_INFO("yaw_by_gyro = %.4f",(double)euler_angles(2));
	PX4_INFO("=========================================");
	usleep(80000);
	char c;
	struct pollfd fds1;
	int ret;
	fds1.fd = 0;
	fds1.events = POLLIN;
	ret = poll(&fds1, 1, 0);
	if (ret > 0)
	{
		read(0, &c, 1);
		if (c == 0x03 || c == 0x63 || c == 'q')
		{
			warnx("User abort\n");
			break;
		}
	}
		}
//	//=====================test 4 end==============================================
//	//=======================test 3测试cholesky分解的正确性================================
//	//math::Matrix<3, 3>
//	double A[3][3]={{4.0,1.0,1.0},{1.0,2.0,3.0},{1.0,3.0,6.0}};
//	double L[3][3];
//	for(int m=0;m<3;m++)
//	{
//		for(int n=0;n<m;n++)
//		{
//			L[m][n]=A[m][n];
//			for(int k=0;k<n;k++)
//			{
//				L[m][n]-=L[m][k]*L[n][k];
//			}
//			if(m==n)
//				L[m][n]=sqrt(L[m][n]);
//			else
//				L[m][n]=L[m][n]/L[n][n];
//		}
//		for(int x=m+1;x<3;x++)
//			L[m][x]=0.0;
//
//	}
//	for(int i=0;i<3;i++)
//	{
//		for(int j=0;j<3;j++)
//		{
//			PX4_INFO("%f",L[i][j]);
//		}
//	}
//	//========================test 3 end==================================

	//=========================test 2 =============================//
//	int manual_sub=orb_subscribe(ORB_ID(manual_control_setpoint));
//	int distance_sub=orb_subscribe(ORB_ID(distance_sensor));
//	int _rc_channel_sub =orb_subscribe(ORB_ID(rc_channels));
//
//	struct manual_control_setpoint_s manual;
//	memset(&manual, 0, sizeof(manual));
//
//	struct distance_sensor_s distance;
//	memset(&distance, 0, sizeof(distance));
//
//	struct rc_channels_s my_rc_channel;
//	memset(&my_rc_channel, 0, sizeof(my_rc_channel));
//
//	//bool update1,update2,update3;
//
//	//orb_check(manual_sub,&update1);
//	//if (update1)
//	//{
//		orb_copy(ORB_ID(manual_control_setpoint),manual_sub,&manual);
//		PX4_INFO("Manual:x=%.2f,y=%.2f,z=%.2f",(double)manual.x,(double)manual.y,(double)manual.z);
//	//}
//	//else
//		//PX4_INFO("error1");
//	//orb_check(distance_sub,&update2);
//	//if (update2)
//
//	while(1)
//	{
//		orb_copy(ORB_ID(distance_sensor),distance_sub,&distance);
//		PX4_INFO("distance_sensor.max_distance = %f\tcurrent_distance= %f",(double)distance.max_distance,(double)distance.current_distance);
//	int a=hrt_absolute_time();
//		PX4_INFO("the absolute time is:%012d",(int)a);
//	usleep(800000);
//	char c;
//	struct pollfd fds1;
//	int ret;
//	fds1.fd = 0;
//	fds1.events = POLLIN;
//	ret = poll(&fds1, 1, 0);
//	if (ret > 0)
//	{
//		read(0, &c, 1);
//		if (c == 0x03 || c == 0x63 || c == 'q')
//		{
//			warnx("User abort\n");
//			break;
//		}
//	}
//	}
//	//}
//	//else
//		//PX4_INFO("error2");
//	//orb_check(_rc_channel_sub,&update3);
//	//if (update3)
//	//{
//		orb_copy(ORB_ID(rc_channels),_rc_channel_sub,&my_rc_channel);
//		PX4_INFO("channels[0]=%f		channels[1]=%f		channels[3]=%f",(double)my_rc_channel.channels[0],(double)my_rc_channel.channels[1],(double)my_rc_channel.channels[3]);
	//==============================test 2 end =============================//
		//}
	//else
		//PX4_INFO("error3");
//====================test 1=====================//
	/*int ret1=orb_exists(ORB_ID(manual_control_setpoint),0);
	if (ret1==OK)
	PX4_INFO("topic1 exist!!");
	else
	PX4_INFO("topic1 dont't exist!!");

	int ret2=orb_exists(ORB_ID(distance_sensor),0);
	if (ret2==OK)
	PX4_INFO("topic2 exist!!");
	else
	PX4_INFO("topic2 dont't exist!!");

	int ret3=orb_exists(ORB_ID(rc_channels),0);
	if (ret3==OK)
	PX4_INFO("topic3 exist!!");
	else
	PX4_INFO("topic3 dont't exist!!");*/
//=================test 1 end============================

	return 0;

}
