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

__EXPORT int my_test_app_main(int argc, char *argv[]);

int my_test_app_main(int argc, char *argv[])
{
	//=======================test 3================================
	//math::Matrix<3, 3>
	double A[3][3]={{4.0,1.0,1.0},{1.0,2.0,3.0},{1.0,3.0,6.0}};
	double L[3][3];
	for(int m=0;m<3;m++)
	{
		for(int n=0;n<m;n++)
		{
			L[m][n]=A[m][n];
			for(int k=0;k<n;k++)
			{
				L[m][n]-=L[m][k]*L[n][k];
			}
			if(m==n)
				L[m][n]=sqrt(L[m][n]);
			else
				L[m][n]=L[m][n]/L[n][n];
		}
		for(int x=m+1;x<3;x++)
			L[m][x]=0.0;

	}
	for(int i=0;i<3;i++)
	{
		for(int j=0;j<3;j++)
		{
			PX4_INFO("%f",L[i][j]);
		}
	}



	return 0;




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



}
