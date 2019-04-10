/*
 * This program aims to estimate the location of indoor UAV under the prior map,
 four mutually perpendicular vl53l1x sensors and IMU data.
 * @author FXK
 */

#include <px4_posix.h>
#include <mathlib/mathlib.h>
#include <uORB/uORB.h>
#include <uORB/topics/control_state.h>
#include <drivers/drv_hrt.h>
#include <uORB/topics/ukf_localization.h>
#include <uORB/topics/distance_sensor.h>	//vl53l1x

static bool task_should_exit = false;
static bool task_running = false;
static int control_task;
float _yaw;
struct control_state_s inertial;
struct distance_sensor_s distance;
struct ukf_localization_s ukf_status;
int _ctrl_state_sub; 					//control state subscription
int _distance_sensor_sub;          		// four mb12xx sonar sensor data

//以下宏定义为室内定位算法需要用到的调节参数
#define PI							3.1415926
#define SONAR_NUMBER_SET 			4				//超声波数量设置
#define MAP_NUMBER_SET   			13				//先验地图维数，目前可选参数包括4/13/25
#define RAY_NUMBER_SET   			5

//先验地图
struct PRIOR_MAP {
	float x[MAP_NUMBER_SET] = { 0, 5, 5, 9, 9, 14, 14, 12, 12, 7, 7, 0, 0 };
	float y[MAP_NUMBER_SET] = { 0, 0, 2, 2, 0, 0, 4, 4, 10, 10, 5, 5, 0 };
};
PRIOR_MAP _map_test;

struct PRIOR_SONAR {
	float x[RAY_NUMBER_SET] = { 1.00, 3.50, 6.00, 1.00, 3.50 };
	float y[RAY_NUMBER_SET] = { -0.2, -0.1375, 0, 0.2, 0.1375 };
};
PRIOR_SONAR _sonar_model;

//墙面的角度
float Wall_Angle[MAP_NUMBER_SET - 1] = { 0, PI / 2, 0, PI / 2, 0, PI / 2, 0, PI
		/ 2, 0, PI / 2, 0, PI / 2 };

//带有坐标的点
struct POINTF {
	POINTF(float a, float b) :
			x(a), y(b) {
	}
	POINTF() {
	}

	float x;
	float y;
};
POINTF _map_start_point, // the map start point for calculate the intersection point
		_map_end_point, // the map end point for calculate the intersection point
		_sonar_start_point,                   // the sonar model start point
		_sonar_end_point,                       // the sonar model end point
		_sonar_end_point_rotation, // the sonar model end point after rotation
		_intersection_point; // the intersection point between the sonar model and the prior map

//函数声明
extern "C" __EXPORT int mc_localization_UKF_main(int argc, char *argv[]);
int task_main(int argc, char *argv[]);
// 使用提示函数
static void usage(const char *reason);
math::Matrix<5,5> cholesky(math::Matrix<5,5> A);
void calculateline(POINTF p1, POINTF p2, float &a, float &b, float &c);
bool Equal(float f1, float f2);
bool Bigger(const POINTF &p1, const POINTF &p2);
float Cross_product(const POINTF &p1, const POINTF &p2);
void Swap(float &f1, float &f2);
void Swap_struct(POINTF &p1, POINTF &p2);
float Intersection(POINTF p1, POINTF p2, POINTF p3, POINTF p4,
		POINTF &intersection_point) ;
void sonar_value_minimum(POINTF location, float yaw,
		math::Matrix<4, 1> &distance_theory_minimum);
void ctrlStateUpdate(bool& updated);
void distanceSensorUpdate(bool& updated);

math::Matrix<5,5> cholesky(math::Matrix<5,5> A)
{
	math::Matrix<5,5> L;
	for(int m=0;m<5;m++ )
	{
		for(int n=0;n<=m;n++)
		{
			L(m,n)=A(m,n);
			for(int k=0;k<n;k++)
			{
				L(m,n)-=L(m,k)*L(n,k);
			}
			if(m==n)
				L(m,n)=sqrt(L(m,n));
			else
				L(m,n)=L(m,n)/L(n,n);
		}
		for(int x=m+1;x<1;x++)
			L(m,x)=0.0;
	}
	return L;
	}

//求交点
//此函数用于计算直线表达式，已知两点p1和p2，得到直线ax+by+c
void calculateline(POINTF p1, POINTF p2, float &a, float &b, float &c) {
	a = p2.y - p1.y;
	b = p1.x - p2.x;
	c = p2.x * p1.y - p1.x * p2.y;
}

//判断两数f1和f2是否相等？
//用于跨立实验
bool Equal(float f1, float f2) {
	return (fabs(f1 - f2) < 1e-4);
}

//比较两点坐标大小，先比较x坐标，若相同则比较y坐标
//用于跨立实验
bool Bigger(const POINTF &p1, const POINTF &p2) {
	return (p1.x > p2.x || (Equal(p1.x, p2.x) && p1.y > p2.y));
}

//计算两向量外积
//用于跨立实验
float Cross_product(const POINTF &p1, const POINTF &p2) {
	return (p1.x * p2.y - p1.y * p2.x);
}

//把两数f1和f2进行交换
void Swap(float &f1, float &f2) {
	float temp;
	temp = f1;
	f1 = f2;
	f2 = temp;
}

//把两位置结构体进行交换
void Swap_struct(POINTF &p1, POINTF &p2) {
	Swap(p1.x, p2.x);
	Swap(p1.y, p2.y);
}

//判定两线段位置关系，并求出交点(如果存在)。返回值交于线上(2)，正交(1)，无交(0)
float Intersection(POINTF p1, POINTF p2, POINTF p3, POINTF p4,
		POINTF &intersection_point) {
	// 为方便运算，保证各线段的起点在前，终点在后。
	if (Bigger(p1, p2)) {
		Swap_struct(p1, p2);
	}
	if (Bigger(p3, p4)) {
		Swap_struct(p3, p4);
	}
	// 将线段按起点坐标排序。若线段1的起点较大，则将两线段交换
	if (Bigger(p1, p3)) {
		Swap_struct(p1, p3);
		Swap_struct(p2, p4);
	}
	//计算向量及其外积
	POINTF v1 = { p2.x - p1.x, p2.y - p1.y }, v2 = { p4.x - p3.x, p4.y - p3.y };
	float Corss = Cross_product(v1, v2);
	//先进行快速排斥试验
	//x坐标已有序，可直接比较。y坐标要先求两线段的最大和最小值
	float ymax1 = p1.y, ymin1 = p2.y, ymax2 = p3.y, ymin2 = p4.y;
	if (ymax1 < ymin1) {
		Swap(ymax1, ymin1);
	}
	if (ymax2 < ymin2) {
		Swap(ymax2, ymin2);
	}
	//如果以两线段为对角线的矩形不相交，则无交点
	if (p1.x > p4.x || p2.x < p3.x || ymax1 < ymin2 || ymin1 > ymax2) {
		return 0;
	}
	//下面进行跨立试验
	POINTF vs1 = { p1.x - p3.x, p1.y - p3.y }, vs2 =
			{ p2.x - p3.x, p2.y - p3.y };
	POINTF vt1 = { p3.x - p1.x, p3.y - p1.y }, vt2 =
			{ p4.x - p1.x, p4.y - p1.y };
	float s1v2, s2v2, t1v1, t2v1;
	//根据外积结果判定否交于线上
	s1v2 = Cross_product(vs1, v2);
	s2v2 = Cross_product(vs2, v2);
	t1v1 = Cross_product(vt1, v1);
	t2v1 = Cross_product(vt2, v1);
	//未交于线上，则判定是否相交
	if (s1v2 * s2v2 > 0 || t1v1 * t2v1 > 0) {
		return 0;
	}
	//以下为相交的情况,计算二阶行列式的两个常数项
	float ConA = p1.x * v1.y - p1.y * v1.x;
	float ConB = p3.x * v2.y - p3.y * v2.x;
	//计算行列式D1和D2的值，除以系数行列式的值，得到交点坐标
	intersection_point.x = (ConB * v1.x - ConA * v2.x) / Corss;
	intersection_point.y = (ConB * v1.y - ConA * v2.y) / Corss;
	//正交返回1
	return 1;
}

//计算超声波的理论观测值(由于UKF不用算jacobi矩阵故省去了计算墙面和超声波角度的部分)
void sonar_value_minimum(POINTF location, float yaw,
		math::Matrix<4, 1> &distance_theory_minimum)
{
	float _distance_theory[4];
	float para_a;
	float para_b;
	float para_c;
	float side_1;
	float side_2;
	float side;

	_distance_theory[0] = 7;
	_distance_theory[1] = 7;
	_distance_theory[2] = 7;
	_distance_theory[3] = 7;


	/*
	 sonar_map_label用于存储有交点的超声波射线和地图线段，首先全部赋值-1
	 当对应位置有交点时，则以交点编号和位置数据替换
	 如果没有交点，则为-1不变，作为后续判断的标志位
	 */

	float _distance_theory_temp = 0;
	float cos_value;
	float sin_value;
	float sin_yaw;
	float cos_yaw;
	sin_yaw = sin(yaw);
	cos_yaw = cos(yaw);

	_sonar_start_point.x = location.x; //Initialize the sonar model position
	_sonar_start_point.y = location.y; //Initialize the sonar model position
	//超声波认为是均匀分布
	for (int i = 0; i < SONAR_NUMBER_SET; i++) {
		cos_value = cos(90 * i * PI / 180);
		sin_value = sin(90 * i * PI / 180);

		for (int j = 0; j < RAY_NUMBER_SET; j++) {
			//旋转矩阵的设置
			/*第一个旋转矩阵：[cos(psi),-sin(psi);sin(psi),cos(psi)];
			 * */
			_sonar_end_point_rotation.x = (_sonar_model.x[j] * cos_yaw)
					- (_sonar_model.y[j] * sin_yaw);
			_sonar_end_point_rotation.y = (_sonar_model.x[j] * sin_yaw)
					+ (_sonar_model.y[j] * cos_yaw);

			//the ith sonar model position x'=x*cos(90*(i-1))-y*sin(90*(i-1))
			//第二个旋转矩阵：[cos(90*(i-1)),-sin((90*(i-1));sin((90*(i-1))),cos((90*(i-1)))]
			_sonar_end_point.x = location.x
					+ (_sonar_end_point_rotation.x * cos_value)
					- (_sonar_end_point_rotation.y * sin_value);
			//the ith sonar model position y'=x*sin(90*(i-1))+y*cos(90*(i-1))
			_sonar_end_point.y = location.y
					+ (_sonar_end_point_rotation.x * sin_value)
					+ (_sonar_end_point_rotation.y * cos_value);


//			angle = atan2((_sonar_end_point.y - _sonar_start_point.y),
//					(_sonar_end_point.x - _sonar_start_point.x));
			//由点_sonar_start_point和点_sonar_end_point计算得到穿过两点的直线para_a*x + para_b*y + para_c
			calculateline(_sonar_start_point, _sonar_end_point, para_a, para_b,
					para_c);

			_distance_theory_temp = 10;

			for (int k = 1; k < MAP_NUMBER_SET; k++) {


				_map_start_point.x = _map_test.x[k - 1]; //the start point of map line
				_map_start_point.y = _map_test.y[k - 1]; //the start point of map line
				_map_end_point.x = _map_test.x[k]; //the end point of map line
				_map_end_point.y = _map_test.y[k]; //the end point of map line

				_intersection_point.x = 1000;
				_intersection_point.y = 1000;

				//判断点_map_start_point和点_map_end_point与直线para_a*x + para_b*y + para_c的位置关系
				//当两点在直线同一侧时，side>0；当两点在直线两侧时，side<0；当至少一个点位于直线上时，side=0.
				side_1 = para_a * _map_start_point.x
						+ para_b * _map_start_point.y + para_c;
				side_2 = para_a * _map_end_point.x + para_b * _map_end_point.y
						+ para_c;
				side = side_1 * side_2;

				if (side > 0) {

				} else {
					// for calculate the intersection between the sonar model and the prior map
					Intersection(_sonar_start_point, _sonar_end_point,
							_map_start_point, _map_end_point,
							_intersection_point);

//					Intersection(_sonar_start_point, _sonar_end_point, angle,
//							_map_start_point, _map_end_point,
//							_intersection_point, _distance_theory_temp);
				}
				// for calculate the distance between the intersection point and the estimation point(the sonar model origin)
				if (((int) _intersection_point.x == 1000)
						&& ((int) _intersection_point.y == 1000)) {

				} else {
					_distance_theory_temp = sqrtf(
							(_intersection_point.x - location.x)
									* (_intersection_point.x - location.x)
									+ (_intersection_point.y - location.y)
											* (_intersection_point.y
													- location.y));

					if (_distance_theory[i] > _distance_theory_temp) {
						//当有交点时开始赋值相应数据
						//i表示第i个超声波
						_distance_theory[i] = _distance_theory_temp;
					}
				}
			}
			usleep(2);
		}
	}
	// store the minimum distance in distance_theory_minimum
	distance_theory_minimum(0, 0) = _distance_theory[0];
	distance_theory_minimum(1, 0) = _distance_theory[1];
	distance_theory_minimum(2, 0) = _distance_theory[2];
	distance_theory_minimum(3, 0) = _distance_theory[3];
}

//检测惯性传感器的数值有没有更新
void ctrlStateUpdate(bool& updated) {
	math::Matrix<3, 3> _R; 			// rotation matrix from attitude quaternions
	orb_check(_ctrl_state_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &inertial);
		math::Quaternion q_att(inertial.q[0], inertial.q[1], inertial.q[2],
				inertial.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		_yaw = euler_angles(2);		// the 3rd element of eulerian angle is yaw???????????????????????
//		ATT = euler_angles;
//		ATT = ATT*(180/PI);
	}
}

//检测距离传感器（超声波或者激光）的数值有没有更新
void distanceSensorUpdate(bool& updated) {
	orb_check(_distance_sensor_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(distance_sensor), _distance_sensor_sub, &distance);
	}
}

//=====================跨立实验函数完=====================

//====================下面是运行的主循环程序========================
//=========taks_main:UKF的主体程序========================
int task_main(int argc, char *argv[])
{
	usleep(1000);
	warnx("mc_localization_UKF is successful!\n");
	//线程启动标志
	task_running = true;
	warnx("mc_localization_UKF start successfully\n");

	//==================Load the prior map，加载地图模型==================================
//	//目前可选维数：4,13,25
//		if (MAP_NUMBER_SET == 4) {
//			//4维的地图
//			auto map_data_boundary_x = std::initializer_list<float>(
//					{ 0.00, 6, 6, 0 });
//			std::copy(map_data_boundary_x.begin(), map_data_boundary_x.end(),
//					_map_test.x);
//			auto map_data_boundary_y = std::initializer_list<float>( { 0.00, 0.00,
//					7.5, 7.5 });
//			std::copy(map_data_boundary_y.begin(), map_data_boundary_y.end(),
//					_map_test.y);
//		}
//		if (MAP_NUMBER_SET == 13) {
//			//13维的地图
//			auto map_data_boundary_x = std::initializer_list<float>( { 0, 6.30,
//					6.30, 0, 0, -1.95, -1.95, -11, -11, -1.95, -1.95, 0, 0 });
//			std::copy(map_data_boundary_x.begin(), map_data_boundary_x.end(),
//					_map_test.x);
//
//			auto map_data_boundary_y = std::initializer_list<float>( { 0, 0, 7.26,
//					7.26, 14.26, 14.26, 5.23, 5.23, 2.03, 2.03, -7.00, -7.00, 0 });
//			std::copy(map_data_boundary_y.begin(), map_data_boundary_y.end(),
//					_map_test.y);
//		}
//		if (MAP_NUMBER_SET == 25) {
//			//25维的地图
//			auto map_data_boundary_x = std::initializer_list<float>( { 0, 6.30,
//					6.30, 0, 0, -1.95, -1.95, -11, -11, -9.75, -9.75, -8.60, -8.60,
//					-7.10, -7.10, -5.95, -5.95, -4.45, -4.45, -3.30, -3.30, -1.95,
//					-1.95, 0, 0 });
//			std::copy(map_data_boundary_x.begin(), map_data_boundary_x.end(),
//					_map_test.x);
//
//			auto map_data_boundary_y = std::initializer_list<float>( { 0, 0, 7.26,
//					7.26, 14.26, 14.26, 5.23, 5.23, 2.03, 2.03, 1.48, 1.48, 2.03,
//					2.03, 1.48, 1.48, 2.03, 2.03, 1.48, 1.48, 2.03, 2.03, -7.00,
//					-7.00, 0 });
//			std::copy(map_data_boundary_y.begin(), map_data_boundary_y.end(),
//					_map_test.y);
//		}
//		//=======================加载地图模型完====================
//		//========================Load the srf01 sonar model，加载超声波射线模型std命名空间？？？=============================
//		//可选根数：1,3,5,7,21
//		if (RAY_NUMBER_SET == 1) {
//		//1根射线模型
//		auto sonar_data_x = std::initializer_list<float>( { 6.00 });
//		std::copy(sonar_data_x.begin(), sonar_data_x.end(), _sonar_model.x);
//		auto sonar_data_y = std::initializer_list<float>( { 0.00 });
//		std::copy(sonar_data_y.begin(), sonar_data_y.end(), _sonar_model.y);
//	}
//
//	if (RAY_NUMBER_SET == 3) {
//		//3根射线模型
//		auto sonar_data_x = std::initializer_list<float>( { 1.00, 6.00, 1.00 });
//		std::copy(sonar_data_x.begin(), sonar_data_x.end(), _sonar_model.x);
//		auto sonar_data_y = std::initializer_list<float>( { -0.2, 0.00, 0.2 });
//		std::copy(sonar_data_y.begin(), sonar_data_y.end(), _sonar_model.y);
//	}
//
//	if (RAY_NUMBER_SET == 5) {
//		//5根射线模型
//		auto sonar_data_x = std::initializer_list<float>( { 1.00, 3.50, 6.00,
//				1.00, 3.50 });
//		std::copy(sonar_data_x.begin(), sonar_data_x.end(), _sonar_model.x);
//		auto sonar_data_y = std::initializer_list<float>( { -0.2, -0.1375, 0,
//				0.2, 0.1375 });
//		std::copy(sonar_data_y.begin(), sonar_data_y.end(), _sonar_model.y);
//	}
//
//	if (RAY_NUMBER_SET == 7) {
//		//7根射线模型
//		auto sonar_data_x = std::initializer_list<float>( { 1.00, 2.00, 3.50,
//				6.00, 1.00, 2.00, 3.50 });
//		std::copy(sonar_data_x.begin(), sonar_data_x.end(), _sonar_model.x);
//		auto sonar_data_y = std::initializer_list<float>( { -0.2, -0.135,
//				-0.1375, 0, 0.2, 0.135, 0.1375 });
//		std::copy(sonar_data_y.begin(), sonar_data_y.end(), _sonar_model.y);
//	}
//
//	if (RAY_NUMBER_SET == 21) {
//		//21根射线模型
//		auto sonar_data_x = std::initializer_list<float>( { 0.60, 1.00, 1.50,
//				2.00, 2.50, 3.00, 3.50, 4.00, 4.50, 5.00, 6.00, 0.60, 1.00,
//				1.50, 2.00, 2.50, 3.00, 3.50, 4.00, 4.50, 5.00 });
//		std::copy(sonar_data_x.begin(), sonar_data_x.end(), _sonar_model.x);
//
//		auto sonar_data_y = std::initializer_list<float>( { -0.015, -0.2, -0.19,
//				-0.135, -0.15, -0.1575, -0.1375, -0.03, -0.04, -0.005, 0, 0.015,
//				0.2, 0.19, 0.135, 0.15, 0.1575, 0.1375, 0.03, 0.04, 0.005 });
//		std::copy(sonar_data_y.begin(), sonar_data_y.end(), _sonar_model.y);
//	}
	//=======================加载超声波模型完====================================
	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
	orb_advert_t ukf_localization_pub = orb_advertise(ORB_ID(ukf_localization),
			&ukf_status);


	//循环开始
	while (!task_should_exit)
	{
		//==================UKF第一步，初始化，设定各种初始状态===================================
		//1、定义需要用到的各种矩阵
		math::Matrix<5,1> X_hat,X_pred;//状态量（x,vx,y,vy,psi）X_hat为X(k+1|k+1)，X_pred为P(k+1|k)
		math::Matrix<5,11> X_hat_sample,X_pred_sample1,X_pred_sample2;//状态量采样点,分别为X(k|k),X(k+1|k)的采样点
		math::Matrix<4,1> Y_pred;//观测量(四个超声波读数值)
		math::Matrix<3,1> u;//输入，对应机体轴角速度和角加速度
		math::Matrix<5,5> P_hat,P_pred,Q;//误差协方差阵P(k|k),P(k+1|k)
		math::Matrix<5,4> Pxz_pred,K;//卡尔曼增益
		math::Matrix<4,4> Pzz_pred,R;
		math::Matrix<2,1> Wm1,Wc1,Wm2,Wc2;//采样点权值
		math::Matrix<4, 1> sonarvalue_theory;
		math::Matrix<4,11> sonarvalue_theory_sample;
		//2、定义需要用到的各种变量
		float T=0.008;
		int n=5,beta=2;
		float alpha1=1,alpha2=0.1;
		float kappa1=3-n, kappa2=3-n;
		float lambda1=(alpha1*alpha1)*(n+kappa1)-n;
		float lambda2=(alpha2*alpha2)*(n+kappa2)-n;
		//3、对变量进行初始化
		Q.zero();//过程噪声初始化
		Q(0, 0) = 1.0f; Q(1, 1) = 0.2f; Q(2, 2) = 1.0f; Q(3, 3) = 0.2f; Q(4, 4) = 0.1f;
		R.identity();//观测噪声初始化
		R = R * 0.007f;
		P_hat(0,0)=1.0f;P_hat(1,1)=0.1f;P_hat(2,2)=1.0f;P_hat(3,3)=0.1f;P_hat(4,4)=0.01f;
		Wm1(0,0)=lambda1/(n+lambda1);
		Wm1(0,1)=1/(n+lambda1);
		Wm2(0,0)=lambda1/(n+lambda1)+1-alpha1*alpha1+beta;
		Wm2(0,1)=1/(n+lambda1);
		usleep(5000);
		warnx("mc_localization_UKF start successfully\n");
		//==================UKF第二步，状态更新==================================
		bool updated = true;
		ctrlStateUpdate(updated);
		if (updated) //IMU数据有更新
		{
			u(0, 0) = inertial.x_acc * (float) cos(_yaw)
					+ inertial.y_acc * (float) sin(_yaw);
			u(1, 0) = inertial.x_acc * (float) sin(_yaw)
					- inertial.y_acc * (float) cos(_yaw);
			//虽然是机体系下的z轴角加速度，但是假设前提是飞机在二维平面运动，机体滚转和偏航角速度均为0
			//目前暂时把陀螺仪z轴角速度认为是偏航角速度
			u(2, 0) = inertial.yaw_rate;
			distanceSensorUpdate(updated);
			//1、超声波（激光）数据有更新
			for(int i=0;i<5;i++)//第一列
				X_hat_sample(i,0)=X_hat(i,0);
			math::Matrix<5,5> cho;
			cholesky(P_hat*(n+lambda1));
			cho=cho.transposed();
			//选取采样点
			for(int j=1;j<6;j++)
			{
				for(int i=0;i<5;i++)
					X_hat_sample(i,j)=X_hat(i,0)+cho(i,j);
			}
			for(int j=6;j<11;j++)
			{
				for(int i=0;i<5;i++)
					X_hat_sample(i,j)=X_hat(i,0)-cho(i,j-5);
			}
			X_pred.zero();
			P_pred.zero();
			//计算各个采样点对应的X(k+1|k)
			for(int j=0;j<11;j++)
			{
				X_pred_sample1(0,j)=X_hat_sample(0,j)+X_hat_sample(1,j)*T+0.5f*T*T*(u(0,0)*(float)cos(X_hat_sample(4,j))+u(1,0)*(float)sin(X_hat_sample(4,j)));
				X_pred_sample1(1,j)=X_hat_sample(1,j)+T*(u(0,0)*(float)cos(X_hat_sample(4,j))+u(1,0)*(float)sin(X_hat_sample(4,j)));
				X_pred_sample1(2,j)=X_hat_sample(2,j)+X_hat_sample(3,j)*T+0.5f*T*T*(u(0,0)*(float)sin(X_hat_sample(4,j))-u(1,0)*(float)cos(X_hat_sample(4,j)));
				X_pred_sample1(3,j)=X_hat_sample(3,j)+T*(u(0,0)*(float)sin(X_hat_sample(4,j))-u(1,0)*(float)cos(X_hat_sample(4,j)));
				X_pred_sample1(4,j)=X_hat_sample(4,j)+u(2,0)*T;
			}
			//对每个点进行加权
			for(int j=0;j<11;j++)
			{
				math::Matrix<5,1> temp;
				double wm;
				if(j==0)
					wm=Wm1(0,0);
				else
					wm=Wm1(1,0);
				for(int i=0;i<5;i++)
				{
					temp(i,0)=X_pred_sample1(i,j);
				}
				X_pred+=temp*wm;
			}
			//计算P(k+1|k)
			for(int j=0;j<11;j++)
			{
				math::Matrix<5,1> temp;
				float wc;
				if(j==0)
					wc=Wc1(0,0);
				else
					wc=Wc1(0,0);
				for(int i=0;i<5;i++)
				{
					temp(i,0)=X_hat_sample(i,j)-X_pred(i,0);
				}

				P_pred+=temp*temp.transposed()*wc;
			}
			P_pred+=Q;
			//2、超声波有数据
			if (updated)
			{
				//==================UKF第三步，观测更新，得到==================================
				for(int i=0;i<5;i++)//第一列
					X_pred_sample2(i,0)=X_pred(i,0);
				math::Matrix<5,5> chol=cholesky(P_pred*(n+lambda2));
				chol=chol.transposed();
				chol/=1000;//??????
				//选取采样点
				for(int j=1;j<6;j++)
				{
					for(int i=0;i<5;i++)
						X_pred_sample2(i,j)=X_pred(i,0)+chol(i,j);
				}
				for(int j=6;j<11;j++)
				{
					for(int i=0;i<5;i++)
						X_pred_sample2(i,j)=X_pred(i,0)-chol(i,j-5);
				}
				//获取观测预测值
				for(int j=0;j<11;j++)
				{
					POINTF position_estimation(X_pred_sample2(0, j), X_pred_sample2(2, j));
					_yaw=X_pred_sample2(4, j);
					sonar_value_minimum(position_estimation, _yaw, sonarvalue_theory);
					for(int i=0;i<4;i++)
						sonarvalue_theory_sample(i,j)=sonarvalue_theory(i,0);
				}
				Y_pred.zero();
				Pxz_pred.zero();
				Pzz_pred.zero();
				//对每个观测点进行加权
				for(int j=0;j<11;j++)
				{
					math::Matrix<4,1> temp;
					float wm;
					if(j==0)
						wm=Wm2(0,0);
					else
						wm=Wm2(1,0);
					for(int i=0;i<4;i++)
					{
						temp(i,0)=sonarvalue_theory_sample(i,j);
					}
					Y_pred+=temp*wm;
				}
				for(int j=0;j<11;j++)
				{
					math::Matrix<5,1> temp_x;
					math::Matrix<4,1> temp_y;
					float wc;
					if(j==0)
						wc=Wc2(0,0);
					else
						wc=Wc2(1,0);
					for(int i=0;i<5;i++)
					{
						temp_x(i,0)=X_pred_sample2(i,j)-X_pred(i,0);
					}
					for(int i=0;i<4;i++)
					{
						temp_y(i,0)=sonarvalue_theory_sample(i,j)-Y_pred(i,0);
					}
					Pxz_pred+=temp_x*(temp_y.transposed())*wc;
					Pzz_pred+=temp_y*(temp_y.transposed())*wc;
				}
				Pzz_pred+=R;
				K=Pxz_pred*(Pzz_pred.inversed());
				//=======================处理跳变===============
				//注意观测编号对应的问题
				math::Matrix<4,1> error;
				for(int i=0;i<4;i++)
				{
					error(i,0)=distance.distance[i]-Y_pred(i,0);
					if(abs(error(i,0)>1))
						for(int j=0;j<5;j++)
							K(j,i)=0;
				}
				X_hat=X_pred+K*error;
				P_hat=P_pred-K*Pzz_pred*(K.transposed());
				//====================处理跳变 end=====================
			}
			else//超声波数据无更新，则只用惯导数据进行递推？？？？注意由于需要求P(k+1|k)所以也需要进行采样
			{
//				X_pred(0,0)=X_hat(0,0)+X_hat(1,0)*T+0.5*T*T*(u(0,0)*cos(X_hat(4,0))+u(1,0)*sin(X_hat(4,0)));
//				X_pred(1,0)=X_hat(1,0)+T*(u(0,0)*cos(X_hat(4,0))+u(1,0)*sin(X_hat(4,0)));
//				X_pred(2,0)=X_hat(2,0)+X_hat(3,0)*T+0.5*T*T*(u(0,0)*sin(X_hat(4,0))-u(1,0)*cos(X_hat(4,0)));
//				X_pred(3,0)=X_hat(3,0)+T*(u(0,0)*sin(X_hat(4,0))-u(1,0)*cos(X_hat(4,0)));
//				X_pred(4,0)=X_hat(4,0)+u(2,0)*T;
				X_hat=X_pred;
				P_hat=P_pred;
			}
			ukf_status.x=X_hat(0,0);
			ukf_status.vx=X_hat(1,0);
			ukf_status.y=X_hat(2,0);
			ukf_status.vy=X_hat(3,0);
			ukf_status.psi=X_hat(4,0);
		}
	}
	orb_publish(ORB_ID(ukf_localization), ukf_localization_pub,
						&ukf_status);
	usleep(10000);
	//关闭线程，串口
	warnx("pos_estimator_sonar_imu exiting.\n");
	task_running = false;

	return 0;
}

int mc_localization_UKF_main(int argc, char *argv[])
{
	math::Matrix<4,4> A,B;
	A(0,0)=1;A(0,1)=2;A(0,2)=3;A(0,3)=1;
	A(1,0)=2;A(1,1)=13;A(1,2)=21;A(1,3)=11;
	A(2,0)=3;A(2,1)=21;A(2,2)=38;A(2,3)=20;
	A(3,0)=1;A(3,1)=11;A(3,2)=20;A(3,3)=15;

	if (argc < 2)
	{
			usage("[pos_estimator_sonar_imu]missing command");
			return 1;
	}
	if (!strcmp(argv[1], "start"))
	{
		if (task_running)
		{
			warnx("[pos_estimator_sonar_imu]already running\n");
			return 0;
		}
		task_should_exit = false;
		control_task = px4_task_spawn_cmd("mc_localization_UKF", SCHED_DEFAULT,SCHED_PRIORITY_MAX - 5, 4000, task_main,
		(argv) ? (char * const *) &argv[2] : (char * const *) NULL);
		return (0);
	}
	if (!strcmp(argv[1], "stop"))
	{
		task_should_exit = true;
		return (0);
	}
	if (!strcmp(argv[1], "status"))
	{
		if (task_running)
		{
			warnx("[mc_localization_UKF] running");
			while (1)
			{
				//进行输出的观察
				printf("\n");
				printf("==============press CTRL+C to abort==============\n");
				char c;
				struct pollfd fds;
				int ret;
				fds.fd = 0;
				fds.events = POLLIN;
				ret = poll(&fds, 1, 0);
				if (ret > 0)
				{
					read(0, &c, 1);
					if (c == 0x03 || c == 0x63 || c == 'q')
					{
						warnx("User abort\n");
					}
						break;
				}
				usleep(400000);
			}
		}
		else
			warnx("[pos_estimator_sonar_imu]stopped");
		return (0);
	}
	usage("unrecognized command");
		return (1);
}

static void usage(const char *reason)
{
	if (reason)
	{
		fprintf(stderr, "%s\n", reason);
	}
	fprintf(stderr,"usage: pos_estimator_sonar_imu {start|stop|status} [param]\n\n");
	exit(1);
}
