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
float _yaw;//鍗曚綅锛氬姬搴�
struct ukf_localization_s ukf_status;
struct control_state_s inertial;
struct distance_sensor_s distance;
int _ctrl_state_sub; 					//control state subscription
int _distance_sensor_sub;          		// four mb12xx sonar sensor data
uint64_t cycle_time1;
uint64_t cycle_time2;


//浠ヤ笅瀹忓畾涔変负瀹ゅ唴瀹氫綅绠楁硶闇�瑕佺敤鍒扮殑璋冭妭鍙傛暟
#define PI							3.1415926
#define SONAR_NUMBER_SET 			4				//瓒呭０娉㈡暟閲忚缃�
#define MAP_NUMBER_SET   			5				//鍏堥獙鍦板浘缁存暟锛岀洰鍓嶅彲閫夊弬鏁板寘鎷�4/13/25
#define RAY_NUMBER_SET   			1				//鍘熸潵涓�5
//iffpc
#define cos_147                     -0.8386706
#define sin_147                     0.5446390
//鍏堥獙鍦板浘
struct PRIOR_MAP {
//	float x[MAP_NUMBER_SET] = { 0, 5, 5, 9, 9, 14, 14, 12, 12, 7, 7, 0, 0 };
//	float y[MAP_NUMBER_SET] = { 0, 0, 2, 2, 0,  0,  4,  4, 10, 10, 5, 5, 0 };//map1
	float x[MAP_NUMBER_SET] = { 0,0,1.9,1.9, 0 };
	float y[MAP_NUMBER_SET] = { 0,5,  5,  0, 0 };//map2
//	float x[MAP_NUMBER_SET] = { 0, 5,   5,   0, 0 };
//	float y[MAP_NUMBER_SET] = { 0, 0, 1.9, 1.9, 0 };//map3

};
PRIOR_MAP _map_test;

struct PRIOR_SONAR {
//	float x[RAY_NUMBER_SET] = { 1.00, 3.50, 6.00, 1.00, 3.50 };
//	float y[RAY_NUMBER_SET] = { -0.2, -0.1375, 0, 0.2, 0.1375 };
		float x[RAY_NUMBER_SET] = { 3.50};
		float y[RAY_NUMBER_SET] = {  0 };
};
PRIOR_SONAR _sonar_model;

//澧欓潰鐨勮搴�
//float Wall_Angle[MAP_NUMBER_SET - 1] = { 0, PI / 2, 0, PI / 2, 0, PI / 2, 0, PI
//		/ 2, 0, PI / 2, 0, PI / 2 };
float Wall_Angle[MAP_NUMBER_SET - 1] = { 0, PI / 2, 0, PI / 2};

//甯︽湁鍧愭爣鐨勭偣
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

//鍑芥暟澹版槑
extern "C" __EXPORT int mc_localization_UKF_main(int argc, char *argv[]);
int task_main(int argc, char *argv[]);
// 浣跨敤鎻愮ず鍑芥暟
static void usage(const char *reason);
math::Matrix<5,5> cholesky(math::Matrix<5,5> A);
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

//鍒ゅ畾涓ょ嚎娈典綅缃叧绯伙紝骞舵眰鍑轰氦鐐�(濡傛灉瀛樺湪)銆傛眰AB鐐逛箣闂寸嚎娈靛拰CD涔嬮棿绾挎鐨勪氦鐐广�傝繑鍥炲�间氦浜庣嚎涓�(2)锛屾浜�(1)锛屾棤浜�(0)
float Intersection(POINTF A, POINTF B, POINTF C, POINTF D,
		POINTF &intersection_point) {
	POINTF CA = { A.x - C.x,A.y - C.y };
	POINTF CB = { B.x - C.x,B.y - C.y };
	POINTF CD = { D.x - C.x,D.y - C.y };
	float dom = CB.x*CA.y - CB.y*CA.x;
	//AB鍜孋D涓嶅叡绾挎儏鍐�
	if (fabs(dom) > 0) {
		float a = (CB.x*CD.y - CD.x*CB.y) / dom;
		float b = (CD.x*CA.y - CA.x*CD.y) / dom;
		if (a >= 0 && b >= 0 && (a + b) >= 1) {
			intersection_point.x = CD.x / (a + b) + C.x;
			intersection_point.y = CD.y / (a + b) + C.y;
			return 1;
		}
		else {
			return 0;
		}
	}
	//CA鍜孋B鍏辩嚎鎯呭喌
	else {
		//CA鍜孋B鍚屽悜
		if (CA.x*CB.y + CA.y*CB.x > 0) {
			//C鍜孉鎴朆閲嶅悎锛屼氦鐐逛负C
			if ((CA.x*CA.x + CA.y*CA.y) < 0 || (CB.x*CB.x + CB.y*CB.y) < 0) {
				intersection_point = C;
				return 2;
			}
			//AB鍜孋D娌℃湁閲嶅悎閮ㄥ垎,娌℃湁浜ょ偣
			else if ((CD.x*CD.x + CD.y*CD.y - CA.x*CA.x - CA.y*CA.y) < 0 && (CD.x*CD.x + CD.y*CD.y - CB.x*CB.x - CB.y*CB.y) < 0) {
				return 0;
			}
			//AB鍜孋D鏈夐噸鍚堥儴鍒嗭紝鍙栬窛绂绘渶灏忕偣浣滀负浜ょ偣
			else {
				intersection_point = CA.x*CA.x + CA.y*CA.y > CB.x*CB.x + CB.y*CB.y ? B : A;
				return 2;
			}
		}
		//CA鍜孋B鍙嶅悜
		else
			return 2;
	}
}

//璁＄畻瓒呭０娉㈢殑鐞嗚瑙傛祴鍊�(鐢变簬UKF涓嶇敤绠梛acobi鐭╅樀鏁呯渷鍘讳簡璁＄畻澧欓潰鍜岃秴澹版尝瑙掑害鐨勯儴鍒�)
void sonar_value_minimum(POINTF location, float yaw,
		math::Matrix<4, 1> &distance_theory_minimum)
{
	float _distance_theory[4];

	_distance_theory[0] = 7;
	_distance_theory[1] = 7;
	_distance_theory[2] = 7;
	_distance_theory[3] = 7;

	/*
	 sonar_map_label鐢ㄤ簬瀛樺偍鏈変氦鐐圭殑瓒呭０娉㈠皠绾垮拰鍦板浘绾挎锛岄鍏堝叏閮ㄨ祴鍊�-1
	 褰撳搴斾綅缃湁浜ょ偣鏃讹紝鍒欎互浜ょ偣缂栧彿鍜屼綅缃暟鎹浛鎹�
	 濡傛灉娌℃湁浜ょ偣锛屽垯涓�-1涓嶅彉锛屼綔涓哄悗缁垽鏂殑鏍囧織浣�
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
	//瓒呭０娉㈣涓烘槸鍧囧寑鍒嗗竷
	for (int i = 0; i < SONAR_NUMBER_SET; i++) {
		cos_value = cos(90 * i * PI / 180);
		sin_value = sin(90 * i * PI / 180);

		for (int j = 0; j < RAY_NUMBER_SET; j++) {
			//鏃嬭浆鐭╅樀鐨勮缃�
			/*绗竴涓棆杞煩闃碉細[cos(psi),-sin(psi);sin(psi),cos(psi)];
			 * */
			_sonar_end_point_rotation.x = (_sonar_model.x[j] * cos_yaw)
					- (_sonar_model.y[j] * sin_yaw);
			_sonar_end_point_rotation.y = (_sonar_model.x[j] * sin_yaw)
					+ (_sonar_model.y[j] * cos_yaw);

			//the ith sonar model position x'=x*cos(90*(i-1))-y*sin(90*(i-1))
			//绗簩涓棆杞煩闃碉細[cos(90*(i-1)),-sin((90*(i-1));sin((90*(i-1))),cos((90*(i-1)))]
			_sonar_end_point.x = location.x
					+ (_sonar_end_point_rotation.x * cos_value)
					- (_sonar_end_point_rotation.y * sin_value);
			//the ith sonar model position y'=x*sin(90*(i-1))+y*cos(90*(i-1))
			_sonar_end_point.y = location.y
					+ (_sonar_end_point_rotation.x * sin_value)
					+ (_sonar_end_point_rotation.y * cos_value);

//			angle = atan2((_sonar_end_point.y - _sonar_start_point.y),
//					(_sonar_end_point.x - _sonar_start_point.x));
			_distance_theory_temp = 10;

			for (int k = 1; k < MAP_NUMBER_SET; k++) {


				_map_start_point.x = _map_test.x[k - 1]; //the start point of map line
				_map_start_point.y = _map_test.y[k - 1]; //the start point of map line
				_map_end_point.x = _map_test.x[k]; //the end point of map line
				_map_end_point.y = _map_test.y[k]; //the end point of map line

				_intersection_point.x = 1000;
				_intersection_point.y = 1000;

					// for calculate the intersection between the sonar model and the prior map
				Intersection(_sonar_start_point, _sonar_end_point,
							_map_start_point, _map_end_point,
							_intersection_point);

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
						//褰撴湁浜ょ偣鏃跺紑濮嬭祴鍊肩浉搴旀暟鎹�
						//i琛ㄧず绗琲涓秴澹版尝
						_distance_theory[i] = _distance_theory_temp;
					}
				}
			}
		}
	}
	// store the minimum distance in distance_theory_minimum
	distance_theory_minimum(0, 0) = _distance_theory[0];
	distance_theory_minimum(1, 0) = _distance_theory[1];
	distance_theory_minimum(2, 0) = _distance_theory[2];
	distance_theory_minimum(3, 0) = _distance_theory[3];
}

//妫�娴嬫儻鎬т紶鎰熷櫒鐨勬暟鍊兼湁娌℃湁鏇存柊
void ctrlStateUpdate(bool& updated) {
	math::Matrix<3, 3> _R; 			// rotation matrix from attitude quaternions
	orb_check(_ctrl_state_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(control_state), _ctrl_state_sub, &inertial);
		ukf_status.acc[0]=inertial.x_acc;//浠跨湡涓庡疄闄呯殑绗﹀彿鐩稿弽锛�
		ukf_status.acc[1]=inertial.y_acc;//浠跨湡涓庡疄闄呯殑绗﹀彿鐩稿弽锛�
		ukf_status.yaw_rate=inertial.yaw_rate;
		math::Quaternion q_att(inertial.q[0], inertial.q[1], inertial.q[2],
				inertial.q[3]);
		_R = q_att.to_dcm();
		math::Vector<3> euler_angles;
		euler_angles = _R.to_euler();
		ukf_status.yaw = euler_angles(2);

//		_yaw = euler_angles(2);		// the 3rd element of eulerian angle is yaw???????????????????????
//		ATT = euler_angles;
//		ATT = ATT*(180/PI);
	}
}

//妫�娴嬭窛绂讳紶鎰熷櫒锛堣秴澹版尝鎴栬�呮縺鍏夛級鐨勬暟鍊兼湁娌℃湁鏇存柊
void distanceSensorUpdate(bool& updated) {
	orb_check(_distance_sensor_sub, &updated);
	if (updated) {
		orb_copy(ORB_ID(distance_sensor), _distance_sensor_sub, &distance);
	}
}

//=====================璺ㄧ珛瀹為獙鍑芥暟瀹�=====================

//====================涓嬮潰鏄繍琛岀殑涓诲惊鐜▼搴�========================
//=========taks_main:UKF鐨勪富浣撶▼搴�========================
int task_main(int argc, char *argv[])
{
	usleep(1000);
	warnx("mc_localization_UKF is successful!\n");
	//绾跨▼鍚姩鏍囧織
	task_running = true;
	warnx("mc_localization_UKF start successfully\n");

	_ctrl_state_sub = orb_subscribe(ORB_ID(control_state));
	_distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
	orb_advert_t ukf_localization_pub = orb_advertise(ORB_ID(ukf_localization),
			&ukf_status);
	while (!task_should_exit)
	{
	//==================UKF绗竴姝ワ紝鍒濆鍖栵紝璁惧畾鍚勭鍒濆鐘舵��===================================
	//1銆佸畾涔夐渶瑕佺敤鍒扮殑鍚勭鐭╅樀
	math::Matrix<5,1> X_hat,X_pred;//鐘舵�侀噺锛坸,vx,y,vy,psi锛塜_hat涓篨(k+1|k+1)锛孹_pred涓篜(k+1|k)
	math::Matrix<5,11> X_hat_sample,X_pred_sample1,X_pred_sample2;//鐘舵�侀噺閲囨牱鐐�,鍒嗗埆涓篨(k|k),X(k+1|k)鐨勯噰鏍风偣
	math::Matrix<4,1> Y_pred;//瑙傛祴閲�(鍥涗釜瓒呭０娉㈣鏁板��)
	math::Matrix<3,1> u;//杈撳叆锛屽搴旀満浣撹酱瑙掗�熷害鍜岃鍔犻�熷害
	math::Matrix<5,5> P_hat,P_pred,Q;//璇樊鍗忔柟宸樀P(k|k),P(k+1|k)
	math::Matrix<5,4> Pxz_pred,K;//鍗″皵鏇煎鐩�
	math::Matrix<4,4> Pzz_pred,R;
	math::Matrix<2,1> Wm1,Wc1,Wm2,Wc2;//閲囨牱鐐规潈鍊�
	math::Matrix<4, 1> sonarvalue_theory;
	math::Matrix<4,11> sonarvalue_theory_sample;
	X_hat(0,0)=1.19f;	X_hat(1,0)=0.0f;	X_hat(2,0)=1.23f;	X_hat(3,0)=0.0f;	X_hat(4,0)=0.0f;
	P_hat(0,0)=1.0f;	P_hat(1,1)=0.1f;	P_hat(2,2)=1.0f;	P_hat(3,3)=0.1f;	P_hat(4,4)=0.01f;
	Q.zero();//杩囩▼鍣０鍒濆鍖�
	Q(0, 0) = 1.0f; Q(1, 1) = 0.2f; Q(2, 2) = 1.0f; Q(3, 3) = 0.2f; Q(4, 4) = 0.2f;
	R.identity();//瑙傛祴鍣０鍒濆鍖�
	R = R * 0.007f;
	//2銆佸畾涔夐渶瑕佺敤鍒扮殑鍚勭鍙橀噺
	float T=0.008f;//or 0.008f  鍔犻�熷害璁¤繍琛屽懆鏈�
	int n=5,beta=2;
	float alpha1=2,alpha2=0.01;
	float kappa1=3-n, kappa2=3-n;
	float lambda1=(alpha1*alpha1)*(n+kappa1)-n;
	float lambda2=(alpha2*alpha2)*(n+kappa2)-n;
	//3銆佸鍙橀噺杩涜鍒濆鍖�
	Wm1(0,0)=lambda1/(n+lambda1);
	Wm1(0,1)=0.5f/(n+lambda1);
	Wc1(0,0)=lambda1/(n+lambda1)+1-alpha1*alpha1+beta;
	Wc1(1,0)=0.5f/(n+lambda1);
	Wm2(0,0)=lambda2/(n+lambda2);
	Wm2(0,1)=0.5f/(n+lambda2);
	Wc2(0,0)=lambda2/(n+lambda2)+1-alpha2*alpha2+beta;
	Wc2(1,0)=0.5f/(n+lambda2);
//	static uint64_t last_run = 0;
	//寰幆寮�濮�
	while (!task_should_exit)
	{
//		float time_period = (hrt_absolute_time() - last_run) / 1000000.0f;
//		last_run = hrt_absolute_time();//娴嬮噺鏁翠釜杩涚▼鑰楁椂
		uint64_t time1=hrt_absolute_time();
		usleep(500);
		warnx("mc_localization_UKF start successfully\n");
		warnx("time of start is:%d",(int)hrt_absolute_time());
		//==================UKF绗簩姝ワ紝鐘舵�佹洿鏂�==================================
		bool updated = true;
		ctrlStateUpdate(updated);
		if (updated) //IMU鏁版嵁鏈夋洿鏂�
		{
//			inertial.x_acc=0;//鍏堢疆闆惰瀵熸晥鏋�
//			inertial.y_acc=0;//鍏堢疆闆惰瀵熸晥鏋�
//			inertial.yaw_rate=0;//鍏堢疆闆惰瀵熸晥鏋�
			u(0, 0) = inertial.x_acc;//u鏄満浣撶郴涓嬬殑鍔犻�熷害
			u(1, 0) = inertial.y_acc;
			//铏界劧鏄満浣撶郴涓嬬殑z杞磋鍔犻�熷害锛屼絾鏄亣璁惧墠鎻愭槸椋炴満鍦ㄤ簩缁村钩闈㈣繍鍔紝鏈轰綋婊氳浆鍜屽亸鑸閫熷害鍧囦负0
			//鐩墠鏆傛椂鎶婇檧铻轰华z杞磋閫熷害璁や负鏄亸鑸閫熷害
			u(2, 0) = inertial.yaw_rate;
			for(int i=0;i<5;i++)//绗竴鍒�
				X_hat_sample(i,0)=X_hat(i,0);
			math::Matrix<5,5> cho;
			cho=cholesky(P_hat*(n+lambda1));
			cho=cho.transposed();
			//閫夊彇閲囨牱鐐�
			for(int j=1;j<6;j++)
			{
				for(int i=0;i<5;i++)
					X_hat_sample(i,j)=X_hat(i,0)+cho(i,j-1);
			}
			for(int j=6;j<11;j++)
			{
				for(int i=0;i<5;i++)
					X_hat_sample(i,j)=X_hat(i,0)-cho(i,j-6);
			}
			X_pred.zero();
			P_pred.zero();
			//璁＄畻鍚勪釜閲囨牱鐐瑰搴旂殑X(k+1|k)
			for(int j=0;j<11;j++)
			{
				X_pred_sample1(0,j)=X_hat_sample(0,j)+X_hat_sample(1,j)*T+0.5f*T*T*(u(0,0)*(float)cos(X_hat_sample(4,j))-u(1,0)*(float)sin(X_hat_sample(4,j)));
				X_pred_sample1(1,j)=X_hat_sample(1,j)+T*(u(0,0)*(float)cos(X_hat_sample(4,j))-u(1,0)*(float)sin(X_hat_sample(4,j)));
				X_pred_sample1(2,j)=X_hat_sample(2,j)+X_hat_sample(3,j)*T+0.5f*T*T*(u(0,0)*(float)sin(X_hat_sample(4,j))+u(1,0)*(float)cos(X_hat_sample(4,j)));
				X_pred_sample1(3,j)=X_hat_sample(3,j)+T*(u(0,0)*(float)sin(X_hat_sample(4,j))+u(1,0)*(float)cos(X_hat_sample(4,j)));
				X_pred_sample1(4,j)=X_hat_sample(4,j)+u(2,0)*T;
//				X_pred_sample1(0,j)=X_hat_sample(0,j)+X_hat_sample(1,j)*T+0.5f*T*T*(u(0,0)*(float)cos(X_hat_sample(4,j))-u(1,0)*(float)sin(X_hat_sample(4,j)));
//				X_pred_sample1(1,j)=X_hat_sample(1,j)+T*(u(0,0)*(float)cos(X_hat_sample(4,j))-u(1,0)*(float)sin(X_hat_sample(4,j)));
//				X_pred_sample1(2,j)=X_hat_sample(2,j)+X_hat_sample(3,j)*T+0.5f*T*T*(u(0,0)*(float)sin(X_hat_sample(4,j))+u(1,0)*(float)cos(X_hat_sample(4,j)));
//				X_pred_sample1(3,j)=X_hat_sample(3,j)+T*(u(0,0)*(float)sin(X_hat_sample(4,j))+u(1,0)*(float)cos(X_hat_sample(4,j)));
//				X_pred_sample1(4,j)=X_hat_sample(4,j)+u(2,0)*T;
			}
			//瀵规瘡涓偣杩涜鍔犳潈
			for(int j=0;j<11;j++)
			{
				math::Matrix<5,1> temp;
				float wm;
				if(j==0)
					wm=Wm1(0,0);
				else
					wm=Wm1(1,0);
				for(int i=0;i<5;i++)
					temp(i,0)=X_pred_sample1(i,j);
				X_pred+=temp*wm;
			}
			//璁＄畻P(k+1|k)
			for(int j=0;j<11;j++)
			{
				math::Matrix<5,1> temp;
				float wc;
				if(j==0)
					wc=Wc1(0,0);
				else
					wc=Wc1(1,0);
				for(int i=0;i<5;i++)
					temp(i,0)=X_pred_sample1(i,j)-X_pred(i,0);
				P_pred+=temp*(temp.transposed())*wc;
			}
			P_pred+=Q;
			distanceSensorUpdate(updated);
//			updated=0;//鏆傛椂灞忚斀瓒呭０娉紝鍙敤鍔犻�熷害璁￠�掓帹
			//1銆佽秴澹版尝锛堟縺鍏夛級鏁版嵁鏈夋洿鏂�
			if (updated)
			{
				ukf_status.laser_distance[0]=distance.distance[0]/1000;
				ukf_status.laser_distance[1]=distance.distance[1]/1000;
				ukf_status.laser_distance[2]=distance.distance[2]/1000;
				ukf_status.laser_distance[3]=distance.distance[3]/1000;
				//==================UKF绗笁姝ワ紝瑙傛祴鏇存柊锛屽緱鍒�==================================
				for(int i=0;i<5;i++)//绗竴鍒�
					X_pred_sample2(i,0)=X_pred(i,0);
				math::Matrix<5,5> chol=cholesky(P_pred*(n+lambda2));
				chol=chol.transposed();
				chol/=100;//??????
				//閫夊彇閲囨牱鐐�
				for(int j=1;j<6;j++)
				{
					for(int i=0;i<5;i++)
						X_pred_sample2(i,j)=X_pred(i,0)+chol(i,j-1);
				}
				for(int j=6;j<11;j++)
				{
					for(int i=0;i<5;i++)
						X_pred_sample2(i,j)=X_pred(i,0)-chol(i,j-6);
				}
				//鑾峰彇瑙傛祴棰勬祴鍊�
				uint64_t time2=hrt_absolute_time();
				for(int j=0;j<11;j++)
				{
					POINTF position_estimation(X_pred_sample2(0, j), X_pred_sample2(2, j));
					_yaw=X_pred_sample2(4, j);
					sonar_value_minimum(position_estimation, _yaw, sonarvalue_theory);
					for(int i=0;i<4;i++)
						sonarvalue_theory_sample(i,j)=sonarvalue_theory(i,0);
				}
				uint64_t time3=hrt_absolute_time();
				cycle_time1=time3-time2;
				Y_pred.zero();
				Pxz_pred.zero();
				Pzz_pred.zero();
				//瀵规瘡涓娴嬬偣杩涜鍔犳潈
				for(int j=0;j<11;j++)
				{
					math::Matrix<4,1> temp;
					float wm;
					if(j==0)
						wm=Wm2(0,0);
					else
						wm=Wm2(1,0);
					for(int i=0;i<4;i++)
						temp(i,0)=sonarvalue_theory_sample(i,j);
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
						temp_x(i,0)=X_pred_sample2(i,j)-X_pred(i,0);
					for(int i=0;i<4;i++)
						temp_y(i,0)=sonarvalue_theory_sample(i,j)-Y_pred(i,0);
					Pxz_pred+=temp_x*(temp_y.transposed())*wc;
					Pzz_pred+=temp_y*(temp_y.transposed())*wc;
				}
				Pzz_pred+=R;
				K=Pxz_pred*(Pzz_pred.inversed());
				//=======================澶勭悊璺冲彉===============
				//娉ㄦ剰瑙傛祴缂栧彿瀵瑰簲鐨勯棶棰�
				math::Matrix<4,1> error;
				//=======鍏堟妸璺濈娴嬮噺鍊兼崲鎴愬浐瀹氬�硷紙鏈疄闄呭疄楠岄琛屾椂閲囩敤锛�======
//				distance.distance[0]=5629;
//				distance.distance[1]=1225;
//				distance.distance[2]=1225;
//				distance.distance[3]=1225;
				//===================================================
				for(int i=0;i<4;i++)
				{
					error(i,0)=distance.distance[i]/1000-Y_pred(i,0);
					if(abs(error(i,0))>0.5)
					{
						for(int j=0;j<5;j++)
							K(j,i)=0;
						ukf_status.corrected=1;//鏈夎烦鍙�
					}
				}
				X_hat=X_pred+K*error;
				P_hat=P_pred-K*Pzz_pred*(K.transposed());

				//====================澶勭悊璺冲彉 end=====================
			}//
			else//瓒呭０娉㈡暟鎹棤鏇存柊锛屽垯鍙敤鎯鏁版嵁杩涜閫掓帹  娉ㄦ剰鐢变簬闇�瑕佹眰P(k+1|k)鎵�浠ュ嵆浣挎病鏈夎娴嬫暟鎹篃闇�瑕佽繘琛岄噰鏍�
			{
//				X_pred(0,0)=X_hat(0,0)+X_hat(1,0)*T+0.5*T*T*(u(0,0)*cos(X_hat(4,0))+u(1,0)*sin(X_hat(4,0)));
//				X_pred(1,0)=X_hat(1,0)+T*(u(0,0)*cos(X_hat(4,0))+u(1,0)*sin(X_hat(4,0)));
//				X_pred(2,0)=X_hat(2,0)+X_hat(3,0)*T+0.5*T*T*(u(0,0)*sin(X_hat(4,0))-u(1,0)*cos(X_hat(4,0)));
//				X_pred(3,0)=X_hat(3,0)+T*(u(0,0)*sin(X_hat(4,0))-u(1,0)*cos(X_hat(4,0)));
//				X_pred(4,0)=X_hat(4,0)+u(2,0)*T;
				X_hat=X_pred;
				P_hat=P_pred;
				ukf_status.corrected=0;
			}
			ukf_status.x=X_hat(0,0);
			ukf_status.vx=X_hat(1,0);
			ukf_status.y=X_hat(2,0);
			ukf_status.vy=X_hat(3,0);
			ukf_status.psi=X_hat(4,0);

		}
		//iffpc south-west to north-east
		float x1= ukf_status.x;
		ukf_status.x=((float)cos_147)*ukf_status.x+((float)sin_147)*(ukf_status.y);
		ukf_status.y=((float)(-sin_147))*x1+((float)cos_147)*(ukf_status.y);
		float vx1= ukf_status.vx;
		ukf_status.vx=((float)cos_147)*ukf_status.vx+((float)sin_147)*(ukf_status.vy);
		ukf_status.vy=((float)(-sin_147))*vx1+((float)cos_147)*(ukf_status.vy);
		orb_publish(ORB_ID(ukf_localization), ukf_localization_pub,
							&ukf_status);
		uint64_t time4=hrt_absolute_time();
		cycle_time2=time4-time1;
	}

	usleep(1000);
	//鍏抽棴绾跨▼锛屼覆鍙�
	warnx("pos_estimator_sonar_imu exiting.\n");
	task_running = false;
	}
	return 0;
}


int mc_localization_UKF_main(int argc, char *argv[])
{
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
				//杩涜杈撳嚭鐨勮瀵�
				PX4_INFO("time interval of small circulation is:%d",(int)cycle_time1);
				PX4_INFO("time interval of big circulation is:%d",(int)cycle_time2);
				PX4_INFO("UKF_x = %.3f\tUKF_y = %.3f",(double)ukf_status.x,(double)ukf_status.y);
				PX4_INFO("UKF_vx = %.3f\tUKF_vy = %.3f",(double)ukf_status.vx,(double)ukf_status.vy);
				PX4_INFO("UKF_psi angle = %.3f",(double)ukf_status.psi);
				PX4_INFO("acc_x = %.3f\tacc_y = %.3f",(double)ukf_status.acc[0],(double)ukf_status.acc[1]);
				PX4_INFO("yaw rate = %.3f\tquaternion yaw = %.3f",(double)ukf_status.yaw_rate,(double)ukf_status.yaw);
				PX4_INFO("laser_distance_1=%.3fm\tlaser_distance_2=%.3fm",(double)ukf_status.laser_distance[0],(double)ukf_status.laser_distance[1]);
				PX4_INFO("laser_distance_3=%.3fm\tlaser_distance_4=%.3fm",(double)ukf_status.laser_distance[2],(double)ukf_status.laser_distance[3]);
				PX4_INFO("control_state:x=%.3fm\ty=%.3fm",(double)inertial.x_pos,(double)inertial.y_pos);
				//PX4_INFO("control_state:vx=%.3fm\vy=%.3fm",(double)inertial.,(double)inertial.y_pos);
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
				usleep(800000);
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
