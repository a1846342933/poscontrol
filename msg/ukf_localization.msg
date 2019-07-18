
float32 x		#position in x direction in map
float32 vx		#velocity in x direction in map
float32 y		#position in y direction in map
float32 vy		#velocity in y direction in map
float32 psi		#yaw angle in map
float32[4] laser_distance 	#4个激光观测量
uint8 corrected	#观测量是否对状态量进行了修正 是1 否0
float32[2] acc		#x方向加速度（机体轴）和y方向加速度（机体轴）
float32 yaw_rate	#z轴角加速度（机体轴）
float32 yaw		#四元数推算出来的机体系偏航角
