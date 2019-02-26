#include <px4_config.h>
#include <px4_defines.h>
#include <px4_getopt.h>
#include <px4_workqueue.h>

#include <drivers/device/i2c.h>

#include <sys/types.h>
#include <stdint.h>
#include <stdlib.h>
#include <stdbool.h>
#include <semaphore.h>
#include <string.h>
#include <fcntl.h>
#include <poll.h>
#include <stdio.h>
#include <math.h>
#include <unistd.h>

//#include <perf/perf_counter.h>

#include <drivers/drv_hrt.h>
#include <drivers/drv_range_finder.h>
#include <drivers/device/ringbuffer.h>

#include <uORB/uORB.h>
#include <uORB/topics/distance_sensor.h>
#include "VL53L1_register_map.h"
//#include "VL53L1X.h"

#include <board_config.h>


/* Configuration Constants */
#define VL53L1X_BUS_DEFAULT PX4_I2C_BUS_EXPANSION

//#define VL53L1X_BASEADDR 0b0101001 // 7-bit address Ĭ�ϵ�ַ0x52实际上是01010010，b代表二进制 7位代表高位，最地位表示读写。
#define VL53L1X_BASEADDR 0b0101000
//#define VL53L1X_BASEADDR 0x70
#define VL53L1X_DEVICE_PATH "/dev/vl53l1x"

//���ǼĴ����ĵ�ַ����
#define	 VL53L1_RANGESTATUS_RANGE_VALID				0
/*!<The Range is valid. */
#define	 VL53L1_RANGESTATUS_SIGMA_FAIL				1
/*!<Sigma Fail. */
#define	 VL53L1_RANGESTATUS_SIGNAL_FAIL				2
/*!<Signal fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED	3
/*!<Target is below minimum detection threshold. */
#define	 VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL			4
/*!<Phase out of valid limits -  different to a wrap exit. */
#define	 VL53L1_RANGESTATUS_HARDWARE_FAIL			5
/*!<Hardware fail. */
#define	 VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL	6
/*!<The Range is valid but the wraparound check has not been done. */
#define	VL53L1_RANGESTATUS_WRAP_TARGET_FAIL			7
/*!<Wrapped target - no matching phase in other VCSEL period timing. */
#define	VL53L1_RANGESTATUS_PROCESSING_FAIL			8
/*!<Internal algo underflow or overflow in lite ranging. */
#define	VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL			9
/*!<Specific to lite ranging. */
#define	VL53L1_RANGESTATUS_SYNCRONISATION_INT			10
/*!<1st interrupt when starting ranging in back to back mode. Ignore data. */
#define	VL53L1_RANGESTATUS_RANGE_VALID_MERGED_PULSE		11
/*!<All Range ok but object is result of multiple pulses merging together.
* Used by RQL for merged pulse detection
*/
#define	VL53L1_RANGESTATUS_TARGET_PRESENT_LACK_OF_SIGNAL	12
/*!<Used  by RQL  as different to phase fail. */
#define	VL53L1_RANGESTATUS_MIN_RANGE_FAIL			13
/*!<User ROI input is not valid e.g. beyond SPAD Array.*/
#define	VL53L1_RANGESTATUS_RANGE_INVALID			14
/*!<lld returned valid range but negative value ! */
#define	 VL53L1_RANGESTATUS_NONE				255

#define VL53L1X_US 1000 /*  1ms */
#define VL53L1X_SAMPLE_RATE 50000 //50ms


//�Զ��庯������
//ע������ǽṹ���ڵĺ������Ͳ��õ�������������ֱ��������Ӧͷ�ļ�VL53L1X.h����
// 1��ʹ����ʾ����
//static void usage(const char *reason);
//2�� VL53L1X������
//__EXPORT int vl531x_main(int argc, char *argv[]);
//3�� VL53L1X�̺߳���
//int vl531x_thread_main(int argc, char *argv[]);


class VL53L1X : public device::I2C
{
public:
	VL53L1X(int bus = VL53L1X_BUS_DEFAULT, int address = VL53L1X_BASEADDR);

	virtual ~VL53L1X();

	virtual int init();

	virtual ssize_t read(device::file_t *filp, char *buffer, size_t buflen);

	virtual int ioctl(device::file_t *filp, int cmd, unsigned long arg);

	/**
	* Diagnostics - print some basic information about the driver.
	*/
	void print_info();

protected:
	virtual int probe();

private:
	int _class_instance;
	orb_advert_t _distance_sensor_pub;
	ringbuffer::RingBuffer *_reports;
	bool _sensor_ok;
	work_s _work;
	int _measure_ticks;
	struct distance_sensor_s report;

	uint8_t _index_counter = VL53L1X_BASEADDR; //_index_counterΪ����8λ��ַ
	uint8_t saved_vhv_init;
	bool did_timeout;
	uint16_t io_timeout;
	static const uint32_t TimingGuard = 4528;
	static const uint16_t TargetRate = 0x0A00;
	uint16_t fast_osc_frequency;
	uint16_t osc_calibrate_val;
	bool calibrated;
	uint8_t saved_vhv_timeout;
	uint16_t timeout_start_ms;

	struct RangingData
	{
		uint16_t range_mm;
		uint8_t range_status; //RangeStatus range_status
		float peak_signal_count_rate_MCPS;
		float ambient_count_rate_MCPS;
	};
	RangingData ranging_data;//������մ����õ��ľ�����

	struct ResultBuffer
	{
		uint8_t range_status;
		// uint8_t report_status: not used
		uint8_t stream_count;
		uint16_t dss_actual_effective_spads_sd0;
		// uint16_t peak_signal_count_rate_mcps_sd0: not used
		uint16_t ambient_count_rate_mcps_sd0;
		// uint16_t sigma_sd0: not used
		// uint16_t phase_sd0: not used
		uint16_t final_crosstalk_corrected_range_mm_sd0;
		uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
	};
	ResultBuffer results;//��Ŵ�I2C�ж�ȡ���ľ�����


	void start();

	/**
	* Stop the automatic measurement state machine.
	*/
	void stop();

	/**
	* Perform a poll cycle; collect from the previous measurement
	* and start a new one.
	*/
	void cycle();
	//int measure();
	int collect();
	static void cycle_trampoline(void *arg);
	void writeReg(uint8_t address, uint16_t reg, uint8_t value);
	void writeReg16Bit(uint8_t address, uint16_t reg, uint16_t value);
	void writeReg32Bit(uint8_t address, uint16_t reg, uint32_t value);
	uint8_t readReg(uint8_t address, uint16_t reg);
	uint16_t readReg16Bit(uint8_t address, uint16_t reg);
	uint32_t readReg32Bit(uint8_t address, uint16_t reg);
	bool sensorInit(uint8_t address, bool io_2v8);
	void startContinuous(uint8_t address, uint32_t period_ms);
	void stopContinuous(uint8_t address);
	bool init(uint8_t address, bool io_2v8);
	uint16_t readsensor(uint8_t address, bool blocking = true);
	void readResults(uint8_t address);
	void getRangingData();
	void updateDSS(uint8_t address);
	enum DistanceMode { Short, Medium, Long, Unknown };
	DistanceMode distance_mode;
	bool setDistanceMode(uint8_t address,DistanceMode mode);
	bool setMeasurementTimingBudget(uint8_t address, uint32_t budget_us);
	uint32_t getMeasurementTimingBudget(uint8_t address);
	uint32_t calcMacroPeriod(uint8_t vcsel_period);
	uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_us,uint32_t macro_period_us);
	uint32_t timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
	uint16_t encodeTimeout(uint32_t timeout_mclks);
	uint32_t decodeTimeout(uint16_t reg_val);
	void setTimeout(uint16_t timeout) { io_timeout = timeout; }
	void startTimeout(){timeout_start_ms=hrt_absolute_time()/1000;}
	bool checkTimeoutExpired(){return (io_timeout>0)&&((uint16_t)(hrt_absolute_time()/1000-timeout_start_ms)>io_timeout);}
	bool dataReady(uint8_t address){return (readReg(address,VL53L1_GPIO__TIO_HV_STATUS)==0);}//该寄存器存储数据是否准备好？等于零表示已准备好。
	float countRateFixedToFloat(uint16_t count_rate_fixed){return (float)count_rate_fixed/(1<<7);}
};

extern "C" __EXPORT int vl53l1x_main(int argc, char *argv[]);



VL53L1X::VL53L1X(int bus , int address) :
	I2C("VL53L1X", VL53L1X_DEVICE_PATH, bus, address, 400000),
	_class_instance(-1),
	_distance_sensor_pub(nullptr),
	_reports(nullptr),
	_sensor_ok(false),
	_measure_ticks(0),
	saved_vhv_init(0),
	io_timeout(0),
	calibrated(false),
	saved_vhv_timeout(0),
	distance_mode(Unknown)
	{
		// up the retries since the device misses the first measure attempts
		I2C::_retries = 3;

		// enable debug() calls
		_debug_enabled = false;//���ù�

		// work_cancel in the dtor will explode if we don't do this...
		memset(&_work, 0, sizeof(_work));
	}

VL53L1X::~VL53L1X()
{
		/* make sure we are truly inactive */
		stop();

		/* free any existing reports */
		if (_reports != nullptr) {
			delete _reports;
		}

		if (_distance_sensor_pub != nullptr) {
			orb_unadvertise(_distance_sensor_pub);
		}

		if (_class_instance != -1) {
			unregister_class_devname(RANGE_FINDER_BASE_DEVICE_PATH, _class_instance);
		}

		// free perf counters
		/*perf_free(_sample_perf);
		perf_free(_comms_errors);*/
}

int	VL53L1X::init()
	{
		int ret = OK;
		
		/* do I2C init (and probe) first */
		if (I2C::init() != OK) {
			ret = PX4_ERROR;
			PX4_ERR("i2c init failed!!!");
			goto out;
		}

		/* allocate basic report buffers */
		_reports = new ringbuffer::RingBuffer(2, sizeof(distance_sensor_s));

		if (_reports == nullptr) {
			ret = PX4_ERROR;
			PX4_ERR("_reports creae failed!!!");
			goto out;
		}
		//��֪�����RANGE_FINDER_BASE_DEVICE_PATH�Ķ��� ���ù�
		_class_instance = register_class_devname(RANGE_FINDER_BASE_DEVICE_PATH);
		memset(&report,0,sizeof(report));
		_distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &report);
		if (_distance_sensor_pub == nullptr) {
			DEVICE_LOG("failed to create distance_sensor object. Did you start uOrb?");
		}

		// XXX we should find out why we need to wait 200 ms here
		usleep(200000);
		/* sensor is ok, but we don't really know if it is within range */
		_sensor_ok = true;

//		if(!sensorInit(_index_counter,true))
//		{PX4_ERR("sensor init failed!!");}
	out:
		return ret;
	}

ssize_t VL53L1X::read(device::file_t *filp, char *buffer, size_t buflen)
{
	unsigned count = buflen / sizeof(struct distance_sensor_s);
	struct distance_sensor_s *rbuf = reinterpret_cast<struct distance_sensor_s *>(buffer);
	int ret = 0;

	/* buffer must be large enough */
	if (count < 1) {
		return -ENOSPC;
	}

	/* if automatic measurement is enabled */
	if (_measure_ticks > 0) {

		/*
		* While there is space in the caller's buffer, and reports, copy them.
		* Note that we may be pre-empted by the workq thread while we are doing this;
		* we are careful to avoid racing with them.
		*/
		while (count--) {
			if (_reports->get(rbuf)) {
				ret += sizeof(*rbuf);
				rbuf++;
			}
		}

		/* if there was no data, warn the caller */
		return ret ? ret : -EAGAIN;
	}

	/* manual measurement - run one conversion */
	do {
		_reports->flush();

		/* trigger a measurement */
//		if (OK != measure()) {
//			ret = -EIO;
//			break;
//		}
//
//		while (!_collect_phase);

		///* run the collection phase */
		//if (OK != collect()) {
		//	ret = -EIO;
		//	break;
		//}

		/* state machine will have generated a report, copy it out */
		if (_reports->get(rbuf)) {
			ret = sizeof(*rbuf);
		}

	} while (0);

	return ret;
}

int VL53L1X::ioctl(device::file_t *filp, int cmd, unsigned long arg)
{
	switch (cmd) {

	case SENSORIOCSPOLLRATE: {
		switch (arg) {

		case 0:
			return -EINVAL;

		case SENSOR_POLLRATE_DEFAULT: {
			/* do we need to start internal polling? */
			bool want_start = (_measure_ticks == 0);

			/* set interval for next measurement to minimum legal value */
			_measure_ticks = USEC2TICK(VL53L1X_SAMPLE_RATE);

			/* if we need to start the poll state machine, do it */
			if (want_start) {
				start();

			}

			start();
			//warnx("start measure!!");
			return OK;
		}

		case SENSOR_POLLRATE_MANUAL: {

			stop();
			_measure_ticks = 0;
			return OK;
		}

									 /* adjust to a legal polling interval in Hz */
		default: {
			/* do we need to start internal polling? */
			bool want_start = (_measure_ticks == 0);

			/* convert hz to tick interval via microseconds */
			unsigned ticks = USEC2TICK(1000000 / arg);

			/* update interval for next measurement */
			_measure_ticks = ticks;

			/* if we need to start the poll state machine, do it */
			if (want_start) {
				start();
			}

			return OK;
		}
		}
	}

	default:
		/* give it to the superclass */
		return I2C::ioctl(filp, cmd, arg);
	}
}



void VL53L1X::print_info()
{
	//perf_print_counter(_sample_perf);
	//perf_print_counter(_comms_errors);
	printf("poll interval:  %u ticks\n", _measure_ticks);
	_reports->print_info("report queue");
}

//probe的执行顺序：在创建一个新的VL53L1X时在I2C初始化之前执行。
int VL53L1X::probe()
{
	//add by fxk
	/*************i2c总线测试1***************/
//	uint8_t aaa = readReg(_index_counter,VL53L1_IDENTIFICATION__MODEL_ID);
//	warnx("the value of the MODULE_TYPE register is: %02x",aaa);
//		uint8_t bbb =readReg(_index_counter,VL53L1_IDENTIFICATION__MODULE_TYPE);
//		warnx("the value of the MODULE_TYPE register is: %02x",bbb);
//		uint8_t ccc =readReg(_index_counter,VL53L1_IDENTIFICATION__REVISION_ID);
//		warnx("the value of the REVISION_ID register is: %02x",ccc);
//		warnx("================================================");

	/*************i2c总线测试2***************/
//_index_counter=0x70;
//set_address(_index_counter);
//int ret=0;
//while(1)
//{
//	uint8_t cmd = 0x51;
//	ret = transfer(&cmd, 1, nullptr, 0);usleep(500000);
//	if(ret!=OK)
//		warnx("sonar address transfer fail!");
//	uint8_t val[2];
//	ret = transfer(nullptr, 0, &val[0], 2);
//	if (ret!=OK)
//		warnx("sonar read distance fail!");
//	else
//		warnx("the distance is:%02x,%02x",val[0],val[1]);
//}
		/*************总线测试2完***************/

	//_index_counter =0b0101000;

	//readReg(_index_counter,VL53L1_I2C_SLAVE__DEVICE_ADDRESS);
	for(_index_counter=0b0000000;_index_counter<0b1000000;_index_counter++)
		{	warnx("the index_counter value is:%02x",_index_counter);
			warnx("the index_counter address is:%02x",readReg(_index_counter,VL53L1_I2C_SLAVE__DEVICE_ADDRESS));
			warnx("the MODEL_ID register is:%02x",readReg(_index_counter,VL53L1_IDENTIFICATION__MODEL_ID));
			warnx("=============================================");
		}
	writeReg(_index_counter,VL53L1_I2C_SLAVE__DEVICE_ADDRESS,0b0101000);
	//_index_counter =0b0101001;
	if (sensorInit(_index_counter, 1) == true) {
//		while(1)
//		{warnx("the time is:%010d",(int)hrt_absolute_time());
//		usleep(1000000);}
		return OK;
	}

	// not found on any address
	return -EIO;
}


//д�Ĵ���
// Write an 8-bit register
void VL53L1X::writeReg(uint8_t address, uint16_t reg, uint8_t value)
{
	int ret=OK;
	set_address(address);
	uint8_t val[3];
	uint8_t reg_h=(reg>>8)&0xFF;
	uint8_t reg_l=reg&0xFF;
	val[0]=reg_h;
	val[1]=reg_l;
	val[2]=value;
	ret|=transfer(&val[0], 3, nullptr, 0);//高8位
	//transfer(&reg_l, 1, nullptr, 0);//低8位
	//transfer(&value, 1,nullptr, 0);
	if(ret!=OK)
		PX4_ERR("writeReg fail!");
}

// Write a 16-bit register
void VL53L1X::writeReg16Bit(uint8_t address, uint16_t reg, uint16_t value)
{
		int ret=OK;
		uint8_t reg_h=(reg>>8)&0xFF;
		uint8_t reg_l=reg&0xFF;
		uint8_t val[4];
		set_address(address);
//		transfer(&reg_h, 1, nullptr, 0);//高8位
//		transfer(&reg_l, 1, nullptr, 0);//低8位
		uint8_t value_h=(value>>8)&0xFF;
		uint8_t value_l=(value)&0xFF;
//		transfer(&value_h, 1, nullptr, 0);
//		transfer(&value_l, 1, nullptr, 0);
		val[0]=reg_h;
		val[1]=reg_l;
		val[2]=value_h;
		val[3]=value_l;
		ret|=transfer(&val[0], 4, nullptr, 0);
		if(ret!=OK)
				PX4_ERR("writeReg16 fail!");
}

// Write a 32-bit register
void VL53L1X::writeReg32Bit(uint8_t address, uint16_t reg, uint32_t value)
{
		int ret=OK;
		uint8_t reg_h=(reg>>8)&0xFF;
		uint8_t reg_l=reg&0xFF;
		uint8_t val[6];
		set_address(address);
//		transfer(&reg_h, 1, nullptr, 0);//高8位
//		transfer(&reg_l, 1, nullptr, 0);//低8位
		uint8_t value_1=(value>>24)&0xFF;//25-32位
		uint8_t value_2=(value>>16)&0xFF;//17-24位
		uint8_t value_3=(value>>8)&0xFF;//9-16位
		uint8_t value_4=value&0xFF;//1-8位

//		transfer(&value_1, 1, nullptr, 0);
//		transfer(&value_2, 1, nullptr, 0);
//		transfer(&value_3, 1, nullptr, 0);
//		transfer(&value_4, 1, nullptr, 0);
		val[0]=reg_h;
		val[1]=reg_l;
		val[2]=value_1;
		val[3]=value_2;
		val[4]=value_3;
		val[5]=value_4;
		ret|=transfer(&val[0], 6, nullptr, 0);
		if(ret!=OK)
				PX4_ERR("writeReg32 fail!");
}


//���Ĵ���
// Read an 8-bit register
uint8_t VL53L1X::readReg(uint8_t address, uint16_t reg)//_index_counterΪ����8λ��ַ��regΪ�Ĵ�����ַ��16λ�Ĵ����洢��8λ����
{
	int ret=OK;
	uint8_t value;
	uint8_t index_array[2];//寄存器地址
	//int ret;
	set_address(address);
	//д�Ĵ����
	uint8_t reg_h=(reg>>8)&0xFF;
	uint8_t reg_l=reg&0xFF;   //test by fxk
	index_array[0]=reg_h;
	index_array[1]=reg_l;
	ret|=transfer(&index_array[0], 2, nullptr, 0);//高8位
	//ret=transfer(&reg_l, 1, nullptr, 0);//低8位
	//PX4_ERR("ret2=%d",ret);//by fxk
//	if (OK != ret) {
//		perf_count(_comms_errors);
//		return ret;
//	}
	/* read from the sensor */
	//���Ĵ������ݴ���value
	ret|=transfer(nullptr, 0, &value, 1);
//	if (OK != ret) {
//		perf_count(_comms_errors);
//		return ret;
//	}
	if(ret!=OK)
			PX4_ERR("readReg fail!");
	return value;
}

// Read a 16-bit register
uint16_t VL53L1X::readReg16Bit(uint8_t address, uint16_t reg)//��16λ�Ĵ����洢��2��8λ���� regΪ�Ĵ�����ַ
{
	uint16_t value;
	uint8_t value_h;
	uint8_t value_l;
	uint8_t val[2];
	int ret=OK;//ret调试用
	set_address(address);
	uint8_t reg_h=(reg>>8)&0xFF;
	uint8_t reg_l=reg&0xFF;
	uint8_t index_array[2];
	index_array[0]=reg_h;
	index_array[1]=reg_l;
	ret|=transfer(&index_array[0], 2, nullptr, 0);//传地址
//	transfer(&reg_h, 1, nullptr, 0);//高8位
//	transfer(&reg_l, 1, nullptr, 0);//低8位
//	transfer(nullptr, 0, &value_h, 1);
//	transfer(nullptr, 0, &value_l, 1);
	ret|= transfer(nullptr, 0, &val[0], 2);
	value_h=val[0];
	value_l=val[1];
	value = (value_h<<8)|value_l;
	if(ret!=OK)
			PX4_ERR("readreg16 fail!!!");
	return value;
}

// Read a 32-bit register
uint32_t VL53L1X::readReg32Bit(uint8_t address, uint16_t reg)
{
	int ret=OK;//调试用
	uint32_t value;
	uint8_t value_1;//25-32
	uint8_t value_2;//17-24
	uint8_t value_3;//9-16
	uint8_t value_4;//1-8
	uint8_t val[4];
	set_address(address);
	uint8_t reg_h=(reg>>8)&0xFF;
	uint8_t reg_l=reg&0xFF;
	uint8_t index_array[2];
	index_array[0]=reg_h;
	index_array[1]=reg_l;
	ret|=transfer(&index_array[0], 2, nullptr, 0);
//	transfer(&reg_h, 1, nullptr, 0);//高8位
//	transfer(&reg_l, 1, nullptr, 0);//低8位
//	transfer(nullptr, 0, &value_1, 1);
//	transfer(nullptr, 0, &value_2, 1);
//	transfer(nullptr, 0, &value_3, 1);
//	transfer(nullptr, 0, &value_4, 1);
	ret|= transfer(nullptr, 0, &val[0], 4);
	value_1=val[0];
	value_2=val[1];
	value_3=val[2];
	value_4=val[3];
	value = (value_1<<24)|(value_2<<16)|(value_3<<24)|(value_4);
	if(ret!=OK)
			PX4_ERR("readreg32 fail!!!");
	return value;
}

//��ʼ���ָ��
void VL53L1X::startContinuous(uint8_t address,uint32_t period_ms)
{
	// from VL53L1_set_inter_measurement_period_ms()
	writeReg32Bit(address, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * osc_calibrate_val);

	writeReg(address, VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range
	writeReg(address, VL53L1_SYSTEM__MODE_START, 0x40); // mode_range__timed
}

//ֹͣ���ָ��
void VL53L1X::stopContinuous(uint8_t address)
{
	writeReg(address,VL53L1_SYSTEM__MODE_START, 0x80); // mode_range__abort
											   // VL53L1_low_power_auto_data_stop_range() begin

	calibrated = false;

	// "restore vhv configs"
	if (saved_vhv_init != 0)
	{
		writeReg(address, VL53L1_VHV_CONFIG__INIT, saved_vhv_init);
	}
	if (saved_vhv_timeout != 0)
	{
		writeReg(address, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, saved_vhv_timeout);
	}

	// "remove phasecal override"
	writeReg(address, VL53L1_PHASECAL_CONFIG__OVERRIDE, 0x00);

	// VL53L1_low_power_auto_data_stop_range() end
}

//对传感器进行初始化
bool VL53L1X::sensorInit(uint8_t address,bool io_2v8)
{
	setTimeout(500);//设置读取系统状态超时的时间阈值 500单位为ms
	// check model ID and module type registers (values specified in datasheet)
	//set_address(_index_counter);
	if ( readReg16Bit(address,VL53L1_IDENTIFICATION__MODEL_ID)!= 0xEACC)
		return false;
	// VL53L1_software_reset() begin
	writeReg(address,VL53L1_SOFT_RESET, 0x00);
	warnx("1:%08d",readReg(address,VL53L1_SOFT_RESET));
	usleep(100);//延时100us
	writeReg(address,VL53L1_SOFT_RESET, 0x01);
	warnx("2:%08d",readReg(address,VL53L1_SOFT_RESET));

	// VL53L1_poll_for_boot_completion() begin

	startTimeout();//获取系统当前时间
	while ((readReg(address, VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x01) == 0)
	{	//检查读取系统状态是否超时
		//checkTimeoutExpired()用于检查读取是否超时 返回ture则为超时
		if (checkTimeoutExpired())
		{
			did_timeout = true;
			PX4_ERR("false 2");//add by fxk
			return false;
		}
	}
	// VL53L1_poll_for_boot_completion() end

	// VL53L1_software_reset() end

	// VL53L1_DataInit() begin

	// sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary
	if (io_2v8)
	{
		writeReg(address, VL53L1_PAD_I2C_HV__EXTSUP_CONFIG,
		readReg(address, VL53L1_PAD_I2C_HV__EXTSUP_CONFIG) | 0x01);
	}

	// store oscillator info for later use
	fast_osc_frequency = readReg16Bit(address,VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY);
	warnx("fast_osc_frequency is :%08d",fast_osc_frequency);
	osc_calibrate_val = readReg16Bit(address,VL53L1_RESULT__OSC_CALIBRATE_VAL);
	warnx("osc_calibrate_val is:%08d",osc_calibrate_val);
	// VL53L1_DataInit() end

	// VL53L1_StaticInit() begin
	// Note that the API does not actually apply the configuration settings below
	// when VL53L1_StaticInit() is called: it keeps a copy of the sensor's
	// register contents in memory and doesn't actually write them until a
	// measurement is started. Writing the configuration here means we don't have
	// to keep it all in memory and avoids a lot of redundant writes later.

	// the API sets the preset mode to LOWPOWER_AUTONOMOUS here:
	// VL53L1_set_preset_mode() begin

	// VL53L1_preset_mode_standard_ranging() begin

	// values labeled "tuning parm default" are from vl53l1_tuning_parm_defaults.h
	// (API uses these in VL53L1_init_tuning_parm_storage_struct())

	// static config
	// API resets PAD_I2C_HV__EXTSUP_CONFIG here, but maybe we don't want to do
	// that? (seems like it would disable 2V8 mode)
	writeReg16Bit(address,VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, TargetRate); // should already be this value after reset

	writeReg(address,VL53L1_GPIO__TIO_HV_STATUS, 0x02);
	warnx("4:%08d",readReg(address,VL53L1_GPIO__TIO_HV_STATUS));
	writeReg(address,VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8); // tuning parm default
	writeReg(address,VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16); // tuning parm default
	writeReg(address,VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
	writeReg(address,VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
	writeReg(address,VL53L1_ALGO__RANGE_MIN_CLIP, 0); // tuning parm default
	writeReg(address,VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE, 0x02); // tuning parm default
	// general config
	writeReg16Bit(address,VL53L1_SYSTEM__THRESH_RATE_HIGH, 0x0000);
	writeReg16Bit(address,VL53L1_SYSTEM__THRESH_RATE_LOW, 0x0000);
	writeReg(address,VL53L1_DSS_CONFIG__APERTURE_ATTENUATION, 0x38);

	// timing config
	// most of these settings will be determined later by distance and timing
	// budget configuration
	writeReg16Bit(address,VL53L1_RANGE_CONFIG__SIGMA_THRESH, 0x168); // tuning parm default
	writeReg16Bit(address,VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 0xC0); // tuning parm default

	// dynamic config
	writeReg(address,VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
	writeReg(address,VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
	writeReg(address,VL53L1_SD_CONFIG__QUANTIFIER, 2); // tuning parm default

	 // VL53L1_preset_mode_standard_ranging() end

	// from VL53L1_preset_mode_timed_ranging_*
	// GPH is 0 after reset, but writing GPH0 and GPH1 above seem to set GPH to 1,
	// and things don't seem to work if we don't set GPH back to 0 (which the API
	// does here).
	writeReg(address,VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
	writeReg(address,VL53L1_SYSTEM__SEED_CONFIG, 1); // tuning parm default

	// from VL53L1_config_low_power_auto_mode
	writeReg(address, VL53L1_SYSTEM__SEQUENCE_CONFIG, 0x8B); // VHV, PHASECAL, DSS1, RANGE
	writeReg16Bit(address, VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
	writeReg(address, VL53L1_DSS_CONFIG__ROI_MODE_CONTROL, 2); // REQUESTED_EFFFECTIVE_SPADS

	// VL53L1_set_preset_mode() end

	// default to long range, 50 ms timing budget
	// note that this is different than what the API defaults to

	// VL53L1_StaticInit() end
	// the API triggers this change in VL53L1_init_and_start_range() once a
	// measurement is started; assumes MM1 and MM2 are disabled
	writeReg16Bit(address,VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM,
	readReg16Bit(address,VL53L1_MM_CONFIG__OUTER_OFFSET_MM) * 4);
	setDistanceMode(_index_counter,Long);
	setMeasurementTimingBudget(address, 50000);
	startContinuous(address, 50);
	return true;
}
uint16_t VL53L1X::readsensor(uint8_t address,bool blocking)
{
	if (blocking)
	{
		startTimeout();
		while (!dataReady(address))
		{//查询方式判断数据是否已经准备好？dataready=false(0)(读寄存器非零)进入循环。进入循环说明没有准备好
			if (checkTimeoutExpired())
			{   //超时则进入判断,进行赋初值
				did_timeout = true;
				ranging_data.range_status = VL53L1_RANGESTATUS_NONE;
				ranging_data.range_mm = 888;
				ranging_data.peak_signal_count_rate_MCPS = 0;
				ranging_data.ambient_count_rate_MCPS = 0;
				return ranging_data.range_mm;
			}
		}
	}

	readResults(address);

	updateDSS(address);

	getRangingData();

	writeReg(address,VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01); // sys_interrupt_clear_range

	return ranging_data.range_mm;
}


//��I2C��������ݲ�ѡ���Եؽ��д洢�ڽṹ��result��
void VL53L1X::readResults(uint8_t address)
{
	//Wire.beginTransmission(address);
	set_address(address);
	uint8_t index_array[2];
	uint8_t val[17];
	uint8_t VL53L1_RESULT__RANGE_STATUS_H=(VL53L1_RESULT__RANGE_STATUS>>8)&0xFF;
	uint8_t VL53L1_RESULT__RANGE_STATUS_L=VL53L1_RESULT__RANGE_STATUS&0xFF;
	index_array[0]=VL53L1_RESULT__RANGE_STATUS_H;
	index_array[1]=VL53L1_RESULT__RANGE_STATUS_L;
	transfer(&index_array[0], 2, nullptr, 0);

	transfer(nullptr, 0, &val[0], 17);
	results.range_status =val[0];
	results.stream_count =val[2];
	
	uint8_t dss_actual_effective_spads_sd0_h=val[3];
	uint8_t dss_actual_effective_spads_sd0_l=val[4];
	results.dss_actual_effective_spads_sd0=(dss_actual_effective_spads_sd0_h<<8)|dss_actual_effective_spads_sd0_l;

	uint8_t ambient_count_rate_mcps_sd0_h =val[7];
	uint8_t ambient_count_rate_mcps_sd0_l =val[8];
	results.ambient_count_rate_mcps_sd0=(ambient_count_rate_mcps_sd0_h<<8)|ambient_count_rate_mcps_sd0_l;

	uint8_t final_crosstalk_corrected_range_mm_sd0_h=val[13];
	uint8_t final_crosstalk_corrected_range_mm_sd0_l=val[14];
	results.final_crosstalk_corrected_range_mm_sd0=(final_crosstalk_corrected_range_mm_sd0_h<<8)|final_crosstalk_corrected_range_mm_sd0_l;

	uint8_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0_h=val[15];
	uint8_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0_l=val[16];
	results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0=(peak_signal_count_rate_crosstalk_corrected_mcps_sd0_h<<8)|peak_signal_count_rate_crosstalk_corrected_mcps_sd0_l;

}

//�Դ���õ������ݽ��д���
void VL53L1X::getRangingData()
{
	// VL53L1_copy_sys_and_core_results_to_range_results() begin
	uint16_t range = results.final_crosstalk_corrected_range_mm_sd0;

	// "apply correction gain"
	// gain factor of 2011 is tuning parm default (VL53L1_TUNINGPARM_LITE_RANGING_GAIN_FACTOR_DEFAULT)
	// Basically, this appears to scale the result by 2011/2048, or about 98%
	// (with the 1024 added for proper rounding).
	ranging_data.range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

	// VL53L1_copy_sys_and_core_results_to_range_results() end

	// set range_status in ranging_data based on value of RESULT__RANGE_STATUS register
	// mostly based on ConvertStatusLite()
	switch (results.range_status)
	{
	case 17: // MULTCLIPFAIL
	case 2: // VCSELWATCHDOGTESTFAILURE
	case 1: // VCSELCONTINUITYTESTFAILURE
	case 3: // NOVHVVALUEFOUND
			// from SetSimpleData()
		ranging_data.range_status = VL53L1_RANGESTATUS_HARDWARE_FAIL;
		break;

	case 13: // USERROICLIP
			 // from SetSimpleData()
		ranging_data.range_status = VL53L1_RANGESTATUS_MIN_RANGE_FAIL;
		break;

	case 18: // GPHSTREAMCOUNT0READY
		ranging_data.range_status = VL53L1_RANGESTATUS_SYNCRONISATION_INT;
		break;

	case 5: // RANGEPHASECHECK
		ranging_data.range_status = VL53L1_RANGESTATUS_OUTOFBOUNDS_FAIL;
		break;

	case 4: // MSRCNOTARGET
		ranging_data.range_status = VL53L1_RANGESTATUS_SIGNAL_FAIL;
		break;

	case 6: // SIGMATHRESHOLDCHECK
		ranging_data.range_status = VL53L1_RANGESTATUS_SIGMA_FAIL;
		break;

	case 7: // PHASECONSISTENCY
		ranging_data.range_status = VL53L1_RANGESTATUS_WRAP_TARGET_FAIL;
		break;

	case 12: // RANGEIGNORETHRESHOLD
		ranging_data.range_status = VL53L1_RANGESTATUS_XTALK_SIGNAL_FAIL;
		break;

	case 8: // MINCLIP
		ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID_MIN_RANGE_CLIPPED;
		break;

	case 9: // RANGECOMPLETE
			// from VL53L1_copy_sys_and_core_results_to_range_results()
		if (results.stream_count == 0)
		{
			ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID_NO_WRAP_CHECK_FAIL;
		}
		else
		{
			ranging_data.range_status = VL53L1_RANGESTATUS_RANGE_VALID;
		}
		break;

	default:
		ranging_data.range_status = VL53L1_RANGESTATUS_NONE;
	}

	// from SetSimpleData()
	ranging_data.peak_signal_count_rate_MCPS =
		countRateFixedToFloat(results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
	ranging_data.ambient_count_rate_MCPS =
		countRateFixedToFloat(results.ambient_count_rate_mcps_sd0);
}

//�Խ�����д���������ɾȥ����
void VL53L1X::updateDSS(uint8_t address)
{
	uint16_t spadCount = results.dss_actual_effective_spads_sd0;

	if (spadCount != 0)
	{
		// "Calc total rate per spad"

		uint32_t totalRatePerSpad =
			(uint32_t)results.peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +
			results.ambient_count_rate_mcps_sd0;

		// "clip to 16 bits"
		if (totalRatePerSpad > 0xFFFF) { totalRatePerSpad = 0xFFFF; }

		// "shift up to take advantage of 32 bits"????
		totalRatePerSpad <<= 16;

		totalRatePerSpad /= spadCount;

		if (totalRatePerSpad != 0)
		{
			// "get the target rate and shift up by 16"
			uint32_t requiredSpads = ((uint32_t)TargetRate << 16) / totalRatePerSpad;

			// "clip to 16 bit"
			if (requiredSpads > 0xFFFF) { requiredSpads = 0xFFFF; }

			// "override DSS config"
			writeReg16Bit(address,VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);
			// DSS_CONFIG__ROI_MODE_CONTROL should already be set to REQUESTED_EFFFECTIVE_SPADS

			return;
		}
	}

	// If we reached this point, it means something above would have resulted in a
	// divide by zero.
	// "We want to gracefully set a spad target, not just exit with an error"

	// "set target to mid point"
	writeReg16Bit(address,VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

bool VL53L1X::setDistanceMode(uint8_t address, DistanceMode mode)
{
	// save existing timing budget
	uint32_t budget_us = getMeasurementTimingBudget(address);

	switch (mode)
	{
	case Short:
		// from VL53L1_preset_mode_standard_ranging_short_range()

		// timing config
		writeReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
		writeReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
		writeReg(address, VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);

		// dynamic config
		writeReg(address, VL53L1_SD_CONFIG__WOI_SD0, 0x07);
		writeReg(address, VL53L1_SD_CONFIG__WOI_SD1, 0x05);
		writeReg(address, VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 6); // tuning parm default
		writeReg(address, VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 6); // tuning parm default

		break;

	case Medium:
		// from VL53L1_preset_mode_standard_ranging()

		// timing config
		writeReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
		writeReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
		writeReg(address, VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);

		// dynamic config
		writeReg(address, VL53L1_SD_CONFIG__WOI_SD0, 0x0B);
		writeReg(address, VL53L1_SD_CONFIG__WOI_SD1, 0x09);
		writeReg(address, VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 10); // tuning parm default
		writeReg(address, VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 10); // tuning parm default

		break;

	case Long: // long
			   // from VL53L1_preset_mode_standard_ranging_long_range()

			   // timing config
		writeReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
		writeReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
		writeReg(address, VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);

		// dynamic config
		writeReg(address, VL53L1_SD_CONFIG__WOI_SD0, 0x0F);
		writeReg(address, VL53L1_SD_CONFIG__WOI_SD1, 0x0D);
		writeReg(address, VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 14); // tuning parm default
		writeReg(address, VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 14); // tuning parm default

		break;

	default:
		// unrecognized mode - do nothing
		return false;
	}
	setMeasurementTimingBudget(address,budget_us);
	 distance_mode = mode;
	  return true;
}

bool VL53L1X::setMeasurementTimingBudget(uint8_t address, uint32_t budget_us)
{
	// assumes PresetMode is LOWPOWER_AUTONOMOUS

	if (budget_us <= TimingGuard) { return false; }

	uint32_t range_config_timeout_us = budget_us -= TimingGuard;
	if (range_config_timeout_us > 1100000) { return false; } // FDA_MAX_TIMING_BUDGET_US * 2

	range_config_timeout_us /= 2;

	// VL53L1_calc_timeout_register_values() begin

	uint32_t macro_period_us;

	// "Update Macro Period for Range A VCSEL Period"
	macro_period_us = calcMacroPeriod(readReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));

	// "Update Phase timeout - uses Timing A"
	// Timeout of 1000 is tuning parm default (TIMED_PHASECAL_CONFIG_TIMEOUT_US_DEFAULT)
	// via VL53L1_get_preset_mode_timing_cfg().
	uint32_t phasecal_timeout_mclks = timeoutMicrosecondsToMclks(1000, macro_period_us);
	if (phasecal_timeout_mclks > 0xFF) { phasecal_timeout_mclks = 0xFF; }
	writeReg(address, VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);

	// "Update MM Timing A timeout"
	// Timeout of 1 is tuning parm default (LOWPOWERAUTO_MM_CONFIG_TIMEOUT_US_DEFAULT)
	// via VL53L1_get_preset_mode_timing_cfg(). With the API, the register
	// actually ends up with a slightly different value because it gets assigned,
	// retrieved, recalculated with a different macro period, and reassigned,
	// but it probably doesn't matter because it seems like the MM ("mode
	// mitigation"?) sequence steps are disabled in low power auto mode anyway.
	writeReg16Bit(address, VL53L1_MM_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
		timeoutMicrosecondsToMclks(1, macro_period_us)));

	// "Update Range Timing A timeout"
	writeReg16Bit(address, VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A, encodeTimeout(
		timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

	// "Update Macro Period for Range B VCSEL Period"
	macro_period_us = calcMacroPeriod(readReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B));

	// "Update MM Timing B timeout"
	// (See earlier comment about MM Timing A timeout.)
	writeReg16Bit(address, VL53L1_MM_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
		timeoutMicrosecondsToMclks(1, macro_period_us)));

	// "Update Range Timing B timeout"
	writeReg16Bit(address, VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B, encodeTimeout(
		timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

	// VL53L1_calc_timeout_register_values() end

	return true;
}

uint32_t VL53L1X::getMeasurementTimingBudget(uint8_t address)
{
	// assumes PresetMode is LOWPOWER_AUTONOMOUS and these sequence steps are
	// enabled: VHV, PHASECAL, DSS1, RANGE

	// VL53L1_get_timeouts_us() begin

	// "Update Macro Period for Range A VCSEL Period"
	uint32_t macro_period_us = calcMacroPeriod(readReg(address, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));

	// "Get Range Timing A timeout"

	uint32_t range_config_timeout_us = timeoutMclksToMicroseconds(decodeTimeout(
		readReg16Bit(address, VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A)), macro_period_us);

	// VL53L1_get_timeouts_us() end

	return  2 * range_config_timeout_us + TimingGuard;
}

uint32_t VL53L1X::calcMacroPeriod(uint8_t vcsel_period)
{
	// from VL53L1_calc_pll_period_us()
	// fast osc frequency in 4.12 format; PLL period in 0.24 format
	uint32_t pll_period_us = ((uint32_t)0x01 << 30) / fast_osc_frequency;

	// from VL53L1_decode_vcsel_period()
	uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;

	// VL53L1_MACRO_PERIOD_VCSEL_PERIODS = 2304
	uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
	macro_period_us >>= 6;
	macro_period_us *= vcsel_period_pclks;
	macro_period_us >>= 6;

	return macro_period_us;
}

uint32_t VL53L1X::timeoutMicrosecondsToMclks(uint32_t timeout_us,uint32_t macro_period_us)
{
	return (((uint32_t)timeout_us<<12)+(macro_period_us>>1))/macro_period_us;
}

uint32_t VL53L1X::timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}

uint16_t VL53L1X::encodeTimeout(uint32_t timeout_mclks)
{
  // encoded format: "(LSByte * 2^MSByte) + 1"

  uint32_t ls_byte = 0;
  uint16_t ms_byte = 0;

  if (timeout_mclks > 0)
  {
    ls_byte = timeout_mclks - 1;

    while ((ls_byte & 0xFFFFFF00) > 0)
    {
      ls_byte >>= 1;
      ms_byte++;
    }

    return (ms_byte << 8) | (ls_byte & 0xFF);
  }
  else { return 0; }
}

uint32_t VL53L1X::decodeTimeout(uint16_t reg_val)
{
  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;
}


void VL53L1X::cycle()
{
	collect();
	work_queue(LPWORK,
		&_work,
		(worker_t)&VL53L1X::cycle_trampoline,
		this,
		_measure_ticks);
}

void VL53L1X::cycle_trampoline(void *arg)
{
	VL53L1X *dev = (VL53L1X *)arg;

	dev->cycle();
}

void VL53L1X::stop()
{
	work_cancel(LPWORK, &_work);
}

void VL53L1X::start()
{
	/* reset the report ring and state machine */
	_reports->flush();

	/* schedule a cycle to start things */
	work_queue(LPWORK, &_work, (worker_t)&VL53L1X::cycle_trampoline, this, USEC2TICK(VL53L1X_US));
	
}


int VL53L1X::collect()
{
	int ret = -EIO;
	readsensor(_index_counter, false);
	
	report.timestamp = hrt_absolute_time();
	report.type = distance_sensor_s::MAV_DISTANCE_SENSOR_LASER;
	//report.orientation = _rotation;

	report.current_distance = ranging_data.range_mm;
	report.min_distance = 0.0f;
	report.max_distance = 4.0f;
	report.covariance = 0.0f;
	//report.signal_quality = -1;

	/* TODO: set proper ID */
	report.id = 0;

	/* publish it, if we are the primary */
	if (_distance_sensor_pub != nullptr) {
		orb_publish(ORB_ID(distance_sensor), _distance_sensor_pub, &report);
	}
	else
		_distance_sensor_pub = orb_advertise(ORB_ID(distance_sensor), &report);

	_reports->force(&report);

	/* notify anyone waiting for data */
	poll_notify(POLLIN);

	ret = OK;
	//perf_end(_sample_perf);

	return ret;
}

/**
* Local functions in support of the shell command.
*/
namespace vl53l1x
{

	VL53L1X	*g_dev;

	int 	start();
	int 	start_bus();
	int 	stop();
	int 	test();
	int 	info();

	/**
	*
	* Attempt to start driver on all available I2C busses.
	*
	* This function will return as soon as the first sensor
	* is detected on one of the available busses or if no
	* sensors are detected.
	*
	*/
	int
		start()
	{
		if (g_dev != nullptr) {
			PX4_ERR("already started");
			return PX4_ERROR;
		}
//		else
//			PX4_ERR("1_g_dev is nullptr!!");
		//for (unsigned i = 0; i < NUM_I2C_BUS_OPTIONS; i++) {
			if (start_bus() == PX4_OK)
			{
				return PX4_OK;
				PX4_INFO("start_bus success!!");
				PX4_ERR("g_dev start!!!!!");
			}
			else
				PX4_ERR("start_bus fail!!");
		//}

		return PX4_ERROR;
	}

	/**
	* Start the driver on a specific bus.
	*
	* This function only returns if the sensor is up and running
	* or could not be detected successfully.
	*/
	int
		start_bus( )
	{
		int fd = -1;

		if (g_dev != nullptr) {
			PX4_ERR("already started");
			return PX4_ERROR;
		}
//		else
//			PX4_ERR("2_g_dev is nullptr!!");

		/* create the driver */
		g_dev = new VL53L1X(PX4_I2C_BUS_EXPANSION,VL53L1X_BASEADDR);

		if (g_dev == nullptr) {
			PX4_ERR("g_dev creat fail!!");
			goto fail;
		}

		if (OK != g_dev->init()) {
			PX4_ERR("g_dev init fail!!");
			goto fail;
		}

		/* set the poll rate to default, starts automatic data collection */
		fd = px4_open(VL53L1X_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("fd open fail!!");
			goto fail;
		}

		if (ioctl(fd, SENSORIOCSPOLLRATE, SENSOR_POLLRATE_DEFAULT) < 0) {
			PX4_ERR("g_dev ioctl fail!!");
			goto fail;
		}

		px4_close(fd);
		return PX4_OK;

	fail:

		if (fd >= 0) {
			px4_close(fd);
		}

		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;
		}

		return PX4_ERROR;
	}

	/**
	* Stop the driver
	*/
	int
		stop()
	{
		if (g_dev != nullptr) {
			delete g_dev;
			g_dev = nullptr;

		}
		else {
			PX4_ERR("driver not running");
			return PX4_ERROR;
		}

		return PX4_OK;
	}

	/**
	* Perform some basic functional tests on the driver;
	* make sure we can collect data from the sensor in polled
	* and automatic modes.
	*/
	int
		test()
	{
		struct distance_sensor_s report;
		ssize_t sz;

		int fd = px4_open(VL53L1X_DEVICE_PATH, O_RDONLY);

		if (fd < 0) {
			PX4_ERR("%s open failed (try 'vl53lxx start' if the driver is not running)", VL53L1X_DEVICE_PATH);
			return PX4_ERROR;
		}

		/* do a simple demand read */
		sz = read(fd, &report, sizeof(report));

		if (sz != sizeof(report)) {
			PX4_ERR("immediate read failed");
			return PX4_ERROR;
		}

		//print_message(report);

		px4_close(fd);

		PX4_INFO("PASS");
		return PX4_OK;
	}


	/**
	* Print a little info about the driver.
	*/
	int info()
	{
		if (g_dev == nullptr) {
			PX4_ERR("driver not running");
			return PX4_ERROR;
		}

		printf("state @ %p\n", g_dev);
		g_dev->print_info();
		struct distance_sensor_s distance;
		memset(&distance, 0, sizeof(distance));
		int distance_sensor_sub;
		distance_sensor_sub = orb_subscribe(ORB_ID(distance_sensor));
		while(1)
		{
			orb_copy(ORB_ID(distance_sensor), distance_sensor_sub, &distance);
			warnx("current distance = %.3fm",(double)distance.current_distance/1000);
			warnx("max distance = %.2f\tmin distance = %.2f",(double)distance.max_distance,(double)distance.min_distance);
			//warnx("covariance = %f",(double)distance.covariance);
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
			          usleep(800000);
		}

		return PX4_OK;
	}

} // namespace vl53l1x


static void
vl53l1x_usage()
{
	PX4_INFO("usage: vl53lxx command");
	//PX4_INFO("options:");
	//PX4_INFO("\t-b --bus i2cbus (%d)", VL53L1X_BUS_DEFAULT);
	//PX4_INFO("\t-a --all");
	//PX4_INFO("\t-R --rotation (%d)", distance_sensor_s::ROTATION_DOWNWARD_FACING);
	PX4_INFO("command:");
	PX4_INFO("start|stop|test|info");
}


int
vl53l1x_main(int argc, char *argv[])
{
	int ch;
	int myoptind = 1;
	const char *myoptarg = nullptr;
	//uint8_t rotation = distance_sensor_s::ROTATION_DOWNWARD_FACING;
	bool start_all = true;

	//int i2c_bus = VL53L1X_BUS_DEFAULT;

	while ((ch = px4_getopt(argc, argv, "ab:R:", &myoptind, &myoptarg)) != EOF) {
		switch (ch) {
//		case 'R':
//			rotation = (uint8_t)atoi(myoptarg);
//			break;
//
//		case 'b':
//			i2c_bus = atoi(myoptarg);
//			break;
//
//		case 'a':
//			start_all = true;
//			break;

		default:
			PX4_WARN("Unknown option!");
			goto out_error;
		}
	}

	/*
	* Start/load the driver.
	*/
	if (!strcmp(argv[myoptind], "start")) {
		if (start_all) {
			return vl53l1x::start();

		}
		else {
			return vl53l1x::start_bus();
		}
	}

	/*
	* Stop the driver
	*/
	if (!strcmp(argv[myoptind], "stop")) {
		return vl53l1x::stop();
	}

	/*
	* Test the driver/device.
	*/
	if (!strcmp(argv[myoptind], "test")) {
		return vl53l1x::test();
	}

	/*
	* Print driver information.
	*/
	if (!strcmp(argv[myoptind], "info") || !strcmp(argv[myoptind], "status")) {
		return vl53l1x::info();
	}

out_error:

	vl53l1x_usage();
	return PX4_ERROR;
}
