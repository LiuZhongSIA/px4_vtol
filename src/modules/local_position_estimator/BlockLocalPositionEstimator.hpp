#pragma once

#include <px4_posix.h>
#include <controllib/blocks.hpp>
#include <mathlib/mathlib.h>
#include <systemlib/perf_counter.h>
#include <lib/geo/geo.h>
#include <matrix/Matrix.hpp>

// uORB Subscriptions
#include <uORB/Subscription.hpp>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/actuator_armed.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_control_mode.h>
#include <uORB/topics/vehicle_attitude.h>
#include <uORB/topics/vehicle_attitude_setpoint.h>
#include <uORB/topics/optical_flow.h>
#include <uORB/topics/sensor_combined.h>
#include <uORB/topics/distance_sensor.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/vehicle_gps_position.h>
#include <uORB/topics/vision_position_estimate.h>
#include <uORB/topics/att_pos_mocap.h>

// uORB Publications
#include <uORB/Publication.hpp>
#include <uORB/topics/vehicle_local_position.h>
#include <uORB/topics/vehicle_global_position.h>
#include <uORB/topics/estimator_status.h>
#include <uORB/topics/ekf2_innovations.h> //创新

using namespace matrix;
using namespace control;

static const float DELAY_MAX = 0.5f; // seconds
static const float HIST_STEP = 0.05f; // 20 hz
static const float BIAS_MAX = 1e-1f;
static const size_t HIST_LEN = 10; // DELAY_MAX / HIST_STEP;
static const size_t N_DIST_SUBS = 4;

enum fault_t {
	FAULT_NONE = 0,
	FAULT_MINOR, //较小的
	FAULT_SEVERE //严重的
};

enum sensor_t {
	SENSOR_BARO = 0, //气压计
	SENSOR_GPS,      //GPS
	SENSOR_LIDAR,    //激光雷达
	SENSOR_FLOW,     //光流
	SENSOR_SONAR,    //超声波
	SENSOR_VISION,   //视觉
	SENSOR_MOCAP,    //动捕
	SENSOR_LAND,     //地面
};

// change this to set when
// the system will abort correcting a measurement
// given a fault has been detected
// 当传感器故障不是严重故障时，才会用传感器信息进行测量更新
static const fault_t fault_lvl_disable = FAULT_SEVERE;
// for fault detection
// chi squared distribution, false alarm probability 0.0001
// see fault_table.py
// note skip 0 index so we can use degree of freedom as index
// 设置的故障诊断阈值
static const float BETA_TABLE[7] = {0,
				    8.82050518214,
				    12.094592431,
				    13.9876612368,
				    16.0875642296,
				    17.8797700658,
				    19.6465647819,
				   };

// 构造函数
class BlockLocalPositionEstimator : public control::SuperBlock
{
// dynamics:
//
//	x(+) = A * x(-) + B * u(+)
//	y_i = C_i*x
//
// kalman filter
//
//	E[xx'] = P
//	E[uu'] = W
//	E[y_iy_i'] = R_i
//
//	prediction
//		x(+|-) = A*x(-|-) + B*u(+)
//		P(+|-) = A*P(-|-)*A' + B*W*B'
//
//	correction
//		x(+|+) =  x(+|-) + K_i * (y_i - H_i * x(+|-) )
//
//
// input:  //加速度计的测量作为输入
// 	ax, ay, az (acceleration NED)
//
// states:
// 	px, py, pz , ( position NED, m)
// 	vx, vy, vz ( vel NED, m/s),
// 	bx, by, bz ( accel bias, m/s^2)  //同时估计加速度偏差
// 	tz (terrain altitude, ASL, m) //地形高度
//
// measurements: //不同的机载传感器的测量进行状态更新，数据融合
//
// 	sonar: pz (measured d*cos(phi)*cos(theta))
//
// 	baro: pz
//
// 	flow: vx, vy (flow is in body x, y frame)
//
// 	gps: px, py, pz, vx, vy, vz (flow is in body x, y frame)
//
// 	lidar: pz (actual measured d*cos(phi)*cos(theta))
//
// 	vision: px, py, pz, vx, vy, vz
//
// 	mocap: px, py, pz
//
// 	land (detects when landed)): pz (always measures agl = 0)
//
public:

	// constants
	// 设定了不同向量的维数
	enum {X_x = 0, X_y, X_z, X_vx, X_vy, X_vz, X_bx, X_by, X_bz, X_tz, n_x};
	enum {U_ax = 0, U_ay, U_az, n_u};
	enum {Y_baro_z = 0, n_y_baro};
	enum {Y_lidar_z = 0, n_y_lidar};
	enum {Y_flow_vx = 0, Y_flow_vy, n_y_flow};
	enum {Y_sonar_z = 0, n_y_sonar};
	enum {Y_gps_x = 0, Y_gps_y, Y_gps_z, Y_gps_vx, Y_gps_vy, Y_gps_vz, n_y_gps};
	enum {Y_vision_x = 0, Y_vision_y, Y_vision_z, n_y_vision};
	enum {Y_mocap_x = 0, Y_mocap_y, Y_mocap_z, n_y_mocap};
	enum {Y_land_vx, Y_land_vy, Y_land_agl = 0, n_y_land};
	enum {POLL_FLOW, POLL_SENSORS, POLL_PARAM, n_poll};

	BlockLocalPositionEstimator();
	void update();
	Vector<float, n_x> dynamics(
		float t,
		const Vector<float, n_x> &x,
		const Vector<float, n_u> &u);
	virtual ~BlockLocalPositionEstimator();

private:
	// prevent copy and assignment
	BlockLocalPositionEstimator(const BlockLocalPositionEstimator &);
	BlockLocalPositionEstimator operator=(const BlockLocalPositionEstimator &);

	// methods
	// ----------------------------
	void initP();
	void initSS();
	void updateSSStates();
	void updateSSParams();

	// predict the next state
	void predict();

	// lidar  1
	int  lidarMeasure(Vector<float, n_y_lidar> &y); //测量、修正、初始化、检查超时
	void lidarCorrect();
	void lidarInit();
	void lidarCheckTimeout();

	// sonar  1
	int  sonarMeasure(Vector<float, n_y_sonar> &y);
	void sonarCorrect();
	void sonarInit();
	void sonarCheckTimeout();

	// baro  1
	int  baroMeasure(Vector<float, n_y_baro> &y);
	void baroCorrect();
	void baroInit();
	void baroCheckTimeout();

	// gps  6
	int  gpsMeasure(Vector<double, n_y_gps> &y);
	void gpsCorrect();
	void gpsInit();
	void gpsCheckTimeout();

	// flow  2
	int  flowMeasure(Vector<float, n_y_flow> &y);
	void flowCorrect();
	void flowInit();
	void flowDeinit();
	void flowCheckTimeout();

	// vision  3
	int  visionMeasure(Vector<float, n_y_vision> &y);
	void visionCorrect();
	void visionInit();
	void visionCheckTimeout();

	// mocap  3
	int  mocapMeasure(Vector<float, n_y_mocap> &y);
	void mocapCorrect();
	void mocapInit();
	void mocapCheckTimeout();

	// land  1
	int  landMeasure(Vector<float, n_y_land> &y);
	void landCorrect();
	void landInit();
	void landCheckTimeout();

	// timeouts
	void checkTimeouts();

	// misc 混杂的
	float agl();
	void correctionLogic(Vector<float, n_x> &dx);           //n_x=10
	void covPropagationLogic(Matrix<float, n_x, n_x> &dP);
	void detectDistanceSensors();
	int getDelayPeriods(float delay, uint8_t *periods);

	// publications
	void publishLocalPos();
	void publishGlobalPos();
	void publishEstimatorStatus();

	// attributes
	// ----------------------------

	// subscriptions
	// 算上land，8个传感器的测量
	uORB::Subscription<actuator_armed_s> _sub_armed;
	uORB::Subscription<vehicle_land_detected_s> _sub_land; //
	uORB::Subscription<vehicle_attitude_s> _sub_att;
	uORB::Subscription<parameter_update_s> _sub_param_update;
	uORB::Subscription<manual_control_setpoint_s> _sub_manual;
	uORB::Subscription<optical_flow_s> _sub_flow; //
	uORB::Subscription<sensor_combined_s> _sub_sensor; //
	uORB::Subscription<vehicle_gps_position_s> _sub_gps; //
	uORB::Subscription<vision_position_estimate_s> _sub_vision_pos; //
	uORB::Subscription<att_pos_mocap_s> _sub_mocap; //
	uORB::Subscription<distance_sensor_s> _sub_dist0; //距离传感器
	uORB::Subscription<distance_sensor_s> _sub_dist1;
	uORB::Subscription<distance_sensor_s> _sub_dist2;
	uORB::Subscription<distance_sensor_s> _sub_dist3;
	uORB::Subscription<distance_sensor_s> *_dist_subs[N_DIST_SUBS];
	uORB::Subscription<distance_sensor_s> *_sub_lidar; //
	uORB::Subscription<distance_sensor_s> *_sub_sonar; //

	// publications
	uORB::Publication<vehicle_local_position_s> _pub_lpos;
	uORB::Publication<vehicle_global_position_s> _pub_gpos;
	uORB::Publication<estimator_status_s> _pub_est_status;
	uORB::Publication<ekf2_innovations_s> _pub_innov;

	// map projection
	struct map_projection_reference_s _map_ref;

	// general parameters
	BlockParamInt  _pub_agl_z;
	BlockParamFloat  _vxy_pub_thresh;
	BlockParamFloat  _z_pub_thresh;

	// sonar parameters
	BlockParamFloat  _sonar_z_stddev; //标准偏差
	BlockParamFloat  _sonar_z_offset; //补偿

	// lidar parameters
	BlockParamFloat  _lidar_z_stddev;
	BlockParamFloat  _lidar_z_offset;

	// accel parameters
	BlockParamFloat  _accel_xy_stddev;
	BlockParamFloat  _accel_z_stddev;

	// baro parameters
	BlockParamFloat  _baro_stddev;

	// gps parameters
	BlockParamInt   _gps_on;
	BlockParamFloat  _gps_delay;
	BlockParamFloat  _gps_xy_stddev;
	BlockParamFloat  _gps_z_stddev;
	BlockParamFloat  _gps_vxy_stddev;
	BlockParamFloat  _gps_vz_stddev;
	BlockParamFloat  _gps_eph_max;
	BlockParamFloat  _gps_epv_max;

	// vision parameters
	BlockParamFloat  _vision_xy_stddev;
	BlockParamFloat  _vision_z_stddev;
	BlockParamFloat  _vision_delay;
	BlockParamInt   _vision_on;

	// mocap parameters
	BlockParamFloat  _mocap_p_stddev;

	// flow parameters
	BlockParamInt  _flow_gyro_comp;
	BlockParamFloat  _flow_z_offset;
	BlockParamFloat  _flow_scale;
	//BlockParamFloat  _flow_board_x_offs;
	//BlockParamFloat  _flow_board_y_offs;
	BlockParamInt    _flow_min_q;

	// land parameters
	BlockParamFloat  _land_z_stddev;

	// process noise
	// 过程噪声
	BlockParamFloat  _pn_p_noise_density;
	BlockParamFloat  _pn_v_noise_density;
	BlockParamFloat  _pn_b_noise_density;
	BlockParamFloat  _pn_t_noise_density;
	BlockParamFloat  _t_max_grade;

	// init origin
	// 初始来源
	BlockParamFloat  _init_origin_lat;
	BlockParamFloat  _init_origin_lon;

	// flow gyro filter
	// 高通滤波器
	BlockHighPass _flow_gyro_x_high_pass;
	BlockHighPass _flow_gyro_y_high_pass;

	// stats
	BlockStats<float, n_y_baro> _baroStats;
	BlockStats<float, n_y_sonar> _sonarStats;
	BlockStats<float, n_y_lidar> _lidarStats;
	BlockStats<float, 1> _flowQStats;
	BlockStats<float, n_y_vision> _visionStats;
	BlockStats<float, n_y_mocap> _mocapStats;
	BlockStats<double, n_y_gps> _gpsStats;
	uint16_t _landCount;

	// low pass
	// 状态的低通滤波器
	BlockLowPassVector<float, n_x> _xLowPass;
	BlockLowPass _aglLowPass;

	// delay blocks
	BlockDelay<float, n_x, 1, HIST_LEN> _xDelay;
	BlockDelay<uint64_t, 1, 1, HIST_LEN> _tDelay;

	// misc
	px4_pollfd_struct_t _polls[3];
	uint64_t _timeStamp;
	uint64_t _time_last_hist;
	uint64_t _time_last_xy;
	uint64_t _time_last_z;
	uint64_t _time_last_tz;
	uint64_t _time_last_flow;
	uint64_t _time_last_baro;
	uint64_t _time_last_gps;
	uint64_t _time_last_lidar;
	uint64_t _time_last_sonar;
	uint64_t _time_init_sonar;
	uint64_t _time_last_vision_p;
	uint64_t _time_last_mocap;
	uint64_t _time_last_land;

	// initialization flags
	bool _receivedGps;
	bool _baroInitialized;
	bool _gpsInitialized;
	bool _lidarInitialized;
	bool _sonarInitialized;
	bool _flowInitialized;
	bool _visionInitialized;
	bool _mocapInitialized;
	bool _landInitialized;

	// reference altitudes
	float _altOrigin;
	bool _altOriginInitialized;
	float _baroAltOrigin;
	float _gpsAltOrigin;

	// status
	bool _validXY;
	bool _validZ;
	bool _validTZ;
	bool _xyTimeout;
	bool _zTimeout;
	bool _tzTimeout;
	bool _lastArmedState;

	// sensor faults
	// 作为标志位，没有实际的控制作用
	fault_t _baroFault;
	fault_t _gpsFault;
	fault_t _lidarFault;
	fault_t _flowFault;
	fault_t _sonarFault;
	fault_t _visionFault;
	fault_t _mocapFault;
	fault_t _landFault;

	// performance counters
	// 性能计数器
	perf_counter_t _loop_perf;
	perf_counter_t _interval_perf;
	perf_counter_t _err_perf;

	// state space
	Vector<float, n_x>  _x; // state vector
	Vector<float, n_u>  _u; // input vector
	Matrix<float, n_x, n_x>  _P; // state covariance matrix

	matrix::Dcm<float> _R_att;
	Vector3f _eul;

	Matrix<float, n_x, n_x>  _A; // dynamics matrix
	Matrix<float, n_x, n_u>  _B; // input matrix
	Matrix<float, n_u, n_u>  _R; // input covariance
	Matrix<float, n_x, n_x>  _Q; // process noise covariance
};
