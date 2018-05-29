#include "../BlockLocalPositionEstimator.hpp"
#include <systemlib/mavlink_log.h>
#include <matrix/math.hpp>

extern orb_advert_t mavlink_log_pub;

// required number of samples for sensor
// to initialize
static const uint32_t 		REQ_BARO_INIT_COUNT = 100;
static const uint32_t 		BARO_TIMEOUT =     100000;	// 0.1 s

// 初始化气压计
void BlockLocalPositionEstimator::baroInit()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) {
		_baroStats.reset();
		return;
	}

	// if finished
	// 统计气压计的连续更新次数，如果一开始连续更新次数大于100,说明初始化成功
	if (_baroStats.getCount() > REQ_BARO_INIT_COUNT) {
		_baroAltOrigin = _baroStats.getMean()(0); //一开始统计的平局值，作为初始化的高度
		mavlink_and_console_log_info(&mavlink_log_pub,
					     "[lpe] baro init %d m std %d cm",
					     (int)_baroStats.getMean()(0),
					     (int)(100 * _baroStats.getStdDev()(0)));
		_baroInitialized = true;
		_baroFault = FAULT_NONE;

		if (!_altOriginInitialized) { //利用气压计进行了高度初始化
			_altOriginInitialized = true;
			_altOrigin = _baroAltOrigin;
		}
	}
}

// 获得气压计的测量高度
int BlockLocalPositionEstimator::baroMeasure(Vector<float, n_y_baro> &y)
{
	//measure
	y.setZero();
	y(0) = _sub_sensor.get().baro_alt_meter;
	_baroStats.update(y);
	_time_last_baro = _timeStamp;
	return OK;
}

// Kalman滤波的测量更新
void BlockLocalPositionEstimator::baroCorrect()
{
	// measure
	Vector<float, n_y_baro> y;

	if (baroMeasure(y) != OK) { return; }

	// subtract baro origin alt
	// 气压计的测量高度，默认使用气压计进行高度初始化
	y -= _baroAltOrigin;

	// baro measurement matrix
	// 气压计的测量矩阵，线性形式
	Matrix<float, n_y_baro, n_x> C;
	C.setZero();
	C(Y_baro_z, X_z) = -1; // measured altitude, negative down dir.

	// 气压计的测量噪声协方差阵
	Matrix<float, n_y_baro, n_y_baro> R;
	R.setZero();
	R(0, 0) = _baro_stddev.get() * _baro_stddev.get();

	// residual
	Matrix<float, n_y_baro, n_y_baro> S_I =
		inv<float, n_y_baro>((C * _P * C.transpose()) + R);
	Vector<float, n_y_baro> r = y - (C * _x);

	// fault detection
	float beta = (r.transpose() * (S_I * r))(0, 0);
	if (beta > BETA_TABLE[n_y_baro]) {
		if (_baroFault < FAULT_MINOR) {
			if (beta > 2.0f * BETA_TABLE[n_y_baro]) {
				mavlink_log_critical(&mavlink_log_pub, "[lpe] baro fault, r %5.2f m, beta %5.2f",
						     double(r(0)), double(beta));
			}
			_baroFault = FAULT_MINOR;
		}
	} else if (_baroFault) {
		_baroFault = FAULT_NONE;
		//mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] baro OK");
	}

	// kalman filter correction if no fault
	if (_baroFault < fault_lvl_disable) { //传感器不发生严重故障时，才进行测量更新
		Matrix<float, n_x, n_y_baro> K = _P * C.transpose() * S_I;
		Vector<float, n_x> dx = K * r;
		correctionLogic(dx);
		_x += dx;
		_P -= K * C * _P;
	}
}

// 如果测量超时，也就是如果中间存在多个周期没有获得气压计信号
void BlockLocalPositionEstimator::baroCheckTimeout()
{
	if (_timeStamp - _time_last_baro > BARO_TIMEOUT) {
		if (_baroInitialized) {
			_baroInitialized = false;
			_baroStats.reset(); //气压计状态会重置
			mavlink_and_console_log_info(&mavlink_log_pub, "[lpe] baro timeout ");
		}
	}
}
