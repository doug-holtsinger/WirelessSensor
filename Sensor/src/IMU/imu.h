/**
 * @brief Header file for IMU
 *
 */


#ifndef __IMU_H__
#define __IMU_H__

#include "TwoWire.h"
#include "LSM6DS3Sensor.h"
#include "LIS3MDLSensor.h"
#include "imu_cal.h"
#include "imu_cmd.h"
#include "param_store.h"
#include "AHRS.h"
#include "logdef.h"

// FIXME: remove constants
// #define ACCELEROMETER_MIN_THRESHOLD 100
// #define GYROSCOPE_MIN_THRESHOLD 0.05
// #define MAGNETOMETER_MIN_THRESHOLD 15
// #define GYROSCOPE_SENSITIVITY_THRESHOLD 16
#define NOISE_THRESHOLD_MULTIPLIER 2

#define DEGREES_PER_RADIAN 57.2957795
#define MILLIDEGREES_PER_DEGREE 1000
#define SECONDS_PER_TIMER_TICK 0.000025
#define TIMER_TICKS_PER_SECOND 40000

typedef enum
{
    IMU_AHRS = 0,
    IMU_SENSOR_MIN = IMU_AHRS,
    IMU_ACCELEROMETER,
    IMU_GYROSCOPE,
    IMU_MAGNETOMETER,
    IMU_SENSOR_MAX = IMU_MAGNETOMETER
} IMU_SENSOR_t;

typedef enum {
        IMU_CALIBRATE_DISABLED = 0,
        IMU_CALIBRATE_MIN = IMU_CALIBRATE_DISABLED,
        IMU_CALIBRATE_ZERO_OFFSET,
        IMU_CALIBRATE_MAGNETOMETER,
        IMU_CALIBRATE_GYROSCOPE,
        IMU_CALIBRATE_MAX = IMU_CALIBRATE_MAGNETOMETER
} IMU_CALIBRATE_t;

typedef enum {
	AHRS_MAHONY,
	AHRS_MADGWICH
} AHRS_ALGORITHM_t;

class IMU {
    public:
        IMU();
        ~IMU();
        void update(void);
        void init(void);
        void get_params(imu_calibration_params_t& params);
        void cmd(const IMU_CMD_t cmd);
        void send_all_client_data();
        void send_client_data(char*);
        void get_angles(float& roll, float& pitch, float& yaw);
        TwoWire* dev_i2c;
    private:
        void sensor_init(void);
        void calibrate_zero_offset(void);
        void calibrate_magnetometer(void);
        void calibrate_gyroscope(void);
        void reset_calibration(void);
        void calibrate_data(void);
        void AHRSCompute(void);
        void init_params(imu_calibration_params_t params);
        void params_save();
        void params_print(imu_calibration_params_t& params);

        LSM6DS3Sensor* AccGyr;
        LIS3MDLSensor* Magneto;
	AHRS* AHRSptr;

        IMU_CALIBRATE_t calibrate_enable = IMU_CALIBRATE_DISABLED;
        bool calibrate_reset = false;
        bool show_pitch = false;
        bool show_yaw   = false;
        bool show_roll  = false;
        bool ideal_data[3] = { false, false, false };
        bool data_hold[IMU_SENSOR_MAX+1] = { { false } };
	bool fixed_data = false;

        float roll, pitch, yaw;
        float yaw_last_cal;

	imu_calibration_params_t cp;				// local copy of calibration params

        ParamStore<imu_calibration_params_t> param_store;	// parameter storage object

        int32_t accelerometer_uncal[3];
        int32_t accelerometer_cal[3];

        int32_t gyroscope_uncal[3];
        int32_t gyroscope_cal_before_correction[3];
        uint32_t gyroscope_cal_before_correction_abs[3];
        float gyroscope_cal[3];

        int32_t magnetometer_uncal[3];
        int32_t magnetometer_cal[3];
	bool magnetometer_stability = true;

        bool timestamp_valid = false;
        int32_t timestamp = 0;
        int32_t timestamp_prev = 0;

        IMU_SENSOR_t sensor_select = IMU_AHRS;
        uint32_t show_input_ahrs = 0;
        float gx, gy, gz, ax, ay, az, mx, my, mz;
	AHRS_ALGORITHM_t AHRSalgorithm = AHRS_MAHONY;
};

#endif
