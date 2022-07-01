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
#include "param_store.h"

// FIXME: remove constants
// #define ACCELEROMETER_MIN_THRESHOLD 100
// #define GYROSCOPE_MIN_THRESHOLD 0.05
// #define MAGNETOMETER_MIN_THRESHOLD 15
#define GYROSCOPE_SENSITIVITY_THRESHOLD 16
#define NOISE_THRESHOLD_MULTIPLIER 2

/* print defines */
#define PRINTF_FLOAT_FORMAT " %c%ld.%01ld"
#define PRINTF_FLOAT_VALUE(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*10)

#define PRINTF_FLOAT_FORMAT2 " %c%ld.%02ld"
#define PRINTF_FLOAT_VALUE2(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*100)


typedef enum
{
    IMU_AHRS = 0,
    IMU_ACCELEROMETER,
    IMU_GYROSCOPE,
    IMU_MAGNETOMETER
} IMU_SENSOR_t;

typedef enum
{
    IMU_NOCMD = 0,
    IMU_CMD_MIN = IMU_NOCMD, 
    IMU_PRINT_MAGNETOMETER,
    IMU_PRINT_GYROSCOPE,
    IMU_PRINT_ACCELEROMETER,
    IMU_PRINT_AHRS,
    IMU_SENSOR_CALIBRATE_NORMALIZED,
    IMU_SENSOR_CALIBRATE_ZERO_OFFSET,
    IMU_SENSOR_CALIBRATE_MAGNETOMETER,
    IMU_SENSOR_CALIBRATE_RESET,
    IMU_SENSOR_CALIBRATE_SAVE,
    IMU_AHRS_INPUT_TOGGLE,
    IMU_AHRS_YAW_TOGGLE,
    IMU_AHRS_PITCH_TOGGLE,
    IMU_AHRS_ROLL_TOGGLE,
    IMU_SENSOR_DATA_ZERO,
    IMU_SENSOR_DATA_IDEAL,
    IMU_SENSOR_DATA_FIXED_TOGGLE,
    IMU_AHRS_PROP_GAIN_UP,
    IMU_AHRS_PROP_GAIN_DOWN,
    IMU_AHRS_INTEG_GAIN_UP,
    IMU_AHRS_INTEG_GAIN_DOWN,
    IMU_AHRS_SAMPLE_FREQ_UP,
    IMU_AHRS_SAMPLE_FREQ_DOWN,
    IMU_GYROSCOPE_SENSITIVITY_UP,
    IMU_GYROSCOPE_SENSITIVITY_DOWN,
    IMU_MAGNETOMETER_STABILITY_TOGGLE,
    IMU_CMD_MAX = IMU_MAGNETOMETER_STABILITY_TOGGLE
} IMU_CMD_t;

typedef enum {
        IMU_CALIBRATE_DISABLED = 0,
        IMU_CALIBRATE_MIN = IMU_CALIBRATE_DISABLED,
        IMU_CALIBRATE_ZERO_OFFSET,
        IMU_CALIBRATE_MAGNETOMETER,
        IMU_CALIBRATE_MAX = IMU_CALIBRATE_MAGNETOMETER
} IMU_CALIBRATE_t;

typedef enum
{
    EULER_ANGLES = 0,

    ACCELEROMETER_NORMAL,
    ACCELEROMETER_CAL,
    ACCELEROMETER_UNCAL,
    ACCELEROMETER_MIN_THRESHOLD,

    GYROSCOPE_NORMAL,
    GYROSCOPE_CAL,
    GYROSCOPE_UNCAL,
    GYROSCOPE_MIN_THRESHOLD,

    MAGNETOMETER_NORMAL,
    MAGNETOMETER_CAL,
    MAGNETOMETER_UNCAL,
    MAGNETOMETER_MIN_THRESHOLD,

    QUATERNION_Q0,
    QUATERNION_Q1,
    QUATERNION_Q2,
    QUATERNION_Q3,
    IMU_DATA_NOTIFY_MAX = QUATERNION_Q3
} IMU_DATA_NOTIFY_t;

typedef enum {
    GYRO_SENSITIVITY = IMU_DATA_NOTIFY_MAX + 1,
    MAGNETOMETER_STABILITY,
    IMU_SETTINGS_NOTIFY_MAX = MAGNETOMETER_STABILITY
} IMU_SETTINGS_NOTIFY_t;

typedef enum {
    PROP_GAIN = IMU_SETTINGS_NOTIFY_MAX + 1,
    INTEG_GAIN,
    SAMPLE_FREQ,
    ALGORITHM
} AHRS_SETTINGS_NOTIFY_t;

class IMU {
    public:
        IMU();
        ~IMU();
        void update(void);
        void init(void);
        void get_params(imu_calibration_params_t& params);
        void cmd(IMU_CMD_t& cmd);
        void send_all_client_data();
        void send_client_data(char*);
        void get_angles(float& roll, float& pitch, float& yaw);
        TwoWire* dev_i2c;
    private:
        void sensor_init(void);
        void calibrate_zero_offset(void);
        void calibrate_magnetometer(void);
        void reset_calibration(void);
        void calibrate_data(void);
        void AHRS(void);
        void init_params(imu_calibration_params_t params);
        void params_save();
        void params_print(imu_calibration_params_t& params);

        LSM6DS3Sensor* AccGyr;
        LIS3MDLSensor* Magneto;

        IMU_CALIBRATE_t calibrate_enable = IMU_CALIBRATE_DISABLED;
        bool calibrate_reset = false;
        bool show_pitch = false;
        bool show_yaw   = false;
        bool show_roll  = false;
        unsigned int ideal_data[3] = { 0, 0, 0 };
        bool zero_data[3] = { false, false, false };
	bool fixed_data = false;

        float roll, pitch, yaw;

	imu_calibration_params_t cp;				// local copy of calibration params

        ParamStore<imu_calibration_params_t> param_store;	// parameter storage object

        int32_t accelerometer_uncal[3];
        int32_t accelerometer_cal[3];

        int32_t gyroscope_uncal[3];
        float gyroscope_cal[3];
        int32_t gyroscope_sensitivity = GYROSCOPE_SENSITIVITY_THRESHOLD;

        int32_t magnetometer_uncal[3];
        int32_t magnetometer_cal[3];
	bool magnetometer_stability = true;
        IMU_SENSOR_t sensor_select = IMU_AHRS;
        uint32_t show_input_ahrs = 0;
        float gx, gy, gz, ax, ay, az, mx, my, mz;
	int32_t AHRSalgorithm = 0;
};

#endif
