/**
 * @brief Header file for IMU 
 *
 */


#ifndef __IMU_H__
#define __IMU_H__

#include "TwoWire.h"
#include "LSM6DS3Sensor.h"
#include "LIS3MDLSensor.h"

// FIXME: remove constants
// #define ACCELEROMETER_MIN_THRESHOLD 100
// #define GYROSCOPE_MIN_THRESHOLD 0.05
// #define MAGNETOMETER_MIN_THRESHOLD 15
#define NOISE_THRESHOLD_MULTIPLIER 2

/* print defines */
#define PRINTF_FLOAT_FORMAT " %c%ld.%02ld"
#define PRINTF_FLOAT_VALUE(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
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
    IMU_PRINT_MAGNETOMETER,
    IMU_PRINT_GYROSCOPE, 
    IMU_PRINT_ACCELEROMETER,
    IMU_PRINT_AHRS,
    IMU_SENSOR_CALIBRATE_TOGGLE,
    IMU_SENSOR_CALIBRATE_RESET,
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
    IMU_GYROSCOPE_SENSITIVITY_DOWN
} IMU_CMD_t;

typedef enum {
        IMU_SENSOR_CALIBRATE_DISABLED = 0,
        IMU_SENSOR_CALIBRATE_MIN = IMU_SENSOR_CALIBRATE_DISABLED,
        IMU_SENSOR_CALIBRATE_ZERO_OFFSET, 
        IMU_SENSOR_CALIBRATE_MAGNETOMETER,
        IMU_SENSOR_CALIBRATE_MAX = IMU_SENSOR_CALIBRATE_MAGNETOMETER
} IMU_SENSOR_CALIBRATE_t;

class IMU {
    public:
        IMU();
        void update(void);
        // FIXME: fold init() into constructor
        void init(void);
        void cmd(IMU_CMD_t& cmd);
        void print_data();
        void get_angles(float& roll, float& pitch, float& yaw);
        TwoWire* dev_i2c;
    private:
        void sensor_init(void);
        void calibrate_zero_offset(void);
        void calibrate_magnetometer(void);
        void reset_calibration(void);
        void calibrate_data(void);
        void AHRS(void);

        LSM6DS3Sensor* AccGyr;
        LIS3MDLSensor* Magneto;

        IMU_SENSOR_CALIBRATE_t calibrate_enable = IMU_SENSOR_CALIBRATE_DISABLED;
        bool calibrate_reset = false;
        bool show_pitch = true;
        bool show_yaw   = true;
        bool show_roll  = true;
        unsigned int ideal_data[3] = { 0, 0, 0 };
        bool zero_data[3] = { false, false, false };
	bool fixed_data = false;

        float roll, pitch, yaw;

        int32_t accelerometer_uncal[3];
        int32_t accelerometer_cal[3];
        int32_t accelerometer_bias;
        uint32_t accelerometer_min_threshold[3] = { 0, 0, 0 };
        int32_t accelerometer_min[3] = { 0, 0, 0 };
        int32_t accelerometer_max[3] = { 0, 0, 0 };

        int32_t gyroscope_uncal[3];
        float gyroscope_cal[3];
        int32_t gyroscope_min[3] =  { 0, 0, 0 };
        int32_t gyroscope_max[3] =  { 0, 0, 0 };
        float gyroscope_min_threshold[3] = { 0.0, 0.0, 0.0 };
        // FIXME: remove constant
        int32_t gyroscope_sensitivity = 16;

        int32_t magnetometer_uncal[3];
        int32_t magnetometer_uncal_last[3];
        int32_t magnetometer_cal[3];
        int32_t magnetometer_min[3] = { 0, 0, 0 };
        int32_t magnetometer_max[3] = { 0, 0, 0 };
        int32_t magnetometer_min_threshold[3] = { 0, 0, 0 };
        IMU_SENSOR_t sensor_select = IMU_AHRS;
        uint32_t show_input_ahrs = 0;
        float gx, gy, gz, ax, ay, az, mx, my, mz;
};

#endif
