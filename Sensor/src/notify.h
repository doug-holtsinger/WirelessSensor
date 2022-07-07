
#ifndef __NOTIFY_H__
#define __NOTIFY_H__

#define NOTIFY_PRINT_STR_MAX_LEN (size_t)256

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

    GYRO_SENSITIVITY, 
    MAGNETOMETER_STABILITY,

    PROP_GAIN, 
    INTEG_GAIN,
    SAMPLE_FREQ,
    AHRS_ALGORITHM,

    BETA_GAIN

} DATA_NOTIFY_t;

#endif
