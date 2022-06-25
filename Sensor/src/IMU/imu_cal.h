/**
 * @brief Header file for IMU_CAL 
 *
 */


#ifndef __IMU_CAL_H__
#define __IMU_CAL_H__

typedef struct {
    uint32_t accelerometer_min_threshold[3]; 
    int32_t accelerometer_min[3]; 
    int32_t accelerometer_max[3];

    int32_t gyroscope_min[3]; 
    int32_t gyroscope_max[3];
    float gyroscope_min_threshold[3];

    int32_t magnetometer_min[3]; 
    int32_t magnetometer_max[3];
    int32_t magnetometer_min_threshold[3];
    int32_t magnetometer_uncal_last[3];
} imu_calibration_params_t;

#endif
