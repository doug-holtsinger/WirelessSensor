#ifndef __AHRS_H__
#define __AHRS_H__

#include "app_config.h"
#include "imu_cmd.h"

#define sampleFreqDef    416.0f   // sample frequency in Hz

typedef enum
{
	ROLL = 0,
	PITCH,
	YAW
} EULER_ANGLE_SELECT_t;

typedef enum
{
	Q0 = 0,
	Q1,
 	Q2,
	Q3	
} QUATERNION_SELECT_t;

class AHRS {
public:
    virtual void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) =0;
    virtual void cmd(const IMU_CMD_t cmd);
#ifdef BLE_CONSOLE_AVAILABLE
    virtual void send_all_client_data(const bool *display_data, const bool settings_display);
#endif
    void ComputeAngles(float& roll, float& pitch, float& yaw);
    void GetNormalizedVectors(const IMU_SENSOR_t sensor, float& o_x, float& o_y, float& o_z) const;
    float GetAngle(const EULER_ANGLE_SELECT_t angle_select) const;
    float GetQuaternion(const QUATERNION_SELECT_t quaternion_select) const;

protected:
    float invSqrt(float x, bool accurate_calc);
#ifdef BLE_CONSOLE_AVAILABLE
    void send_client_data(char *p);
#endif
    virtual void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) =0;

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to auxiliary frame
    float q0X = 1.0f, q1X = 0.0f, q2X = 0.0f, q3X = 0.0f;    // saved quaternion of sensor frame relative to auxiliary frame

    float axN = 0.0f, ayN = 0.0f, azN = 0.0f;
    float gxN = 0.0f, gyN = 0.0f, gzN = 0.0f;
    float mxN = 0.0f, myN = 0.0f, mzN = 0.0f;
    float l_roll = 0.0;
    float l_pitch = 0.0;
    float l_yaw = 0.0;
    float l_roll_atan2f[2] = { 0.0, 0.0 };
    float l_yaw_atan2f[2] = { 0.0, 0.0 };

    float sampleFreq = sampleFreqDef;

};

#endif
