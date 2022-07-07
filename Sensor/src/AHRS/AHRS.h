#ifndef __AHRS_H__
#define __AHRS_H__

#include "imu_cmd.h"

#define sampleFreqDef    416.0f   // sample frequency in Hz

class AHRS {
public:
    // FIXME virtual ~AHRS();
    virtual void Update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) =0;
    virtual void cmd(const IMU_CMD_t cmd);
    virtual void send_all_client_data();
    void ComputeAngles(float& roll, float& pitch, float& yaw);

protected:
    float invSqrt(float x);
    void send_client_data(char *p);
    virtual void UpdateIMU(float gx, float gy, float gz, float ax, float ay, float az) =0;

    float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;    // quaternion of sensor frame relative to auxiliary frame
    float q0X = 1.0f, q1X = 0.0f, q2X = 0.0f, q3X = 0.0f;    // saved quaternion of sensor frame relative to auxiliary frame

    float axN, ayN, azN;
    float gxN, gyN, gzN;
    float mxN, myN, mzN;

    float sampleFreq = sampleFreqDef;

};

#endif
