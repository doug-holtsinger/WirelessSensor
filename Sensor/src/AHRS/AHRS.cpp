
#include <string.h>
#include <math.h>

#include "AHRS.h"
#include "logdef.h"
#include "notify.h"

#include "bsp.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"



//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

//---------------------------------------------------------------------------------------------------
float AHRS::invSqrt(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long*)&y;
    i = 0x5f3759df - (i>>1);
    y = *(float*)&i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

void AHRS::send_client_data(char *p)
{
    uint8_t *p_data = (uint8_t *)p;
    ble_svcs_send_client_notification(p_data, strlen(p));
}

void AHRS::send_all_client_data()
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];

    // Accelerometer
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , ACCELEROMETER_NORMAL, PRINTF_FLOAT_VALUE2(axN), PRINTF_FLOAT_VALUE2(ayN), PRINTF_FLOAT_VALUE2(azN) );
    send_client_data(s);

    // Gyroscope
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , GYROSCOPE_NORMAL, PRINTF_FLOAT_VALUE2(gxN), PRINTF_FLOAT_VALUE2(gyN), PRINTF_FLOAT_VALUE2(gzN) );
    send_client_data(s);

    // Magnetometer
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , MAGNETOMETER_NORMAL, PRINTF_FLOAT_VALUE2(mxN), PRINTF_FLOAT_VALUE2(myN), PRINTF_FLOAT_VALUE2(mzN) );
    send_client_data(s);

    // Quaternion
    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q0, PRINTF_FLOAT_VALUE2(q0X) );
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q1, PRINTF_FLOAT_VALUE2(q1X) );
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q2, PRINTF_FLOAT_VALUE2(q2X) );
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q3, PRINTF_FLOAT_VALUE2(q3X) );
    send_client_data(s);

    snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , SAMPLE_FREQ, PRINTF_FLOAT_VALUE2(sampleFreq) );
    send_client_data(s);

}

void AHRS::ComputeAngles(float& roll, float& pitch, float& yaw) 
{
  roll = atan2f(q0 * q1 + q2 * q3, 0.5f - q1 * q1 - q2 * q2);
  roll = roll * 57.29578f;

  // attitude = asin(2*qx*qy + 2*qz*qw)
  pitch = asinf(-2.0f * (q1 * q3 - q0 * q2));
  pitch = pitch * 57.29578f;

  yaw = atan2f(q1 * q2 + q0 * q3, 0.5f - q2 * q2 - q3 * q3);
  yaw =  yaw * 57.29578f + 180.0f;
}

void AHRS::cmd(const IMU_CMD_t cmd)
{
    switch (cmd)
    {
        case IMU_AHRS_SAMPLE_FREQ_UP:
            sampleFreq += 32.0f;
            break;
        case IMU_AHRS_SAMPLE_FREQ_DOWN:
            sampleFreq -= 32.0f;
            break;
	default: break;
    }
}


