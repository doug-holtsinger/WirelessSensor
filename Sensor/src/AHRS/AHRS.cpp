
#include <string.h>
#include <math.h>

#include "AHRS.h"
#include "logdef.h"
#include "notify.h"

#ifdef BLE_CONSOLE_AVAILABLE
#include "bsp.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"
#endif


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

#ifdef BLE_CONSOLE_AVAILABLE
void AHRS::send_client_data(char *p)
{
    uint8_t *p_data = (uint8_t *)p;
    ble_svcs_send_client_notification(p_data, strlen(p));
}

void AHRS::send_all_client_data(const bool *display_data, const bool settings_display)
{
    char s[NOTIFY_PRINT_STR_MAX_LEN];

    // Accelerometer
    if (display_data[IMU_ACCELEROMETER])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , ACCELEROMETER_NORMAL, PRINTF_FLOAT_VALUE2(axN), PRINTF_FLOAT_VALUE2(ayN), PRINTF_FLOAT_VALUE2(azN) );
        send_client_data(s);
    }

    // Gyroscope
    if (display_data[IMU_GYROSCOPE])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT3 , GYROSCOPE_NORMAL_X, PRINTF_FLOAT_VALUE3(gxN) );
        send_client_data(s);
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT3 , GYROSCOPE_NORMAL_Y, PRINTF_FLOAT_VALUE3(gyN) );
        send_client_data(s);
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT3 , GYROSCOPE_NORMAL_Z, PRINTF_FLOAT_VALUE3(gzN) );
        send_client_data(s);
    }

    // Magnetometer
    if (display_data[IMU_MAGNETOMETER])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , MAGNETOMETER_NORMAL, PRINTF_FLOAT_VALUE2(mxN), PRINTF_FLOAT_VALUE2(myN), PRINTF_FLOAT_VALUE2(mzN) );
        send_client_data(s);
    }

    // Quaternion
    if (display_data[IMU_AHRS])
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q0, PRINTF_FLOAT_VALUE4(q0X) );
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q1, PRINTF_FLOAT_VALUE4(q1X) );
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q2, PRINTF_FLOAT_VALUE4(q2X) );
        send_client_data(s);

        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT4 , QUATERNION_Q3, PRINTF_FLOAT_VALUE4(q3X) );
        send_client_data(s);
    }

    if (settings_display)
    {
        snprintf(s, NOTIFY_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , SAMPLE_FREQ, PRINTF_FLOAT_VALUE2(sampleFreq) );
        send_client_data(s);
    }

}
#endif

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

void AHRS::GetNormalizedVectors(IMU_SENSOR_t sensor, float& o_x, float& o_y, float& o_z)
{
    switch(sensor)
    {
        case IMU_ACCELEROMETER:
	    o_x = axN; o_y = ayN ; o_z = azN; break;
        case IMU_GYROSCOPE:
	    o_x = gxN; o_y = gyN ; o_z = gzN; break;
        case IMU_MAGNETOMETER:
	    o_x = mxN; o_y = myN ; o_z = mzN; break;
	default: break;
    }

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


