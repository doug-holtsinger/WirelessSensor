#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bsp.h"
#include "imu.h"
#include "MahonyAHRS.h"
#include "ble_svcs_cmd.h"
#include "ble_svcs.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"

// Nordic I2C
#include "nrfx_twi.h"

#include "app_config.h"
#include "param_store.h"
#include "imu_cal.h"

#define IMU_PRINT_STR_MAX_LEN (size_t)256


#ifdef __cplusplus
extern "C" {
#endif
extern void fds_evt_handler_C(fds_evt_t const * p_evt);
#ifdef __cplusplus
}
#endif


IMU::IMU()
{
}

IMU::~IMU()
{
}

void IMU::init(void)
{
    imu_calibration_params_t imu_cal_params;
    dev_i2c = new TwoWire();
    sensor_init();
    reset_calibration();
    // Initialize param storage device
    param_store.init(&cp);
    // Read parameters from storage and initialize local copy
    imu_cal_params = param_store.get();
    params_print(imu_cal_params);
    init_params(imu_cal_params);
}

void IMU::params_save()
{
    params_print(cp);
    param_store.set(&cp);
}

void IMU::params_print(imu_calibration_params_t& params)
{
    NRF_LOG_INFO("IMU Parameters");
    for (int i = 0 ; i < 3 ; i++)
    {
        //NRF_LOG_INFO("  Mag min %d 0x%04x", i, params.magnetometer_min[i]);
        //NRF_LOG_INFO("  Mag max %d 0x%04x", i, params.magnetometer_max[i]);
        NRF_LOG_INFO("  Mag min %d thresh 0x%04x", i, params.magnetometer_min_threshold[i]);
    }
    for (int i = 0 ; i < 3 ; i++)
    {
        //NRF_LOG_INFO("  Gyr min %d 0x%04x", i, params.gyroscope_min[i]);
        NRF_LOG_INFO("  Gyr max %d 0x%04x", i, params.gyroscope_max[i]);
        // NRF_LOG_INFO("  Gyr min thresh %d %s" PRINTF_FLOAT_FORMAT PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[i]) );
    }
    for (int i = 0 ; i < 3 ; i++)
    {
        //NRF_LOG_INFO("  Acc min %d 0x%04x", i, params.accelerometer_min[i]);
        //NRF_LOG_INFO("  Acc max %d 0x%04x", i, params.accelerometer_max[i]);
        NRF_LOG_INFO("  Acc min thresh %d 0x%04x", i, params.accelerometer_min_threshold[i]);
    }
}

void IMU::init_params(imu_calibration_params_t params)
{
    // FIXME -- reset AHRS settings, like gyro sensitivity, as well
    for (int i = 0 ; i < 3 ; i++)
    {
        cp.magnetometer_min[i] = params.magnetometer_min[i];
	cp.magnetometer_max[i] = params.magnetometer_max[i];
        cp.magnetometer_min_threshold[i] = params.magnetometer_min_threshold[i];
        cp.magnetometer_uncal_last[i] = params.magnetometer_uncal_last[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.gyroscope_min[i] = params.gyroscope_min[i];
        cp.gyroscope_max[i] = params.gyroscope_max[i];
        cp.gyroscope_min_threshold[i] = params.gyroscope_min_threshold[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.accelerometer_min[i] = params.accelerometer_min[i];
        cp.accelerometer_max[i] = params.accelerometer_max[i];
        cp.accelerometer_min_threshold[i] = params.accelerometer_min_threshold[i];
    }
}

void IMU::get_params(imu_calibration_params_t& params)
{

    for (int i = 0 ; i < 3 ; i++)
    {
        params.magnetometer_min[i] = cp.magnetometer_min[i];
	params.magnetometer_max[i] = cp.magnetometer_max[i];
        params.magnetometer_min_threshold[i] = cp.magnetometer_min_threshold[i];
        params.magnetometer_uncal_last[i] = cp.magnetometer_uncal_last[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        params.gyroscope_min[i] = cp.gyroscope_min[i];
        params.gyroscope_max[i] = cp.gyroscope_max[i];
        params.gyroscope_min_threshold[i] = cp.gyroscope_min_threshold[i];
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        params.accelerometer_min[i] = cp.accelerometer_min[i];
        params.accelerometer_max[i] = cp.accelerometer_max[i];
        params.accelerometer_min_threshold[i] = cp.accelerometer_min_threshold[i];
    }
}

void IMU::send_debug_data(char *p)
{
#ifdef BLE_CONSOLE_AVAILABLE
    {
        uint8_t *p_data = (uint8_t *)p;
        ble_svcs_send_debug_data(p_data, strlen(p));
    }
#endif
#ifdef SERIAL_CONSOLE_AVAILABLE
    {
        char s[IMU_PRINT_STR_MAX_LEN];
        snprintf(s, IMU_PRINT_STR_MAX_LEN, "%s\r\n", p);
        puts(s);
    }
#endif
}

void IMU::calibrate_magnetometer(void)
{
    //
    //  magnetometer calibration -- done while moving
    //
    for (int i = 0 ; i < 3 ; i++) {
        if (cp.magnetometer_min[i] == 0 && cp.magnetometer_max[i] == 0)
        {
            cp.magnetometer_min[i] = cp.magnetometer_max[i] = cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];
        } else if (magnetometer_uncal[i] < cp.magnetometer_min[i])
        {
            cp.magnetometer_min[i] = cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];
        } else if (magnetometer_uncal[i] > cp.magnetometer_max[i])
        {
            cp.magnetometer_max[i] = cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];
        }
    }
}

void IMU::calibrate_zero_offset(void)
{
    uint32_t noise_threshold;
    int32_t noise_threshold_i;
    float noise_threshold_f;
    int32_t accelerometer_bias;

    //
    // magnetometer calibration -- done while motionless
    //
    for (int i = 0 ; i < 3 ; i++)
    {
        noise_threshold_i = NOISE_THRESHOLD_MULTIPLIER * abs( magnetometer_uncal[i] - cp.magnetometer_uncal_last[i]);
        if (noise_threshold_i > cp.magnetometer_min_threshold[i])
        {
            cp.magnetometer_min_threshold[i] = noise_threshold_i;
        }
    }

    //
    // gyroscope calibration -- done while motionless
    //
    for (int i = 0 ; i < 3 ; i++)
    {
        if (cp.gyroscope_min[i] == 0 && cp.gyroscope_max[i] == 0)
        {
            cp.gyroscope_min[i] = cp.gyroscope_max[i] = gyroscope_uncal[i];
        } else if (gyroscope_uncal[i] < cp.gyroscope_min[i])
        {
            cp.gyroscope_min[i] = gyroscope_uncal[i];
        } else if (gyroscope_uncal[i] > cp.gyroscope_max[i])
        {
            cp.gyroscope_max[i] = gyroscope_uncal[i];
        }
        noise_threshold_f = NOISE_THRESHOLD_MULTIPLIER * abs ( gyroscope_uncal[i] - ((cp.gyroscope_max[i] + cp.gyroscope_min[i]) / 2) ) / (float)(1ULL << gyroscope_sensitivity) ;
        if (noise_threshold_f > cp.gyroscope_min_threshold[i])
        {
            cp.gyroscope_min_threshold[i] = noise_threshold_f;
        }
    }

    //
    // accelerometer calibration -- done while motionless with
    // IMU Z axis pointing up.
    //
    for (int i = 0 ; i < 3 ; i++) {
        if (i == 2)
        {
            accelerometer_bias = 1000;    // 1G bias in Z direction
        } else
        {
            accelerometer_bias = 0;
        }
        if (cp.accelerometer_min[i] == 0 && cp.accelerometer_max[i] == 0)
        {
            cp.accelerometer_min[i] = cp.accelerometer_max[i] = (accelerometer_uncal[i] - accelerometer_bias);
        } else if ((accelerometer_uncal[i] - accelerometer_bias) < cp.accelerometer_min[i])
        {
            cp.accelerometer_min[i] = accelerometer_uncal[i] - accelerometer_bias;
        } else if ((accelerometer_uncal[i] - accelerometer_bias) > cp.accelerometer_max[i])
        {
            cp.accelerometer_max[i] = accelerometer_uncal[i] - accelerometer_bias;
        }
        noise_threshold = NOISE_THRESHOLD_MULTIPLIER * abs( accelerometer_uncal[i] - ((cp.accelerometer_max[i] + cp.accelerometer_min[i]) / 2) - accelerometer_bias);
        if (noise_threshold > cp.accelerometer_min_threshold[i])
        {
            cp.accelerometer_min_threshold[i] = noise_threshold;
        }
    }
}

void IMU::reset_calibration(void)
{
    // FIXME -- reset AHRS settings, like gyro sensitivity, as well
    for (int i = 0 ; i < 3 ; i++)
    {
        cp.magnetometer_min[i] = cp.magnetometer_max[i] = cp.magnetometer_min_threshold[i] = 0;
        cp.magnetometer_uncal_last[i] = 0;
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.gyroscope_min[i] = cp.gyroscope_max[i] = cp.gyroscope_min_threshold[i] = 0;
    }

    for (int i = 0 ; i < 3 ; i++)
    {
        cp.accelerometer_min[i] = cp.accelerometer_max[i] = cp.accelerometer_min_threshold[i] = 0;
    }
    calibrate_reset = false;
}

void IMU::calibrate_data(void)
{
    int32_t magnetometer_diff = 0;

    // calibrate raw data values using zero offset and Min thresholds.
    for (int i = 0 ; i < 3 ; i++) {
        accelerometer_cal[i] = accelerometer_uncal[i] - ((cp.accelerometer_max[i] + cp.accelerometer_min[i]) / 2);
        if ((uint32_t)abs(accelerometer_cal[i]) < cp.accelerometer_min_threshold[i])
        {
            accelerometer_cal[i] = 0;
        }
        gyroscope_cal[i] = ( gyroscope_uncal[i] - ((cp.gyroscope_max[i] + cp.gyroscope_min[i]) / 2) ) / (float)(1ULL << gyroscope_sensitivity) ;
        if (gyroscope_cal[i] > -cp.gyroscope_min_threshold[i] &&
            gyroscope_cal[i] < cp.gyroscope_min_threshold[i] )
        {
            gyroscope_cal[i] = 0;
        }

        magnetometer_diff = abs(magnetometer_uncal[i] - cp.magnetometer_uncal_last[i]);
        if (magnetometer_diff < cp.magnetometer_min_threshold[i])
        {
            magnetometer_cal[i] = cp.magnetometer_uncal_last[i] - ((cp.magnetometer_max[i] + cp.magnetometer_min[i]) / 2);
        } else
        {
            magnetometer_cal[i] = magnetometer_uncal[i] - ((cp.magnetometer_max[i] + cp.magnetometer_min[i]) / 2);
        }
        cp.magnetometer_uncal_last[i] = magnetometer_uncal[i];

    }

}

void IMU::get_angles(float& o_roll, float& o_pitch, float& o_yaw)
{
    o_roll = roll;
    o_pitch = pitch;
    o_yaw = yaw;
}

void IMU::AHRS()
{
    if (sensor_select == IMU_AHRS)
    {
        gx = gyroscope_cal[0];
        gy = gyroscope_cal[1];
        gz = gyroscope_cal[2];
        ax = (float)accelerometer_cal[0];
        ay = (float)accelerometer_cal[1];
        az = (float)accelerometer_cal[2];
        mx = (float)magnetometer_cal[0];
        my = (float)magnetometer_cal[1];
        mz = (float)magnetometer_cal[2];
        if (zero_data[1])
        {
            gz = gy = gx = 0.0f;
        }
        if (zero_data[0])
        {
            az = ay = ax = 0.0f;
        }
        if (zero_data[2])
        {
            mz = my = mx = 0.0f;
        }
	if (ideal_data[0])
	{
            ax = 1000.0f; ay = 0.0f ; az = 0.0f;
	}
	if (ideal_data[1])
	{
            gx = 0.0f; gy = 0.0f ; gz = 0.0f;
	}
	if (ideal_data[2])
	{
            mx = 400.0f; my = 0.0f ; mz = 0.0f;
	}
        MahonyAHRSupdate(gx, gy, gz, ax, ay, az, mx, my, mz);
        MahonyAHRSComputeAngles(roll, pitch, yaw);
	if (fixed_data)
	{
		roll = 45.0;
		pitch = 90.0;
		yaw = 180.0;
	}
    }

}

void IMU::print_debug_data()
{
    char s[IMU_PRINT_STR_MAX_LEN];

#if defined(BLE_CONSOLE_AVAILABLE) && !defined(SERIAL_CONSOLE_AVAILABLE)
    // Check if BLE connected, otherwise return.
    if ( !ble_svcs_connected() ) {
	    return;
    }
#endif


    // Euler Angles
    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT , EULER_ANGLES, PRINTF_FLOAT_VALUE(roll), PRINTF_FLOAT_VALUE(pitch), PRINTF_FLOAT_VALUE(yaw));
    send_debug_data(s);

    // Accelerometer
    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , ACCELEROMETER_NORMAL, PRINTF_FLOAT_VALUE2(axN), PRINTF_FLOAT_VALUE2(ayN), PRINTF_FLOAT_VALUE2(azN) );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
             ACCELEROMETER_CAL,
            (int)accelerometer_cal[0],
            (int)accelerometer_cal[1],
            (int)accelerometer_cal[2]
            );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
             ACCELEROMETER_UNCAL,
            (int)accelerometer_uncal[0],
            (int)accelerometer_uncal[1],
            (int)accelerometer_uncal[2]
            );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
             ACCELEROMETER_MIN_THRESHOLD,
            (int)cp.accelerometer_min_threshold[0],
            (int)cp.accelerometer_min_threshold[1],
            (int)cp.accelerometer_min_threshold[2]
            );
    send_debug_data(s);

    // Gyroscope
    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , GYROSCOPE_NORMAL, PRINTF_FLOAT_VALUE2(gxN), PRINTF_FLOAT_VALUE2(gyN), PRINTF_FLOAT_VALUE2(gzN) );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 ,
            GYROSCOPE_CAL,
            PRINTF_FLOAT_VALUE2(gyroscope_cal[0]),
            PRINTF_FLOAT_VALUE2(gyroscope_cal[1]),
            PRINTF_FLOAT_VALUE2(gyroscope_cal[2])
        );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
            GYROSCOPE_UNCAL,
            (int)gyroscope_uncal[0],
            (int)gyroscope_uncal[1],
            (int)gyroscope_uncal[2]
            );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT ,
            GYROSCOPE_MIN_THRESHOLD,
            PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[0]),
            PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[1]),
            PRINTF_FLOAT_VALUE(cp.gyroscope_min_threshold[2])
        );
    send_debug_data(s);

    // Magnetometer
    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 PRINTF_FLOAT_FORMAT2 , MAGNETOMETER_NORMAL, PRINTF_FLOAT_VALUE2(mxN), PRINTF_FLOAT_VALUE2(myN), PRINTF_FLOAT_VALUE2(mzN) );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
            MAGNETOMETER_CAL,
            (int)magnetometer_cal[0],
            (int)magnetometer_cal[1],
            (int)magnetometer_cal[2]
            );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
            MAGNETOMETER_UNCAL,
            (int)magnetometer_uncal[0],
            (int)magnetometer_uncal[1],
            (int)magnetometer_uncal[2]
            );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d %04d %04d %04d",
            MAGNETOMETER_MIN_THRESHOLD,
            (int)cp.magnetometer_min_threshold[0],
            (int)cp.magnetometer_min_threshold[1],
            (int)cp.magnetometer_min_threshold[2]
            );
    send_debug_data(s);

    // Quaternion
    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q0, PRINTF_FLOAT_VALUE2(q0X) );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q1, PRINTF_FLOAT_VALUE2(q1X) );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q2, PRINTF_FLOAT_VALUE2(q2X) );
    send_debug_data(s);

    snprintf(s, IMU_PRINT_STR_MAX_LEN, "%d " PRINTF_FLOAT_FORMAT2 , QUATERNION_Q3, PRINTF_FLOAT_VALUE2(q3X) );
    send_debug_data(s);

}

void IMU::update()
{
    LSM6DS3StatusTypeDef lsm_err_code = LSM6DS3_STATUS_OK;
    LIS3MDLStatusTypeDef lis_err_code = LIS3MDL_STATUS_OK;
    uint8_t reg_data = 0;
    bool new_data_avail = false;

    //
    // Read LSM6DS3
    //
    lsm_err_code = AccGyr->ReadReg(LSM6DS3_ACC_GYRO_STATUS_REG, &reg_data);
    APP_ERROR_CHECK(lsm_err_code);

    // FIXME -- can combine status of Accel and Gyro according to app note
    // Gyro status lags Accel status.
    if ((reg_data & LSM6DS3_ACC_GYRO_XLDA_MASK) == LSM6DS3_ACC_GYRO_XLDA_DATA_AVAIL)
    {
        new_data_avail = true;
        lsm_err_code = AccGyr->Get_X_Axes(accelerometer_uncal);
        APP_ERROR_CHECK(lsm_err_code);
    }
    if ((reg_data & LSM6DS3_ACC_GYRO_GDA_MASK) == LSM6DS3_ACC_GYRO_GDA_DATA_AVAIL)
    {
        new_data_avail = true;
        lsm_err_code = AccGyr->Get_G_Axes(gyroscope_uncal);
        APP_ERROR_CHECK(lsm_err_code);
    }

    //
    // Read LIS3MDL
    //

    lis_err_code = Magneto->GetAxes(magnetometer_uncal);
    APP_ERROR_CHECK(lis_err_code);

    if (calibrate_reset)
    {
        reset_calibration();
    } else if (new_data_avail)
    {
        if (calibrate_enable == IMU_SENSOR_CALIBRATE_ZERO_OFFSET)
        {
            calibrate_zero_offset();
        }
        if (calibrate_enable == IMU_SENSOR_CALIBRATE_MAGNETOMETER)
        {
            calibrate_magnetometer();
        }
        calibrate_data();
        AHRS();
    }

}

/**
 */
void IMU::sensor_init(void)
{
    AccGyr = new LSM6DS3Sensor(dev_i2c, TWI_ADDRESS_LSM6DS3);
    AccGyr->Enable_X();
    AccGyr->Enable_G();
    // AccGyr->Enable_G_Filter(LSM6DS3_ACC_GYRO_HPCF_G_16Hz32);
    // AccGyr->Enable_6D_Orientation();

    Magneto = new LIS3MDLSensor(dev_i2c, TWI_ADDRESS_LIS3MDL);
    Magneto->Enable();
}

void IMU::cmd(IMU_CMD_t& cmd)
{
    char s[IMU_PRINT_STR_MAX_LEN];
    switch (cmd)
    {
        case IMU_PRINT_MAGNETOMETER:
            // magnetometer
            sensor_select = IMU_MAGNETOMETER;
            break;
        case IMU_PRINT_GYROSCOPE:
            // gyro
            sensor_select = IMU_GYROSCOPE;
            break;
        case IMU_PRINT_ACCELEROMETER:
            // accelerometer
            sensor_select = IMU_ACCELEROMETER;
            break;
        case IMU_PRINT_AHRS:
            sensor_select = IMU_AHRS;
            break;
        case IMU_AHRS_INPUT_TOGGLE:
            show_input_ahrs = ( show_input_ahrs + 1 ) % 4;
            break;
        case IMU_SENSOR_CALIBRATE_TOGGLE:
            // calibrate command
            switch (calibrate_enable)
            {
                case IMU_SENSOR_CALIBRATE_DISABLED:
                        calibrate_enable = IMU_SENSOR_CALIBRATE_ZERO_OFFSET; break;
                case IMU_SENSOR_CALIBRATE_ZERO_OFFSET:
                        calibrate_enable = IMU_SENSOR_CALIBRATE_MAGNETOMETER; break;
                case IMU_SENSOR_CALIBRATE_MAGNETOMETER:
                        calibrate_enable = IMU_SENSOR_CALIBRATE_DISABLED; break;
            }
            if (calibrate_enable == IMU_SENSOR_CALIBRATE_DISABLED)
	    {
                snprintf(s, IMU_PRINT_STR_MAX_LEN, "Calibrate Disabled");
		send_debug_data(s);
	    } else if (calibrate_enable == IMU_SENSOR_CALIBRATE_ZERO_OFFSET)
	    {
                snprintf(s, IMU_PRINT_STR_MAX_LEN, "Calibrate Zero Offset Enable");
		send_debug_data(s);
	    } else if (calibrate_enable == IMU_SENSOR_CALIBRATE_MAGNETOMETER)
	    {
                snprintf(s, IMU_PRINT_STR_MAX_LEN, "Calibrate Magnetometer Enable");
		send_debug_data(s);
	    }
            break;
        case IMU_SENSOR_CALIBRATE_RESET:
            // reset calibration values
            calibrate_reset = true;
            calibrate_enable = IMU_SENSOR_CALIBRATE_DISABLED;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "Calibrate Reset/Disabled");
            send_debug_data(s);
            break;
        case IMU_SENSOR_CALIBRATE_SAVE:
	    params_save();
            break;
        case IMU_AHRS_YAW_TOGGLE:
            show_yaw = !show_yaw;
            break;
        case IMU_AHRS_PITCH_TOGGLE:
            show_pitch = !show_pitch;
            break;
        case IMU_AHRS_ROLL_TOGGLE:
            show_roll = !show_roll;
            break;
        case IMU_SENSOR_DATA_ZERO:
            if (sensor_select == IMU_GYROSCOPE)
	    {
                zero_data[1] = !zero_data[1];
                if (zero_data[1])
	        {
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Gyro Data Disabled");
                    send_debug_data(s);
	        } else
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Gyro Data Enabled");
                    send_debug_data(s);
	        }
            } else if (sensor_select == IMU_ACCELEROMETER)
            {
                zero_data[0] = !zero_data[0];
                if (zero_data[0])
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Acc Data Disabled");
                    send_debug_data(s);
		} else
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Acc Data Enabled");
                    send_debug_data(s);
		}
            } else if (sensor_select == IMU_MAGNETOMETER)
            {
                zero_data[2] = !zero_data[2];
                if (zero_data[2])
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Mag Data Disabled");
                    send_debug_data(s);
		} else
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Mag Data Enabled");
                    send_debug_data(s);
		}
            }
            break;
        case IMU_SENSOR_DATA_IDEAL:
            if (sensor_select == IMU_GYROSCOPE)
            {
                ideal_data[1] = ideal_data[1] ? 0 : 1;
                if (ideal_data[1])
	        {
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Gyro Data Ideal");
                    send_debug_data(s);
	        } else
	        {
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Gyro Data Real");
                    send_debug_data(s);
		}
            } else if (sensor_select == IMU_ACCELEROMETER)
            {
                ideal_data[0] = ideal_data[0] ? 0 : 1;
                if (ideal_data[0])
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Acc Data Ideal");
                    send_debug_data(s);
		} else
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Acc Data Real");
                    send_debug_data(s);
		}
            } else if (sensor_select == IMU_MAGNETOMETER)
            {
                ideal_data[2] = ideal_data[2] ? 0 : 1;
                if (ideal_data[2])
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Mag Data Ideal");
                    send_debug_data(s);
		} else
		{
                    snprintf(s, IMU_PRINT_STR_MAX_LEN, "Mag Data Real");
                    send_debug_data(s);
		}
            }
            break;
        case IMU_SENSOR_DATA_FIXED_TOGGLE:
	    fixed_data = !fixed_data;
	    break;
        case IMU_AHRS_PROP_GAIN_UP:
            twoKp += 0.1f;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "twoKp %lu", (uint32_t)(twoKp * 1000.0));
            send_debug_data(s);
            break;
        case IMU_AHRS_PROP_GAIN_DOWN:
            twoKp -= 0.1f;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "twoKp %lu", (uint32_t)(twoKp * 1000.0));
            send_debug_data(s);
            break;
        case IMU_AHRS_INTEG_GAIN_UP:
            twoKi += 0.1f;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "twoKi %lu", (uint32_t)(twoKi * 1000.0));
            send_debug_data(s);
            break;
        case IMU_AHRS_INTEG_GAIN_DOWN:
            twoKi -= 0.1f;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "twoKi %lu", (uint32_t)(twoKi * 1000.0));
            send_debug_data(s);
            break;
        case IMU_AHRS_SAMPLE_FREQ_UP:
            sampleFreq += 32.0f;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "sampleFreq %lu", (uint32_t)(sampleFreq));
            send_debug_data(s);
            break;
        case IMU_AHRS_SAMPLE_FREQ_DOWN:
            sampleFreq -= 32.0f;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "sampleFreq %lu", (uint32_t)(sampleFreq));
            send_debug_data(s);
            break;
        case IMU_GYROSCOPE_SENSITIVITY_UP:
            gyroscope_sensitivity++;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "gyro sens: %ld", gyroscope_sensitivity);
            send_debug_data(s);
            break;
        case IMU_GYROSCOPE_SENSITIVITY_DOWN:
            gyroscope_sensitivity--;
            snprintf(s, IMU_PRINT_STR_MAX_LEN, "gyro sense: %ld", gyroscope_sensitivity);
            send_debug_data(s);
            break;
        default: break;
    }
}

