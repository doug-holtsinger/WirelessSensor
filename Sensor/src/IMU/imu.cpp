#include <stdint.h>
#include <stdlib.h>

#include "imu.h"
#include "MahonyAHRS.h"

// Nordic I2C
#include "nrfx_twi.h"

IMU::IMU()
{
    // NRF_LOG_INFO("Before TwoWire init.");
    // dev_i2c = new TwoWire();
    // NRF_LOG_INFO("After TwoWire init.");
    // sensor_init();
    // NRF_LOG_INFO("After sensor_init.");
}

void IMU::init(void)
{
    dev_i2c = new TwoWire();
    sensor_init();
}

void IMU::calibrate_magnetometer(void)
{
    //
    //  magnetometer calibration -- done while moving
    //
    for (int i = 0 ; i < 3 ; i++) {
        if (magnetometer_min[i] == 0 && magnetometer_max[i] == 0) 
        {
            magnetometer_min[i] = magnetometer_max[i] = magnetometer_uncal_last[i] = magnetometer_uncal[i];
        } else if (magnetometer_uncal[i] < magnetometer_min[i]) 
        {
            magnetometer_min[i] = magnetometer_uncal_last[i] = magnetometer_uncal[i];
        } else if (magnetometer_uncal[i] > magnetometer_max[i]) 
        {
            magnetometer_max[i] = magnetometer_uncal_last[i] = magnetometer_uncal[i];
        }
    }
}

void IMU::calibrate_zero_offset(void)
{
    uint32_t noise_threshold;
    int32_t noise_threshold_i;
    float noise_threshold_f;

    //
    // magnetometer calibration -- done while motionless
    //
    for (int i = 0 ; i < 3 ; i++)
    {
        noise_threshold_i = NOISE_THRESHOLD_MULTIPLIER * abs( magnetometer_uncal[i] - magnetometer_uncal_last[i]); 
        if (noise_threshold_i > magnetometer_min_threshold[i])
        {
            magnetometer_min_threshold[i] = noise_threshold_i;
        }
    }

    //
    // gyroscope calibration -- done while motionless
    //
    for (int i = 0 ; i < 3 ; i++) 
    {
        if (gyroscope_min[i] == 0 && gyroscope_max[i] == 0) 
        {
            gyroscope_min[i] = gyroscope_max[i] = gyroscope_uncal[i];
        } else if (gyroscope_uncal[i] < gyroscope_min[i]) 
        {
            gyroscope_min[i] = gyroscope_uncal[i];
        } else if (gyroscope_uncal[i] > gyroscope_max[i]) 
        {
            gyroscope_max[i] = gyroscope_uncal[i];
        }
        noise_threshold_f = NOISE_THRESHOLD_MULTIPLIER * abs ( gyroscope_uncal[i] - ((gyroscope_max[i] + gyroscope_min[i]) / 2) ) / (float)(1ULL << gyroscope_sensitivity) ;
        if (noise_threshold_f > gyroscope_min_threshold[i])
        {
            gyroscope_min_threshold[i] = noise_threshold_f;
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
        if (accelerometer_min[i] == 0 && accelerometer_max[i] == 0) 
        {
            accelerometer_min[i] = accelerometer_max[i] = (accelerometer_uncal[i] - accelerometer_bias);
        } else if ((accelerometer_uncal[i] - accelerometer_bias) < accelerometer_min[i]) 
        {
            accelerometer_min[i] = accelerometer_uncal[i] - accelerometer_bias;
        } else if ((accelerometer_uncal[i] - accelerometer_bias) > accelerometer_max[i]) 
        {
            accelerometer_max[i] = accelerometer_uncal[i] - accelerometer_bias;
        }
        noise_threshold = NOISE_THRESHOLD_MULTIPLIER * abs( accelerometer_uncal[i] - ((accelerometer_max[i] + accelerometer_min[i]) / 2) - accelerometer_bias);
        if (noise_threshold > accelerometer_min_threshold[i])
        {
            accelerometer_min_threshold[i] = noise_threshold;
        }
    }
}

void IMU::reset_calibration(void)
{
    for (int i = 0 ; i < 3 ; i++) 
    {
        magnetometer_min[i] = magnetometer_max[i] = magnetometer_min_threshold[i] = 0;
        magnetometer_uncal_last[i] = 0;
    }

    for (int i = 0 ; i < 3 ; i++) 
    {
        gyroscope_min[i] = gyroscope_max[i] = gyroscope_min_threshold[i] = 0; 
    }

    for (int i = 0 ; i < 3 ; i++) 
    {
        accelerometer_min[i] = accelerometer_max[i] = accelerometer_min_threshold[i] = 0;
    }
    calibrate_reset = false;
}

void IMU::calibrate_data(void)
{
    int32_t magnetometer_diff = 0;

    // calibrate raw data values using zero offset and Min thresholds.
    for (int i = 0 ; i < 3 ; i++) {
        accelerometer_cal[i] = accelerometer_uncal[i] - ((accelerometer_max[i] + accelerometer_min[i]) / 2);
        if ((uint32_t)abs(accelerometer_cal[i]) < accelerometer_min_threshold[i])
        {
            accelerometer_cal[i] = 0;
        }
        gyroscope_cal[i] = ( gyroscope_uncal[i] - ((gyroscope_max[i] + gyroscope_min[i]) / 2) ) / (float)(1ULL << gyroscope_sensitivity) ;
        if (gyroscope_cal[i] > -gyroscope_min_threshold[i] &&
            gyroscope_cal[i] < gyroscope_min_threshold[i] )
        {
            gyroscope_cal[i] = 0;
        }

        magnetometer_diff = abs(magnetometer_uncal[i] - magnetometer_uncal_last[i]);
        if (magnetometer_diff < magnetometer_min_threshold[i]) 
        {
            magnetometer_cal[i] = magnetometer_uncal_last[i] - ((magnetometer_max[i] + magnetometer_min[i]) / 2);
        } else 
        {
            magnetometer_cal[i] = magnetometer_uncal[i] - ((magnetometer_max[i] + magnetometer_min[i]) / 2);
        }
        magnetometer_uncal_last[i] = magnetometer_uncal[i];

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


void IMU::print_data()
{

    if (sensor_select == IMU_AHRS) 
    {
        if (show_input_ahrs == 1) 
        {
            if (show_roll)
                   printf("gyro = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(gx), PRINTF_FLOAT_VALUE(gy), PRINTF_FLOAT_VALUE(gz) );
            if (show_pitch)
                  printf("acce = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(ax), PRINTF_FLOAT_VALUE(ay), PRINTF_FLOAT_VALUE(az) );
            if (show_yaw)
                   printf("magn = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(mx), PRINTF_FLOAT_VALUE(my), PRINTF_FLOAT_VALUE(mz) );
	} else if (show_input_ahrs == 2) 
        {
           if (show_roll)
               printf("gyro normal = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(gxN), PRINTF_FLOAT_VALUE(gyN), PRINTF_FLOAT_VALUE(gzN) );
           if (show_pitch)
                printf("acce normal = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(axN), PRINTF_FLOAT_VALUE(ayN), PRINTF_FLOAT_VALUE(azN) );
           if (show_yaw)
                printf("magn normal = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(mxN), PRINTF_FLOAT_VALUE(myN), PRINTF_FLOAT_VALUE(mzN) );
	} else if (show_input_ahrs == 3) 
        {
           if (show_roll)
               printf("q0X q2X q3X = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(q0X), PRINTF_FLOAT_VALUE(q2X), PRINTF_FLOAT_VALUE(q3X) );
           if (show_pitch)
               printf("q1X  q2X  q3X = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(q1X), PRINTF_FLOAT_VALUE(q2X), PRINTF_FLOAT_VALUE(q3X) );
           if (show_yaw)
               printf("q0X  q1X  q2X = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n", PRINTF_FLOAT_VALUE(q0X), PRINTF_FLOAT_VALUE(q1X), PRINTF_FLOAT_VALUE(q2X) );
        } else {
            if (show_roll)
                    printf("Roll  = " PRINTF_FLOAT_FORMAT " \r\n", PRINTF_FLOAT_VALUE(roll));
            if (show_pitch)
                    printf("Pitch = " PRINTF_FLOAT_FORMAT " \r\n", PRINTF_FLOAT_VALUE(pitch));
            if (show_yaw)
                    printf("Yaw   = " PRINTF_FLOAT_FORMAT " \r\n", PRINTF_FLOAT_VALUE(yaw));
        }
    } else if (sensor_select == IMU_ACCELEROMETER)
    {
        printf("Accel = = %04d %04d %04d  |  %04d %04d %04d |  %04d %04d %04d\r\n", 
            (int)accelerometer_cal[0],
            (int)accelerometer_cal[1],
            (int)accelerometer_cal[2],
            (int)accelerometer_uncal[0],
            (int)accelerometer_uncal[1],
            (int)accelerometer_uncal[2],
            (int)accelerometer_min_threshold[0],
            (int)accelerometer_min_threshold[1],
            (int)accelerometer_min_threshold[2]
            );
    } else if (sensor_select == IMU_GYROSCOPE)
    {
        printf("Gyro calibrated = " PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT PRINTF_FLOAT_FORMAT "\r\n",
            PRINTF_FLOAT_VALUE(gyroscope_cal[0]),
            PRINTF_FLOAT_VALUE(gyroscope_cal[1]),
            PRINTF_FLOAT_VALUE(gyroscope_cal[2])
        );
    } else if (sensor_select == IMU_MAGNETOMETER)
    {
        printf("Mag = %04d %04d %04d  |  %04d %04d %04d  |  %04d %04d %04d\r\n", 
            (int)magnetometer_cal[0],
            (int)magnetometer_cal[1],
            (int)magnetometer_cal[2],
            (int)magnetometer_uncal[0],
            (int)magnetometer_uncal[1],
            (int)magnetometer_uncal[2],
            (int)magnetometer_min_threshold[0],
            (int)magnetometer_min_threshold[1],
            (int)magnetometer_min_threshold[2]
            );
    }
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

    // TBD -- can combine status of Accel and Gyro according to app note
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
                for (int i=0 ; i < 100 ; i++) 
                {
                    printf("Calibrate Disabled\r\n");
                }
            else if (calibrate_enable == IMU_SENSOR_CALIBRATE_ZERO_OFFSET)
                for (int i=0 ; i < 100 ; i++) 
                {
                    printf("Calibrate Zero Offset Enable\r\n");
                }
            else if (calibrate_enable == IMU_SENSOR_CALIBRATE_MAGNETOMETER)
                for (int i=0 ; i < 100 ; i++) 
                {
                    printf("Calibrate Magnetometer Enable\r\n");
                }
            break;
        case IMU_SENSOR_CALIBRATE_RESET:
            // reset calibration values
            calibrate_reset = true;
            calibrate_enable = IMU_SENSOR_CALIBRATE_DISABLED;
            printf("Calibrate Reset/Disabled\r\n");
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
                printf("Gyroscope Data Disabled\r\n");
            else
                printf("Gyroscope Data Enabled\r\n");
            } else if (sensor_select == IMU_ACCELEROMETER) 
            {
                zero_data[0] = !zero_data[0];
                if (zero_data[0]) 
                    printf("Accelerometer Data Disabled\r\n");
                else
                    printf("Accelerometer Data Enabled\r\n");
            } else if (sensor_select == IMU_MAGNETOMETER) 
            {
                zero_data[2] = !zero_data[2];
                if (zero_data[2]) 
                    printf("Magnetometer Data Disabled\r\n");
                else
                    printf("Magnetometer Data Enabled\r\n");
            }
            break;
        case IMU_SENSOR_DATA_IDEAL:
            if (sensor_select == IMU_GYROSCOPE) 
            {
                ideal_data[1] = ideal_data[1] ? 0 : 1;
            if (ideal_data[1]) 
                printf("Gyroscope Data Ideal\r\n");
            else
                printf("Gyroscope Data Real\r\n");
            } else if (sensor_select == IMU_ACCELEROMETER) 
            {
                ideal_data[0] = ideal_data[0] ? 0 : 1;
                if (ideal_data[0]) 
                    printf("Accelerometer Data Ideal\r\n");
                else
                    printf("Accelerometer Data Real\r\n");
            } else if (sensor_select == IMU_MAGNETOMETER) 
            {
                ideal_data[2] = ideal_data[2] ? 0 : 1;
                if (ideal_data[2]) 
                    printf("Magnetometer Data Ideal\r\n");
                else
                    printf("Magnetometer Data Real\r\n");
            }
            break;
        case IMU_SENSOR_DATA_FIXED_TOGGLE:
	    fixed_data = !fixed_data;
	    break;
        case IMU_AHRS_PROP_GAIN_UP:
            twoKp += 0.1f;
            printf("twoKp : %lu\r\n", (uint32_t)(twoKp * 1000.0));
            break;
        case IMU_AHRS_PROP_GAIN_DOWN:
            twoKp -= 0.1f;
            printf("twoKp : %lu\r\n", (uint32_t)(twoKp * 1000.0));
            break;
        case IMU_AHRS_INTEG_GAIN_UP:
            twoKi += 0.1f;
            printf("twoKi : %lu\r\n", (uint32_t)(twoKi * 1000.0));
            break;
        case IMU_AHRS_INTEG_GAIN_DOWN:
            twoKi -= 0.1f;
            printf("twoKi : %lu\r\n", (uint32_t)(twoKi * 1000.0));
            break;
        case IMU_AHRS_SAMPLE_FREQ_UP:
            sampleFreq += 32.0f;
            printf("sampleFreq : %lu\r\n", (uint32_t)(sampleFreq));
            break;
        case IMU_AHRS_SAMPLE_FREQ_DOWN:
            sampleFreq -= 32.0f;
            printf("sampleFreq : %lu\r\n", (uint32_t)(sampleFreq));
            break;
        case IMU_GYROSCOPE_SENSITIVITY_UP:
            gyroscope_sensitivity++; 
            printf("gyroscope_sensitivity : %ld\r\n", gyroscope_sensitivity);
            break;
        case IMU_GYROSCOPE_SENSITIVITY_DOWN:
            gyroscope_sensitivity--; 
            printf("gyroscope_sensitivity : %ld\r\n", gyroscope_sensitivity);
            break;
        default: break;
    }
}

