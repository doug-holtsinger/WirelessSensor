/**
 * @brief Header file for BLE
 *
 */
#ifndef __BLE_SVCS_CMD_H__
#define __BLE_SVCS_CMD_H__

typedef enum
{
    BLE_NOCMD = 0,
    BLE_STOP_ADVERTISING,
    BLE_MANUFACT_DATA_TOGGLE
} BLE_CMD_t;

typedef enum
{
        ROLL_ANGLE,
        PITCH_ANGLE,
        YAW_ANGLE
} EULER_ANGLE_t;

#endif
