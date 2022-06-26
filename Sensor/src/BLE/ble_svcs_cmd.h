/**
 * @brief Header file for BLE
 *
 */
#ifndef __BLE_SVCS_CMD_H__
#define __BLE_SVCS_CMD_H__

typedef enum
{
    //FIXME
    BLE_NOCMD = 128,
    BLE_STOP_ADVERTISING,
    BLE_MANUFACT_DATA_TOGGLE
} BLE_CMD_t;

#if 0
typedef enum
{
        ROLL_ANGLE,
        PITCH_ANGLE,
        YAW_ANGLE
} EULER_ANGLE_t;
#endif

#endif
