/**
 * @brief Header file for global definitions 
 *
 */


#ifndef __LOGDEF_H__
#define __LOGDEF_H__

/* print defines */
#define PRINTF_FLOAT_FORMAT " %c%ld.%01ld"
#define PRINTF_FLOAT_VALUE(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*10)

#define PRINTF_FLOAT_FORMAT2 " %c%ld.%02ld"
#define PRINTF_FLOAT_VALUE2(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*100)


#endif
