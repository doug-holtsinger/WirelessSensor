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

#define PRINTF_FLOAT_FORMAT3 " %c%ld.%03ld"
#define PRINTF_FLOAT_VALUE3(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*1000)

#define PRINTF_FLOAT_FORMAT4 " %c%ld.%04ld"
#define PRINTF_FLOAT_VALUE4(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*10000)

#define PRINTF_FLOAT_FORMAT7 " %c%ld.%07ld"
#define PRINTF_FLOAT_VALUE7(val) (uint8_t)(((val) < 0 && (val) > -1.0) ? '-' : ' '),   \
                           (int32_t)(val),                                       \
                           (int32_t)((((val) > 0) ? (val) - (int32_t)(val)       \
                                                : (int32_t)(val) - (val))*10000000)



#endif
