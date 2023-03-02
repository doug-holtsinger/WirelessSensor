
#ifndef _APPDEMUX_
#define _APPDEMUX_

#include <vector>
#include <cstdint>
#include <functional>
#include <type_traits>
#include "imu_cmd.h"

typedef uint8_t APP_CMD_t;

struct AppHandler {
    std::function<void(const APP_CMD_t cmd)> handler;
    APP_CMD_t cmd_min;
    APP_CMD_t cmd_max;
};

void appDemuxExecHandler(const APP_CMD_t cmd);
#if 0
template <typename T>
void appDemuxAddHandler(std::function<void(const T cmd)> handler, const T cmd_max);
#else
void appDemuxAddHandler(const std::function<void(const APP_CMD_t cmd)>& handler, const APP_CMD_t cmd_max);
#endif

template <typename T>
constexpr APP_CMD_t appDemuxCmdType(T e) noexcept
{
    return static_cast<std::underlying_type_t<T>>(e);
}

#endif
