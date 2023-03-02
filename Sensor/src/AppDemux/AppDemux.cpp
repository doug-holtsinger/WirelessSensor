
#include <AppDemux.h>
#include "nrf_log.h"

std::vector<AppHandler> apphandlers;
APP_CMD_t cmdNext = 0; 

void appDemuxExecHandler(const APP_CMD_t cmd)
{
    NRF_LOG_INFO("appDemuxExecHandler s %d", apphandlers.size());
    for (auto it = apphandlers.cbegin() ; it != apphandlers.cend() ; ++it)
    {
        NRF_LOG_INFO("appDemuxExecHandler %d %d %d", cmd, (*it).cmd_min, (*it).cmd_max );
        if (cmd >= (*it).cmd_min && cmd <= (*it).cmd_max)
	{
            NRF_LOG_INFO("appDemuxExecHandler call %d", cmd - (*it).cmd_min );
            (*it).handler(cmd - (*it).cmd_min);
	    break;
	}
    }
}

extern int32_t destruct;
#if 0
template <typename T>
void appDemuxAddHandler(std::function<void(const T cmd)> handler, const T cmd_max)
#else
void appDemuxAddHandler(const std::function<void(const APP_CMD_t cmd)>& handler, const APP_CMD_t cmd_max)
#endif
{
#if 1
    const APP_CMD_t app_cmd_max = cmdNext + cmd_max;
    AppHandler app_handler{handler, cmdNext, app_cmd_max};
    apphandlers.push_back(app_handler);
    cmdNext = app_cmd_max + 1; 
#endif
}

#if 0
template <typename T>
constexpr APP_CMD_t appDemuxCmdType(T e) noexcept
{
    return static_cast<std::underlying_type_t<T>>(e);
}
#endif

