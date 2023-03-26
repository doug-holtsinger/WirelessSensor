
#include <AppDemux.h>
#include "nrf_log.h"

std::vector<AppHandler> apphandlers;
APP_CMD_t cmdNext = 0; 

void appDemuxExecHandler(const APP_CMD_t cmd)
{
    for (auto it = apphandlers.cbegin() ; it != apphandlers.cend() ; ++it)
    {
        if (cmd >= (*it).cmd_min && cmd <= (*it).cmd_max)
	{
            (*it).handler(cmd - (*it).cmd_min);
	    break;
	}
    }
}

void appDemuxAddHandler(const std::function<void(const APP_CMD_t cmd)>& handler, const APP_CMD_t cmd_max)
{
    const APP_CMD_t app_cmd_max = cmdNext + cmd_max;
    AppHandler app_handler{handler, cmdNext, app_cmd_max};
    apphandlers.push_back(app_handler);
    cmdNext = app_cmd_max + 1; 
}

