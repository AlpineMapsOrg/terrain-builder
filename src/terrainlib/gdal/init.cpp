#include <mutex>

#include <cpl_conv.h>
#include <gdal.h>

#include "build_config.h"
#include "gdal/init.h"
#include "log.h"

namespace gdal {

std::once_flag g_gdal_initialized_once_flag;

inline void GdalErrorHandler(CPLErr eErrClass, int err_no, const char *msg) {
    spdlog::level::level_enum level;
    switch (eErrClass) {
        case CE_None:
            return;
        case CE_Debug:
            level = spdlog::level::debug;
            break;
        case CE_Warning:
            level = spdlog::level::warn;
            break;
        case CE_Failure:
            level = spdlog::level::err;
            break;
        case CE_Fatal:
            level = spdlog::level::critical;
            break;
        default:
            level = spdlog::level::err;
            LOG_WARN("Unknown GDAL error class: {}", (int)eErrClass);
            break;
    }

    Log::get_logger().get()->log(level, "GDAL({}): {}", err_no, msg);
}

void initialize_once() {
    std::call_once(g_gdal_initialized_once_flag, []() {
        CPLSetErrorHandler(GdalErrorHandler);

        if constexpr (IS_DEBUG_BUILD) {
            // Every coordinate transformation checks each result by mapping it back.
            CPLSetConfigOption("CHECK_WITH_INVERT_PROJ", "YES");
        }

        LOG_DEBUG("calling GDALAllRegister...");
        GDALAllRegister();
    });
}

} // namespace gdal
