/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *****************************************************************************/

#include <mutex>

#include <cpl_conv.h>
#include <gdal.h>

#include "build_config.h"
#include "gis/gdal.h"
#include "cli_support/log.h"

namespace gis::gdal {

namespace {

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

    cli_support::Log::get_logger().get()->log(level, "GDAL({}): {}", err_no, msg);
}

} // namespace

void initialize_once() {
    std::call_once(g_gdal_initialized_once_flag, []() {
        LOG_DEBUG("calling GDALAllRegister...");
        CPLSetErrorHandler(GdalErrorHandler);

        // TODO: enable again once longitudes beyond 180 pass the check; GDAL rejects them because their round trip comes back within the turn, which fails the scenario "antimeridian" of "ECEF coverage encloses the transformed box" in unittests/terrainlib/gis_srs_coverage.cpp.
        /*
        if constexpr (IS_DEBUG_BUILD) {
            // Every coordinate transformation checks each result by mapping it back.
            CPLSetConfigOption("CHECK_WITH_INVERT_PROJ", "YES");
        }
        */

        GDALAllRegister();
        // Initialize GDAL's global block-cache lock before concurrent reads.
        GDALGetCacheMax64();
    });
}

} // namespace gis::gdal
