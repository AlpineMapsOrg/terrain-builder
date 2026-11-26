/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2023 Martin Braunsperger
 * Copyright (C) 2023 Adam Celarek-Litofcenko
 * Copyright (C) 2023 Adrian Gawor
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

#include <cstdlib>
#include <utility>

#include "Downloader.h"
#include "HttpClient.h"
#include "TileUrlBuilder.h"
#include "cli.h"
#include "cli_support/log.h"
#include "common_error_codes.h"
#include "error/Context.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"

[[nodiscard]]
alp::Result<void> run(const tile_downloader::cli::Args &args) {
    if (args.srs != 3857) {
        return alp::make_error(Unsupported{}, alp::make_context("EPSG:{} as the output SRS", args.srs));
    }

    const auto provider_config = args.provider.has_value()
        ? tile_downloader::tile_provider_config(*args.provider)
        : tile_downloader::TileProviderConfig { *args.url_pattern, args.url_y_direction };
    const tile_downloader::TileUrlBuilder url_builder(provider_config);

    const radix::tile::Id root_id = {args.zoom, {args.x, args.y}};

    ALP_TRY(tile_downloader::HttpClient http, tile_downloader::HttpClient::make());
    tile_downloader::Downloader downloader(url_builder, std::move(http), args.output, args.max_zoom_level, root_id.zoom_level);
    ALP_TRY(downloader.download_recursive(root_id));
    return {};
}

int main(int argc, char **argv) {
    const tile_downloader::cli::Args args = tile_downloader::cli::parse(argc, argv);
    cli_support::Log::init(args.log_level);

    const alp::Result<void> result = run(args);
    if (!result.has_value()) {
        LOG_REPORT(result.error());
        return EXIT_FAILURE;
    }
    return EXIT_SUCCESS;
}
