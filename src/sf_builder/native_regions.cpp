/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
 * Copyright (C) 2026 Martin Braunsperger
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

#include "native_regions.h"

#include <optional>
#include <utility>

#include "error/try.h"
#include "gis/srs/Kind.h"
#include "std_compat.h"

namespace sf {

alp::Result<std::vector<radix::tile::SrsBounds>, ComputeNativeRegionsError> compute_native_regions(
    const gis::srs::System& dataset_srs, const radix::tile::SrsBounds& native_bounds, const radix::geometry::Aabb3d& ecef_bounds) {
    ALP_TRY(const std::vector<radix::geometry::Aabb3d> coverage, gis::srs::coverage::compute_from_ecef(ecef_bounds, dataset_srs));

    // A geographic x is a longitude, which repeats every turn; a projected x is an easting, which does not.
    const bool is_geographic = dataset_srs.kind() == gis::srs::Kind::Geographic;
    std::vector<radix::tile::SrsBounds> regions;
    for (const radix::geometry::Aabb3d& bounds : coverage) {
        const radix::tile::SrsBounds horizontal_bounds(bounds);
        if (is_geographic) {
            ALP_TRY(const std::vector<radix::tile::SrsBounds> wrapped, gis::srs::wrap_longitudes(dataset_srs, horizontal_bounds, native_bounds.centre()));
            std_compat::append_range(regions, wrapped);
        } else {
            regions.push_back(horizontal_bounds);
        }
    }
    return regions;
}

std::vector<radix::tile::SrsBounds> clip_to_native_bounds(const std::vector<radix::tile::SrsBounds>& regions, const radix::tile::SrsBounds& native_bounds) {
    std::vector<radix::tile::SrsBounds> windows;
    for (const radix::tile::SrsBounds& region : regions) {
        const std::optional<radix::tile::SrsBounds> window_opt = radix::geometry::intersection_strict(region, native_bounds);
        if (window_opt.has_value()) {
            windows.push_back(*window_opt);
        }
    }
    return windows;
}

alp::Result<bool, IntersectsNativeBoundsError> intersects_native_bounds(
    const gis::srs::System& dataset_srs, const radix::tile::SrsBounds& native_bounds, const radix::geometry::Aabb3d& ecef_bounds) {
    auto regions_result = compute_native_regions(dataset_srs, native_bounds, ecef_bounds);
    if (!regions_result.has_value()) {
        ALP_TRY(std::move(regions_result).error().split<gis::UntransformablePoint>());
        // The dataset's srs does not reach the bounds.
        return false;
    }
    return !clip_to_native_bounds(*regions_result, native_bounds).empty();
}

} // namespace sf
