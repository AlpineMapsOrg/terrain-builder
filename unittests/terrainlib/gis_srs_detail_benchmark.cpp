// Copyright (C) 2026 Adam Celarek-Litofcenko
// Copyright (C) 2026 Martin Braunsperger
// SPDX-License-Identifier: GPL-3.0-or-later

#include "../result_helpers.h"
#include "gis/srs/System.h"
#include "gis/srs/Transform.h"
#include "gis/srs/well_known.h"
#include "gis/srs/detail.h"

#include <array>

#include <catch2/benchmark/catch_benchmark.hpp>
#include <catch2/catch_test_macros.hpp>
#include <ogr_spatialref.h>

TEST_CASE("Projected bounds transformation benchmark", "[gis][srs][detail][!benchmark][.]")
{
    const gis::srs::System source = gis::srs::wgs84();
    const gis::srs::System target = REQUIRE_VALUE(gis::srs::from_epsg(31287));
    const gis::srs::Transform transform = REQUIRE_VALUE(gis::srs::get_transform(source, target));
    OGRCoordinateTransformation *gdal_transform = transform.as_gdal();
    std::array<double, 4> bounds {};
    REQUIRE(gis::srs::detail::ogr_transform_bounds(gdal_transform, 12.6, 47, 12.61, 47.01, &bounds[0], &bounds[1], &bounds[2], &bounds[3], 257));
    BENCHMARK("GDAL TransformBounds")
    {
        gdal_transform->TransformBounds(12.6, 47, 12.61, 47.01, &bounds[0], &bounds[1], &bounds[2], &bounds[3], 257);
        return bounds;
    };
    BENCHMARK("cached projected bounds")
    {
        gis::srs::detail::ogr_transform_bounds(gdal_transform, 12.6, 47, 12.61, 47.01, &bounds[0], &bounds[1], &bounds[2], &bounds[3], 257);
        return bounds;
    };
}
