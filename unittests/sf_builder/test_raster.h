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

#pragma once

#include <array>
#include <filesystem>
#include <vector>

#include <catch2/catch_test_macros.hpp>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <radix/tile.h>

#include "gis/srs/System.h"

// Writes a north-up raster with a constant height of 100 m.
inline std::filesystem::path write_raster(
    const std::filesystem::path& directory, const gis::srs::System& srs, const radix::tile::SrsBounds& bounds, const glm::uvec2 size)
{
    GDALAllRegister();
    const auto path = directory / "heights.tif";
    GDALDriver* driver = GetGDALDriverManager()->GetDriverByName("GTiff");
    REQUIRE(driver != nullptr);
    GDALDataset* dataset = driver->Create(path.c_str(), static_cast<int>(size.x), static_cast<int>(size.y), 1, GDT_Float32, nullptr);
    REQUIRE(dataset != nullptr);
    std::array<double, 6> transform { bounds.min.x, bounds.width() / size.x, 0, bounds.max.y, 0, -bounds.height() / size.y };
    REQUIRE(dataset->SetGeoTransform(transform.data()) == CE_None);
    REQUIRE(dataset->SetSpatialRef(&srs.as_gdal()) == CE_None);
    GDALRasterBand* band = dataset->GetRasterBand(1);
    REQUIRE(band->SetNoDataValue(-9999) == CE_None);
    REQUIRE(band->SetUnitType("m") == CE_None);
    std::vector<float> heights(static_cast<size_t>(size.x) * size.y, 100.0f);
    REQUIRE(band->RasterIO(GF_Write, 0, 0, static_cast<int>(size.x), static_cast<int>(size.y), heights.data(), static_cast<int>(size.x), static_cast<int>(size.y), GDT_Float32, 0, 0) == CE_None);
    double min = 0, max = 0, mean = 0, deviation = 0;
    REQUIRE(band->ComputeStatistics(false, &min, &max, &mean, &deviation, nullptr, nullptr) == CE_None);
    GDALClose(dataset);
    return path;
}
