/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2023 Martin Braunsperger
 * Copyright (C) 2023 Adam Celarek-Litofcenko
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
#include <cmath>
#include <cstdint>

#include <cpl_error.h>
#include <fmt/format.h>
#include <gdal.h>
#include <gdal_priv.h>
#include <glm/glm.hpp>
#include <libassert/assert.hpp>
#include <radix/geometry.h>
#include <radix/raster.h>

#include "error/Result.h"
#include "error/try.h"
#include "error/utils.h"
#include "fmt_impls.h"
#include "gis/CplError.h"
#include "gis/Dataset.h"
#include "gis/GeoTransform.h"
#include "log.h"

namespace sf {

// The heights of a dataset's pixels, one per pixel.
using HeightMap = radix::Raster<float>;

class RawDatasetReader {
public:
    [[nodiscard]]
    static alp::Result<RawDatasetReader, gis::Dataset::ReadGeoTransformError> for_dataset(gis::Dataset& dataset) {
        // TODO: should we use PixelTransform here?
        ALP_TRY(const gis::GeoTransform pixel_transform, dataset.geo_transform());
        return RawDatasetReader(dataset.as_gdal(), pixel_transform);
    }

    GDALDataset *gdal_dataset() {
        return this->dataset;
    }

    double get_no_data_value() {
        DEBUG_ASSERT(this->dataset->GetRasterCount() >= 1);
        GDALRasterBand *height_band = this->dataset->GetRasterBand(1);
        return height_band->GetNoDataValue();
    }

    glm::uvec2 dataset_size() {
        GDALRasterBand *heights_band = this->dataset->GetRasterBand(1); // non-owning pointer
        return glm::uvec2(heights_band->GetXSize(), heights_band->GetYSize());
    }

    // TODO: support reading other data types
    alp::Result<HeightMap, gis::CplError> read_data_in_pixel_bounds(const radix::geometry::Aabb2i& bounds) {
        DEBUG_ASSERT(glm::all(glm::greaterThanEqual(bounds.min, glm::ivec2(0))));
        DEBUG_ASSERT(glm::all(glm::greaterThanEqual(bounds.max, glm::ivec2(0))));
        DEBUG_ASSERT(glm::all(glm::lessThan(bounds.min, glm::ivec2(this->dataset_size()))));
        DEBUG_ASSERT(glm::all(glm::lessThan(bounds.max, glm::ivec2(this->dataset_size()))));

        DEBUG_ASSERT(this->dataset->GetRasterCount() >= 1);
        GDALRasterBand *height_band = this->dataset->GetRasterBand(1); // non-owning pointer

        // Initialize the HeightData for reading
        HeightMap height_data(glm::uvec2(bounds.width(), bounds.height()));
        if (bounds.width() == 0 || bounds.height() == 0) {
            LOG_WARN("Target dataset bounds are empty");
            return height_data;
        }

        // Read data from the heights band into heights_data
        CPLErrorReset();
        const int32_t read_result = height_band->RasterIO(
            GF_Read, bounds.min.x, bounds.min.y, bounds.width(), bounds.height(),
            static_cast<void*>(height_data.buffer().data()), bounds.width(), bounds.height(), GDT_Float32, 0, 0);

        if (read_result != CE_None) {
            return alp::make_error(gis::CplError::get_last_error(), fmt::format("reading pixels {} from {}", bounds, this->dataset->GetDescription()));
        }

        return height_data;
    }
    alp::Result<HeightMap, gis::CplError> read_data_in_pixel_bounds_clamped(radix::geometry::Aabb2i &bounds) {
        const auto original_bounds = bounds;

        const glm::ivec2 max_in_bounds = glm::ivec2(this->dataset_size()) - glm::ivec2(1);
        bounds.min = glm::clamp(bounds.min, glm::ivec2(0), max_in_bounds);
        bounds.max = glm::clamp(bounds.max, bounds.min, max_in_bounds);

        if (original_bounds != bounds) {
            LOG_TRACE("Clamped target dataset bounds from [({}, {})-({}, {})] to [({}, {})-({}, {})]",
                        original_bounds.min.x, original_bounds.min.y, original_bounds.max.x, original_bounds.max.y,
                        bounds.min.x, bounds.min.y, bounds.max.x, bounds.max.y);
        }

        if (bounds.width() == 0 || bounds.height() == 0) {
            LOG_WARN("Target dataset bounds are empty (clamped)");
            return HeightMap();
        }

        return this->read_data_in_pixel_bounds(bounds);
    }

    alp::Result<HeightMap, gis::CplError> read_data_in_srs_bounds(const radix::tile::SrsBounds &bounds) {
        // Transform the SrsBounds to pixel space
        radix::geometry::Aabb2i pixel_bounds = this->transform_srs_bounds_to_pixel_bounds(bounds);

        // Use the transformed pixel bounds to read data
        return this->read_data_in_pixel_bounds(pixel_bounds);
    }

    radix::geometry::Aabb2i transform_srs_bounds_to_pixel_bounds(const radix::tile::SrsBounds &bounds) const {
        radix::geometry::Aabb2d subpixel_bounds = this->transform_srs_bounds_to_subpixel_bounds(bounds);

        const radix::geometry::Aabb2i pixel_bounds(
            glm::ivec2(static_cast<int32_t>(std::floor(subpixel_bounds.min.x)), static_cast<int32_t>(std::floor(subpixel_bounds.min.y))),
            glm::ivec2(static_cast<int32_t>(std::ceil(subpixel_bounds.max.x)), static_cast<int32_t>(std::ceil(subpixel_bounds.max.y))));

        return pixel_bounds;
    }

    radix::tile::SrsBounds transform_srs_bounds_to_subpixel_bounds(const radix::tile::SrsBounds &bounds) const {
        radix::tile::SrsBounds transformed_bounds;
        transformed_bounds.min = this->transform_srs_point_to_subpixel(bounds.min);
        transformed_bounds.max = this->transform_srs_point_to_subpixel(bounds.max);
        transformed_bounds = radix::tile::SrsBounds(glm::min(transformed_bounds.min, transformed_bounds.max),
                                                    glm::max(transformed_bounds.min, transformed_bounds.max));
        return transformed_bounds;
    }

    radix::tile::SrsBounds transform_pixel_bounds_to_srs_bounds(const radix::tile::SrsBounds &bounds) const {
        radix::tile::SrsBounds transformed_bounds;
        transformed_bounds.min = this->transform_pixel_to_srs_point(bounds.min);
        transformed_bounds.max = this->transform_pixel_to_srs_point(bounds.max);
        transformed_bounds = radix::tile::SrsBounds(glm::min(transformed_bounds.min, transformed_bounds.max),
                                                    glm::max(transformed_bounds.min, transformed_bounds.max));
        return transformed_bounds;
    }
    radix::tile::SrsBounds transform_pixel_bounds_to_srs_bounds(const radix::geometry::Aabb2i &bounds) const {
        return this->transform_pixel_bounds_to_srs_bounds(radix::tile::SrsBounds(glm::dvec2(bounds.min), glm::dvec2(bounds.max)));
    }

    template <typename T>
    inline glm::ivec2 transform_srs_point_to_pixel(const glm::tvec2<T> &p) const {
        glm::dvec2 subpixel = this->transform_srs_point_to_subpixel(p);
        const glm::ivec2 pixel(std::rint(subpixel.x), std::rint(subpixel.y));
        return pixel;
    }

    template <typename T>
    inline glm::dvec2 transform_srs_point_to_subpixel(const glm::tvec2<T> &p) const {
        return this->pixel_transform.apply_inverse(glm::dvec2(p));
    }

    template <typename T>
    inline glm::dvec2 transform_pixel_to_srs_point(const glm::tvec2<T> &p) const {
        return this->pixel_transform.apply(glm::dvec2(p));
    }

private:
    RawDatasetReader(GDALDataset* dataset, const gis::GeoTransform& pixel_transform)
        : dataset(dataset), pixel_transform(pixel_transform) {}

    // TODO: should we store a gis::Dataset?
    GDALDataset *dataset;
    gis::GeoTransform pixel_transform;
};

}
