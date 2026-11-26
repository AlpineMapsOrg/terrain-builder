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

#pragma once

#include <cstdint>
#include <filesystem>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include <fmt/format.h>
#include <glm/vec2.hpp>
#include <radix/tile.h>

#include "common_error_codes.h"
#include "containers/UniqueResource.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "gis/CplError.h"
#include "gis/GeoTransform.h"
#include "gis/PixelTransform.h"
#include "gis/Transform.h"
#include "gis/georeferencing.h"
#include "gis/srs/System.h"
#include "gis/srs/coverage.h"
#include "gis/srs/error.h"

class GDALDataset;

namespace gis {

// A dataset has no spatial reference system.
struct MissingSrs {
    // The name of the dataset.
    std::string dataset;

    bool operator==(const MissingSrs &) const = default;
};

inline std::string format_as(const MissingSrs &error) {
    return fmt::format("dataset {} does not have a spatial reference system assigned", error.dataset);
}

// The dataset has no georeferencing of the requested kind.
struct MissingGeoreferencing : alp::MessageCode<"the dataset has no georeferencing of the requested kind"> {};

// The dataset has several kinds of georeferencing.
struct AmbiguousGeoreferencing : alp::MessageCode<"the dataset has several kinds of georeferencing"> {};

struct DatasetDeleter {
    void operator()(GDALDataset *dataset) const noexcept;
};

class Dataset {
public:
    using ReadGeoTransformError = alp::Error<MissingGeoreferencing, InvalidGeoreferencing, srs::UnsupportedAxisOrder>;
    using ReadPixelTransformOfKindError = alp::Error<MissingGeoreferencing, InvalidGeoreferencing, srs::UnsupportedAxisOrder>;
    using ReadGeoreferencingKindError = alp::Error<MissingGeoreferencing, AmbiguousGeoreferencing>;
    using ReadPixelTransformError = alp::Error<ReadGeoreferencingKindError, ReadPixelTransformOfKindError>;
    using ComputeBoundsError = alp::Error<Unsupported, ReadGeoTransformError>;
    using ComputeCoverageError = alp::Error<MissingSrs, ComputeBoundsError, srs::coverage::Error>;

    Dataset(std::filesystem::path path);
    Dataset(GDALDataset* dataset); // takes over ownership
    [[nodiscard]]
    static alp::Result<Dataset, CplError> open_raster(std::filesystem::path path);
    [[nodiscard]]
    static alp::Result<Dataset, CplError> open_vector(std::filesystem::path path);
    [[nodiscard]]
    static alp::Result<std::shared_ptr<Dataset>, CplError> open_shared_raster(std::filesystem::path path);
    // Opens the dataset's file again, as a dataset of its own of the same kind.
    // May be called from several threads at once.
    [[nodiscard]]
    alp::Result<Dataset, CplError> reopen() const;

    [[nodiscard]] std::string name() const;

    // The kinds of georeferencing this dataset has.
    [[nodiscard]] std::vector<GeoreferencingKind> georeferencing_kinds() const;
    // The dataset's only kind of georeferencing.
    // Fails if it has none or several.
    [[nodiscard]] alp::Result<GeoreferencingKind, ReadGeoreferencingKindError> georeferencing_kind() const;
    // The dataset's geotransform.
    // Fails if it has none, if it is invalid, or if its system does not order its data axes east, north.
    [[nodiscard]] alp::Result<GeoTransform, ReadGeoTransformError> geo_transform() const;
    // The transform from the dataset's pixels into its system by its only kind of georeferencing.
    // Fails if it has no or several kinds of georeferencing, or if that georeferencing cannot be turned into a transform.
    [[nodiscard]] alp::Result<PixelTransform, ReadPixelTransformError> pixel_transform() const;
    // The transform from the dataset's pixels into its system by its georeferencing of `kind`.
    // Fails if it has none of `kind`, if its system does not order its data axes east, north, or if the georeferencing cannot be turned into a transform.
    [[nodiscard]] alp::Result<PixelTransform, ReadPixelTransformOfKindError> pixel_transform(GeoreferencingKind kind) const;
    // Bounds of a north-up raster without rotation or shear, in its own SRS.
    [[nodiscard]] alp::Result<radix::tile::SrsBounds, ComputeBoundsError> bounds() const;
    [[nodiscard]] alp::Result<radix::tile::SrsAndHeightBounds, ComputeBoundsError> bounds3d(bool approx_ok = false) const;
    // See srs::coverage::compute_geographic and srs::coverage::compute_mercator.
    [[nodiscard]] alp::Result<std::vector<radix::tile::SrsBounds>, ComputeCoverageError> geographic_coverage() const;
    [[nodiscard]] alp::Result<std::vector<radix::tile::SrsBounds>, ComputeCoverageError> mercator_coverage() const;
    // The spatial reference system of the raster, or else of its first layer that has one.
    [[nodiscard]] alp::Result<srs::System, MissingSrs> srs() const;
    // The spatial reference system of the dataset's georeferencing of `kind`.
    [[nodiscard]] alp::Result<srs::System, MissingSrs> srs(GeoreferencingKind kind) const;
    // The width and height of the raster in pixels.
    [[nodiscard]] glm::uvec2 raster_size() const;
    [[nodiscard]] uint32_t band_count() const;
    [[nodiscard]] GDALDataset *as_gdal();
    [[nodiscard]] const GDALDataset *as_gdal() const;

private:
    Dataset(std::filesystem::path path, GDALDataset *dataset, uint32_t open_flags);
    UniquePointer<GDALDataset, DatasetDeleter> m_gdal_dataset;
    std::optional<std::filesystem::path> m_path;
    uint32_t m_open_flags = 0;
};

} // namespace gis
