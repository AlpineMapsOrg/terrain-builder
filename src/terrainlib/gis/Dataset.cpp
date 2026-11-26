/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
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

#include "gis/Dataset.h"

#include <algorithm>
#include <array>
#include <cassert>
#include <cmath>
#include <memory>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string_view>
#include <vector>

#include <cpl_error.h>
#include <fmt/format.h>
#include <gdal_priv.h>
#include <libassert/assert.hpp>
#include <ogrsf_frmts.h>

#include "cli_support/log.h"
#include "earth.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "gis/OgrError.h"
#include "gis/gdal.h"
#include "gis/srs/System.h"
#include "gis/srs/Unit.h"
#include "gis/srs/classify.h"
#include "gis/srs/coverage.h"
#include "gis/srs/well_known.h"

namespace gis {

void DatasetDeleter::operator()(GDALDataset *dataset) const noexcept {
    GDALReleaseDataset(dataset);
}

[[nodiscard]]
static alp::Result<GDALDataset *, CplError> open_gdal_dataset(const std::filesystem::path &path, unsigned int flags) {
    gdal::initialize_once();
    const std::string path_str = path.string();
    static std::mutex gdal_open_mutex;
    const std::lock_guard lock(gdal_open_mutex);
    // GDAL records why an open failed only with GDAL_OF_VERBOSE_ERROR, and a reset keeps an older error from standing in for it.
    CPLErrorReset();
    GDALDataset *dataset = static_cast<GDALDataset *>(GDALOpenEx(path_str.c_str(), flags | GDAL_OF_VERBOSE_ERROR, nullptr, nullptr, nullptr));
    if (dataset == nullptr) {
        return alp::make_error(CplError::get_last_error(), fmt::format("opening a dataset at {}", path_str));
    }
    return dataset;
}

alp::Result<Dataset, CplError> Dataset::open_raster(std::filesystem::path path) {
    constexpr uint32_t open_flags = GDAL_OF_RASTER;
    ALP_TRY(GDALDataset *dataset, open_gdal_dataset(path, open_flags));
    return Dataset(path, dataset, open_flags);
}
alp::Result<Dataset, CplError> Dataset::open_vector(std::filesystem::path path) {
    constexpr uint32_t open_flags = GDAL_OF_VECTOR;
    ALP_TRY(GDALDataset *dataset, open_gdal_dataset(path, open_flags));
    return Dataset(path, dataset, open_flags);
}
// TODO: investigate GDAL_OF_THREAD_SAFE again (see 7455f47b)
alp::Result<std::shared_ptr<Dataset>, CplError> Dataset::open_shared_raster(std::filesystem::path path) {
    constexpr uint32_t open_flags = GDAL_OF_RASTER;
    ALP_TRY(GDALDataset *dataset, open_gdal_dataset(path, open_flags));
    return std::make_shared<Dataset>(Dataset(path, dataset, open_flags));
}

Dataset::Dataset(std::filesystem::path path) {
    const auto result = open_gdal_dataset(path, 0);
    if (result.has_value()) {
        m_path = path;
        m_gdal_dataset.reset(result.value());
    } else {
        LOG_REPORT(result.error());
        throw std::runtime_error("Failed to open dataset at path: " + path.string());
    }
}

Dataset::Dataset(GDALDataset *dataset) {
    m_gdal_dataset.reset(dataset);
    if (!m_gdal_dataset) {
        LOG_ERROR("Dataset is null.\n");
        throw std::runtime_error("Dataset is null.");
    }
}
Dataset::Dataset(const std::filesystem::path path, GDALDataset *dataset, const uint32_t open_flags) : Dataset(dataset) {
    m_path = path;
    m_open_flags = open_flags;
}

alp::Result<Dataset, CplError> Dataset::reopen() const {
    LOG_TRACE("Reopening dataset {}.", m_path.has_value() ? m_path->string() : "unknown");
    ASSERT(m_path.has_value(), "only a dataset opened from a path can be reopened");
    ALP_TRY(GDALDataset *dataset, open_gdal_dataset(this->m_path.value(), this->m_open_flags));
    return Dataset(this->m_path.value(), dataset, this->m_open_flags);
}

std::vector<GeoreferencingKind> Dataset::georeferencing_kinds() const {
    std::vector<GeoreferencingKind> kinds;
    std::array<double, 6> geo_transform;
    if (this->m_gdal_dataset->GetGeoTransform(geo_transform.data()) == CE_None) {
        kinds.push_back(GeoreferencingKind::GeoTransform);
    }
    if (this->m_gdal_dataset->GetGCPCount() != 0) {
        kinds.push_back(GeoreferencingKind::GcpPolynomial);
    }
    if (CSLCount(this->m_gdal_dataset->GetMetadata("RPC")) != 0) {
        kinds.push_back(GeoreferencingKind::Rpc);
    }
    if (CSLCount(this->m_gdal_dataset->GetMetadata("GEOLOCATION")) != 0) {
        kinds.push_back(GeoreferencingKind::GeolocationArray);
    }
    return kinds;
}

alp::Result<GeoTransform, Dataset::ReadGeoTransformError> Dataset::geo_transform() const {
    const std::string context = fmt::format("reading the geotransform of {}", this->name());
    std::array<double, 6> coefficients;
    if (this->m_gdal_dataset->GetGeoTransform(coefficients.data()) != CE_None) {
        return alp::make_error(MissingGeoreferencing{}, context);
    }
    // The geotransform's x is the first data axis of its system, so the system must order its data axes east, north.
    const auto system_result = this->srs(GeoreferencingKind::GeoTransform);
    if (system_result && !srs::is_east_north_axis_order(*system_result)) {
        return alp::make_error(srs::UnsupportedAxisOrder{}, context);
    }
    auto geo_transform_result = GeoTransform::from_coefficients(coefficients);
    if (!geo_transform_result) {
        return alp::make_error(InvalidGeoreferencing{}, context, std::move(geo_transform_result).error());
    }
    return *std::move(geo_transform_result);
}

alp::Result<GeoreferencingKind, Dataset::ReadGeoreferencingKindError> Dataset::georeferencing_kind() const {
    const std::string context = fmt::format("reading the kind of georeferencing of {}", this->name());
    const std::vector<GeoreferencingKind> kinds = this->georeferencing_kinds();
    if (kinds.empty()) {
        return alp::make_error(MissingGeoreferencing{}, context);
    }
    // With several kinds there is no silent choice, because GDAL would prefer one of them in a fixed order even when it is less accurate.
    // TODO: maybe replace this failure with a priority list of kinds.
    if (kinds.size() > 1) {
        return alp::make_error(AmbiguousGeoreferencing{}, context);
    }
    return kinds.front();
}

alp::Result<PixelTransform, Dataset::ReadPixelTransformError> Dataset::pixel_transform() const {
    const std::string context = fmt::format("reading the pixel transform of {}", this->name());
    ALP_TRY_CONTEXT(const GeoreferencingKind kind, this->georeferencing_kind(), context);
    return this->pixel_transform(kind);
}

alp::Result<PixelTransform, Dataset::ReadPixelTransformOfKindError> Dataset::pixel_transform(const GeoreferencingKind kind) const {
    const std::string context = fmt::format("reading the pixel transform of {}", this->name());
    if (!std::ranges::contains(this->georeferencing_kinds(), kind)) {
        return alp::make_error(MissingGeoreferencing{}, context);
    }
    // GDAL's pixel transformer gives easting or longitude first, so its system must order its data axes that way.
    const auto system_result = this->srs(kind);
    if (system_result && !srs::is_east_north_axis_order(*system_result)) {
        return alp::make_error(srs::UnsupportedAxisOrder{}, context);
    }
    ALP_TRY_CONTEXT(PixelTransform transform, make_pixel_transform(*this, kind), context);
    return transform;
}

std::string Dataset::name() const {
    if (m_path.has_value()) {
        return m_path->stem().string();
    }
    if (m_gdal_dataset) {
        const char *name = m_gdal_dataset->GetDescription();
        if (name && strlen(name) > 0) {
            return std::string(name);
        }
        else {
            return "Anonymous";
        }
    }
    UNREACHABLE();
}

alp::Result<radix::tile::SrsBounds, Dataset::ComputeBoundsError> Dataset::bounds() const {
    const std::string context = fmt::format("computing the bounds of {}", this->name());
    // GDAL would silently prefer the geotransform over these, even when it is less accurate.
    if (std::ranges::any_of(this->georeferencing_kinds(), [](const GeoreferencingKind kind) { return kind != GeoreferencingKind::GeoTransform; })) {
        return alp::make_error(Unsupported{}, "the dataset has GCP, RPC or geolocation-array georeferencing").context(context);
    }
    ALP_TRY_CONTEXT(const GeoTransform geo_transform, this->geo_transform(), context);

    // https://gdal.org/user/raster_data_model.html
    // gdal has a row/column raster format, where row 0 is the top most row.
    // an affine transform is used to convert row/column into the datasets SRS.

    // We support neither shear nor rotation, nor mirrored or south-up rasters.
    const glm::dvec2 pixel_size = geo_transform.pixel_size();
    if (!geo_transform.is_axis_aligned() || pixel_size.x <= 0.0 || pixel_size.y >= 0.0) {
        return alp::make_error(Unsupported{}, "the geotransform is rotated, sheared or not north-up").context(context);
    }

    const glm::dvec2 north_west = geo_transform.apply(glm::dvec2(0, 0));
    const glm::dvec2 south_east = geo_transform.apply(glm::dvec2(this->raster_size()));
    return radix::tile::SrsBounds{{north_west.x, south_east.y}, {south_east.x, north_west.y}};
}

alp::Result<radix::tile::SrsAndHeightBounds, Dataset::ComputeBoundsError> Dataset::bounds3d(bool approx_ok) const {
    const auto band = this->m_gdal_dataset->GetRasterBand(1);

    // GDAL reports a band without unit metadata as an empty unit type.
    const std::string_view unit_name = band->GetUnitType();
    if (unit_name.empty()) {
        LOG_WARN("Dataset {} does not declare a height unit, assuming metres", this->name());
    } else {
        const std::optional<srs::Unit> unit_opt = srs::parse_length_unit(unit_name);
        if (!unit_opt.has_value() || !unit_opt->is_approx(srs::units::metre)) {
            const auto message = alp::make_context("the height unit '{}' is not metres", unit_name);
            const auto context = alp::make_context("computing the 3D bounds of {}", this->name());
            return alp::make_error(Unsupported{}, message).context(context);
        }
    }

    glm::dvec2 height_range;
    const auto result = band->GetStatistics(approx_ok, false, &height_range.x, &height_range.y, nullptr, nullptr);
    if (result != CE_None) {
        // Twice the geoid's largest departure covers heights above the ellipsoid as well as above the geoid.
        const double margin = 2 * earth::largest_geoid_undulation;
        height_range = earth::elevation_range + glm::dvec2(-margin, margin);
    }

    ALP_TRY(const radix::tile::SrsBounds bounds2d, this->bounds());
    radix::tile::SrsAndHeightBounds bounds3d;
    bounds3d.min = glm::dvec3(bounds2d.min, height_range[0]);
    bounds3d.max = glm::dvec3(bounds2d.max, height_range[1]);
    return bounds3d;
}

alp::Result<std::vector<radix::tile::SrsBounds>, Dataset::ComputeCoverageError> Dataset::geographic_coverage() const {
    const auto context = alp::make_context("computing WGS84 bounds covering dataset {}", this->name());
    ALP_TRY_CONTEXT(const srs::System reference, this->srs(), context);
    ALP_TRY_CONTEXT(const radix::tile::SrsBounds bounds, this->bounds(), context);
    ALP_TRY_CONTEXT(std::vector<radix::tile::SrsBounds> coverage, srs::coverage::compute_geographic(reference, bounds), context);
    return coverage;
}

alp::Result<std::vector<radix::tile::SrsBounds>, Dataset::ComputeCoverageError> Dataset::mercator_coverage() const {
    const auto context = alp::make_context("computing Web Mercator bounds covering dataset {}", this->name());
    ALP_TRY_CONTEXT(const srs::System reference, this->srs(), context);
    ALP_TRY_CONTEXT(const radix::tile::SrsBounds bounds, this->bounds(), context);
    ALP_TRY_CONTEXT(std::vector<radix::tile::SrsBounds> coverage, srs::coverage::compute_mercator(reference, bounds), context);
    return coverage;
}

alp::Result<srs::System, MissingSrs> Dataset::srs() const {
    const OGRSpatialReference *source_srs = m_gdal_dataset->GetSpatialRef();
    for (int layer_index = 0; source_srs == nullptr && layer_index < m_gdal_dataset->GetLayerCount(); ++layer_index) {
        if (OGRLayer *layer = m_gdal_dataset->GetLayer(layer_index)) {
            source_srs = layer->GetSpatialRef();
        }
    }
    if (source_srs == nullptr) {
        return alp::make_error(MissingSrs{this->name()});
    }
    return srs::System(*source_srs);
}

alp::Result<srs::System, MissingSrs> Dataset::srs(const GeoreferencingKind kind) const {
    if (!std::ranges::contains(this->georeferencing_kinds(), kind)) {
        return alp::make_error(MissingSrs{this->name()});
    }
    const OGRSpatialReference *system = nullptr;
    switch (kind) {
    case GeoreferencingKind::GeoTransform:
        system = this->m_gdal_dataset->GetSpatialRef();
        break;
    case GeoreferencingKind::GcpPolynomial:
        system = this->m_gdal_dataset->GetGCPSpatialRef();
        break;
    case GeoreferencingKind::Rpc:
        return srs::wgs84();
    case GeoreferencingKind::GeolocationArray: {
        const char *definition = this->m_gdal_dataset->GetMetadataItem("SRS", "GEOLOCATION");
        if (definition == nullptr) {
            return alp::make_error(MissingSrs{this->name()});
        }
        auto system_result = srs::from_user_input(definition);
        if (!system_result) {
            return alp::make_error(MissingSrs{this->name()}, std::move(system_result).error());
        }
        return std::move(system_result).value();
    }
    }
    if (system == nullptr) {
        return alp::make_error(MissingSrs{this->name()});
    }
    return srs::System(*system);
}

glm::uvec2 Dataset::raster_size() const {
    const int width = m_gdal_dataset->GetRasterXSize();
    const int height = m_gdal_dataset->GetRasterYSize();
    ASSERT(width >= 0);
    ASSERT(height >= 0);
    return glm::uvec2(width, height);
}

uint32_t Dataset::band_count() const {
    const auto n = m_gdal_dataset->GetRasterCount();
    ASSERT(n >= 0);
    return static_cast<uint32_t>(n);
}

GDALDataset *Dataset::as_gdal() {
    return m_gdal_dataset.get();
}
const GDALDataset *Dataset::as_gdal() const {
    return m_gdal_dataset.get();
}

} // namespace gis
