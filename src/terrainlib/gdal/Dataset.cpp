/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 alpinemaps.org
 * Copyright (C) 2022 Adam Celarek <family name at cg tuwien ac at>
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

#include "gdal/Dataset.h"

#include <algorithm>
#include <cassert>
#include <memory>
#include <mutex>
#include <stdexcept>

#include <cpl_error.h>
#include <fmt/format.h>
#include <gdal_priv.h>
#include <libassert/assert.hpp>
#include <ogrsf_frmts.h>

#include "ctb/Grid.hpp"
#include "error/log.h"
#include "error/try.h"
#include "error/unwrap.h"
#include "error/utils.h"
#include "gdal/init.h"
#include "log.h"
#include "srs/transform.h"
#include "srs/transform_bounds.h"

namespace gdal {

void DatasetDeleter::operator()(GDALDataset *dataset) const noexcept {
    GDALClose(dataset);
}

[[nodiscard]]
static alp::Result<GDALDataset *, CplError> open_gdal_dataset(const std::filesystem::path &path, unsigned int flags) {
    initialize_once();
    const std::string path_str = path.string();
    static std::mutex gdal_open_mutex;
    const std::lock_guard lock(gdal_open_mutex);
    // GDAL records why an open failed only with GDAL_OF_VERBOSE_ERROR, and a reset keeps an older error from standing in for it.
    CPLErrorReset();
    GDALDataset *dataset = static_cast<GDALDataset *>(GDALOpenEx(path_str.c_str(), flags | GDAL_OF_VERBOSE_ERROR, nullptr, nullptr, nullptr));
    if (dataset == nullptr) {
        return alp::make_unexpected(CplError::get_last_error(), fmt::format("opening a dataset at {}", path_str));
    }
    return dataset;
}

alp::Result<Dataset, CplError> Dataset::open_raster(std::filesystem::path path) {
    ALP_TRY(GDALDataset *dataset, open_gdal_dataset(path, GDAL_OF_RASTER));
    return Dataset(path, dataset);
}
alp::Result<Dataset, CplError> Dataset::open_vector(std::filesystem::path path) {
    ALP_TRY(GDALDataset *dataset, open_gdal_dataset(path, GDAL_OF_VECTOR));
    return Dataset(path, dataset);
}
alp::Result<std::shared_ptr<Dataset>, CplError> Dataset::open_shared_raster(std::filesystem::path path) {
    ALP_TRY(GDALDataset *dataset, open_gdal_dataset(path, GDAL_OF_RASTER));
    return std::make_shared<Dataset>(Dataset(path, dataset));
}

Dataset::Dataset(std::filesystem::path path) {
    const alp::Result<GDALDataset *, CplError> result = open_gdal_dataset(path, 0);
    if (result.has_value()) {
        m_path = path;
        m_gdal_dataset.reset(result.value());
    } else {
        LOG_UNEXPECTED(result.error());
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
Dataset::Dataset(const std::filesystem::path path, GDALDataset *dataset) : Dataset(dataset) {
    m_path = path;
}

Dataset Dataset::clone() {
    LOG_TRACE("Cloning dataset {}.", m_path.has_value() ? m_path->string() : "unknown");
    if (!m_path.has_value()) {
        LOG_ERROR("Cannot clone dataset.");
        throw std::runtime_error("Cannot clone dataset.");
    }
    return Dataset(this->m_path.value());
}

alp::Result<PixelTransform, PixelTransformError> Dataset::pixel_transform() const {
    ALP_TRY_CONTEXT(
        PixelTransform transform,
        PixelTransform::from_dataset(*this->m_gdal_dataset),
        fmt::format("reading the pixel transform of {}", this->name()));
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

radix::tile::SrsBounds Dataset::bounds() const {
    const PixelTransform transform = alp::unwrap(this->pixel_transform());

    // https://gdal.org/user/raster_data_model.html
    // gdal has a row/column raster format, where row 0 is the top most row.
    // an affine transform is used to convert row/column into the datasets SRS.
    // computing bounds is going first from row/column to dataset SRS and then to target SRS

    // we don't support sheering or rotation for now
    if (!transform.is_axis_aligned()) {
        throw std::runtime_error("Dataset geo transform contains sheering or rotation. This is not supported!");
    }

    const glm::dvec2 north_west = transform.apply(glm::dvec2(0, 0));
    const glm::dvec2 south_east = transform.apply(glm::dvec2(widthInPixels(), heightInPixels()));
    return {{north_west.x, south_east.y}, {south_east.x, north_west.y}};
}

radix::tile::SrsAndHeightBounds Dataset::bounds3d(bool approx_ok) const {
    const auto band = this->m_gdal_dataset->GetRasterBand(1);

    glm::dvec2 height_range;
    const auto result = band->GetStatistics(approx_ok, false, &height_range.x, &height_range.y, nullptr, nullptr);
    if (result != CE_None) {
        const char *unit = band->GetUnitType();
        ASSERT(unit != nullptr);
        ASSERT(strcmp(unit, "m") == 0 || strcmp(unit, "meters") == 0);
        height_range = {-11000.0, 9000.0}; // Mariana Trench and Mount Everest
    }

    const auto bounds2d = this->bounds();
    radix::tile::SrsAndHeightBounds bounds3d;
    bounds3d.min = glm::dvec3(bounds2d.min, height_range[0]);
    bounds3d.max = glm::dvec3(bounds2d.max, height_range[1]);
    return bounds3d;
}

radix::tile::SrsBounds Dataset::bounds(const srs::System &targetSrs) const {
    const auto l_bounds = bounds();
    if (l_bounds.width() <= 0.0)
        throw std::runtime_error("west coordinate > east coordinate. This is not supported.");
    if (l_bounds.height() <= 0.0)
        throw std::runtime_error("south coordinate > north coordinate. This is not supported.");

    return alp::unwrap(srs::encompassing_bounds_transfer(srs(), targetSrs, l_bounds, 2000));
}

srs::System Dataset::srs() const {
    const srs::System *source_srs = m_gdal_dataset->GetSpatialRef();
    for (int layer_index = 0; source_srs == nullptr && layer_index < m_gdal_dataset->GetLayerCount(); ++layer_index) {
        if (OGRLayer *layer = m_gdal_dataset->GetLayer(layer_index)) {
            source_srs = layer->GetSpatialRef();
        }
    }
    if (source_srs == nullptr) {
        throw std::runtime_error("The source dataset does not have a spatial reference system assigned");
    }
    auto srs = *source_srs;
    srs.SetAxisMappingStrategy(OAMS_TRADITIONAL_GIS_ORDER);
    return srs;
}

unsigned Dataset::widthInPixels() const {
    return ctb::i_pixel(m_gdal_dataset->GetRasterXSize());
}

unsigned Dataset::heightInPixels() const {
    return ctb::i_pixel(m_gdal_dataset->GetRasterYSize());
}

double Dataset::widthInPixels(const radix::tile::SrsBounds &bounds, const srs::System &bounds_srs) const {
    return bounds.width() / pixelWidthIn(bounds_srs);
}

double Dataset::heightInPixels(const radix::tile::SrsBounds &bounds, const srs::System &bounds_srs) const {
    return bounds.height() / pixelHeightIn(bounds_srs);
}

unsigned Dataset::n_bands() const {
    const auto n = m_gdal_dataset->GetRasterCount();
    DEBUG_ASSERT(n >= 0);
    return unsigned(n);
}

GDALDataset *Dataset::as_gdal() {
    return m_gdal_dataset.get();
}
const GDALDataset *Dataset::as_gdal() const {
    return m_gdal_dataset.get();
}

double Dataset::gridResolution(const srs::System &target_srs) const {
    return std::min(pixelWidthIn(target_srs), pixelHeightIn(target_srs));
}

double Dataset::pixelWidthIn(const srs::System &target_srs) const {
    const auto b0 = bounds();
    const auto b1 = alp::unwrap(srs::non_exact_bounds_transform(b0, srs(), target_srs));
    return b1.width() / widthInPixels();
}

double Dataset::pixelHeightIn(const srs::System &target_srs) const {
    const auto b = alp::unwrap(srs::non_exact_bounds_transform(bounds(), srs(), target_srs));
    return b.height() / heightInPixels();
}

glm::dvec2 Dataset::pixelSizeIn(const srs::System &target_srs) const {
    return glm::dvec2(this->pixelWidthIn(target_srs), this->pixelHeightIn(target_srs));
}

} // namespace gdal
