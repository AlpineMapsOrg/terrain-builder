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

#pragma once

#include <memory>
#include <string>
#include <optional>
#include <filesystem>

#include <glm/vec2.hpp>
#include <radix/tile.h>

#include "containers/UniqueResource.h"
#include "error/Result.h"
#include "gdal/CplError.h"
#include "gdal/PixelTransform.h"
#include "log.h"
#include "srs/System.h"

class GDALDataset;

namespace gdal {

struct DatasetDeleter {
    void operator()(GDALDataset *dataset) const noexcept;
};

class Dataset {
public:
    Dataset(std::filesystem::path path);
    Dataset(GDALDataset* dataset); // takes over ownership
    [[nodiscard]]
    static alp::Result<Dataset, CplError> open_raster(std::filesystem::path path);
    [[nodiscard]]
    static alp::Result<Dataset, CplError> open_vector(std::filesystem::path path);
    [[nodiscard]]
    static alp::Result<std::shared_ptr<Dataset>, CplError> open_shared_raster(std::filesystem::path path);
    Dataset clone();

    [[nodiscard]] std::string name() const;

    [[nodiscard]] alp::Result<PixelTransform, PixelTransformError> pixel_transform() const;

    [[nodiscard]] radix::tile::SrsBounds bounds() const;
    [[nodiscard]] radix::tile::SrsAndHeightBounds bounds3d(bool approx_ok = false) const;
    [[nodiscard]] radix::tile::SrsBounds bounds(const srs::System &targetSrs) const;
    [[nodiscard]] srs::System srs() const;
    [[nodiscard]] unsigned int widthInPixels() const;
    [[nodiscard]] unsigned int heightInPixels() const;
    [[nodiscard]] double widthInPixels(const radix::tile::SrsBounds &bounds, const srs::System &bounds_srs) const;
    [[nodiscard]] double heightInPixels(const radix::tile::SrsBounds &bounds, const srs::System &bounds_srs) const;
    [[nodiscard]] unsigned int n_bands() const;
    [[nodiscard]] GDALDataset *as_gdal();
    [[nodiscard]] const GDALDataset *as_gdal() const;

    [[nodiscard]] double gridResolution(const srs::System &target_srs) const;
    [[nodiscard]] double pixelWidthIn(const srs::System &target_srs) const;
    [[nodiscard]] double pixelHeightIn(const srs::System &target_srs) const;
    [[nodiscard]] glm::dvec2 pixelSizeIn(const srs::System &target_srs) const;

private:
    Dataset(const std::filesystem::path path, GDALDataset *dataset);
    UniquePointer<GDALDataset, DatasetDeleter> m_gdal_dataset;
    std::optional<std::filesystem::path> m_path;
};

} // namespace gdal
