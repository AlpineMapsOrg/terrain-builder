/*****************************************************************************
 * AlpineMaps.org
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

#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <string>
#include <utility>

#include <cpl_string.h>
#include <fmt/format.h>
#include <gdal_alg.h>
#include <gdal_priv.h>
#include <libassert/assert.hpp>

#include "error/try.h"
#include "error/utils.h"
#include "gis/CplError.h"
#include "gis/Dataset.h"
#include "gis/GeoTransform.h"
#include "gis/PixelTransform.h"

namespace gis {

namespace {

// The SRC_METHOD value that makes GDAL use `kind`.
const char* to_gdal_method(const GeoreferencingKind kind) {
    switch (kind) {
    case GeoreferencingKind::GeoTransform:
        return "GEOTRANSFORM";
    case GeoreferencingKind::GcpPolynomial:
        return "GCP_POLYNOMIAL";
    case GeoreferencingKind::Rpc:
        return "RPC";
    case GeoreferencingKind::GeolocationArray:
        return "GEOLOC_ARRAY";
    }
    UNREACHABLE("invalid kind", kind);
}

// Fails if the geotransform of `dataset` is not valid.
alp::Result<void, InvalidGeoreferencing> check_geo_transform(GDALDataset &dataset) {
    std::array<double, 6> coefficients;
    if (dataset.GetGeoTransform(coefficients.data()) != CE_None) {
        return {};
    }
    // GDAL accepts a geotransform with a coefficient that is not finite.
    auto geo_transform_result = GeoTransform::from_coefficients(coefficients);
    if (!geo_transform_result) {
        return alp::make_error(InvalidGeoreferencing{}, std::move(geo_transform_result).error());
    }
    return {};
}

// Maps like GDALGenImgProjTransform, but gives 2D points height 0.
int transform_rpc_pixels(void *transformer, const int dst_to_src, const int count, double *xs, double *ys, double *zs, int *successes) {
    // GDAL's RPC transformer rejects a null z for the inverse.
    detail::CoordinateBuffer heights;
    if (zs == nullptr) {
        heights.resize(count);
        zs = heights.data();
    }
    return GDALGenImgProjTransform(transformer, dst_to_src, count, xs, ys, zs, successes);
}

} // namespace

alp::Result<PixelTransform, InvalidGeoreferencing> make_pixel_transform(const Dataset &dataset, const GeoreferencingKind kind) {
    // GDAL takes the dataset mutably, as Dataset's own const methods do through its pointer.
    GDALDataset &gdal_dataset = const_cast<GDALDataset &>(*dataset.as_gdal());
    if (kind == GeoreferencingKind::GeoTransform) {
        ALP_TRY(check_geo_transform(gdal_dataset));
    }

    CPLStringList options;
    options.SetNameValue("SRC_METHOD", to_gdal_method(kind));
    auto [handle, gdal_error] = capture_error([&] {
        return GDALCreateGenImgProjTransformer2(GDALDataset::ToHandle(&gdal_dataset), nullptr, options.List());
    });
    if (handle == nullptr) {
        const std::string context = fmt::format("creating a transformer from the dataset's {}", kind);
        return alp::make_error(InvalidGeoreferencing{}, context, std::move(gdal_error));
    }
    std::shared_ptr<void> argument(handle, &GDALDestroyGenImgProjTransformer);
    const TransformerFunction function = (kind == GeoreferencingKind::Rpc) ? &transform_rpc_pixels : &GDALGenImgProjTransform;
    const auto system_result = dataset.srs(kind);
    std::string source_name = fmt::format("pixels of {}", dataset.name());
    std::string target_name = system_result ? fmt::format("{}", *system_result) : "unknown srs";
    Transform transform = make_transform(function, std::move(argument), std::move(source_name), std::move(target_name));
    return PixelTransform(std::move(transform), kind);
}

PixelTransform::PixelTransform(Transform transform, const GeoreferencingKind kind) : Transform(std::move(transform)), _kind(kind) {}

PixelTransform PixelTransform::inverse() const {
    return PixelTransform(this->Transform::inverse(), this->_kind);
}

} // namespace gis
