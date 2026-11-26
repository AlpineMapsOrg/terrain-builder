/*******************************************************************************
 * Copyright 2014 GeoData <geodata@soton.ac.uk>
 * Copyright 2022 Adam Celarek-Litofcenko
 * Copyright (C) 2022 Martin Braunsperger
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License.  You may obtain a copy
 * of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *******************************************************************************/

#include "gis/DatasetReader.h"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <memory>
#include <optional>
#include <ranges>
#include <source_location>
#include <span>
#include <string>
#include <utility>
#include <vector>

#include <cpl_error.h>
#include <cpl_string.h>
#include <fmt/format.h>
#include <gdal.h>
#include <gdal_alg.h>
#include <gdal_priv.h>
#include <gdalwarper.h>
#include <glm/gtx/component_wise.hpp>
#include <libassert/assert.hpp>
#include <ogr_spatialref.h>
#include <radix/raster.h>
#include <vrtdataset.h>

#include "enumerate.h"
#include "error/try.h"
#include "error/utils.h"
#include "geometry/geometry.h"
#include "gis/CplError.h"
#include "gis/GeoTransform.h"
#include "gis/OgrError.h"
#include "gis/georeferencing.h"
#include "gis/srs/Kind.h"
#include "gis/srs/Unit.h"
#include "gis/srs/classify.h"
#include "gis/srs/well_known.h"
#include "glm_utils.h"

namespace gis {

namespace {

// The spatial reference system of `projection`.
srs::System target_srs(const Projection projection) {
    switch (projection) {
    case Projection::WebMercator:
        return srs::webmercator();
    case Projection::Geographic:
        return srs::wgs84();
    }
    UNREACHABLE("invalid projection", projection);
}

// Maps the error of `result` to OutOfMemory if OGR ran out of memory, otherwise to `code`, with the OGR error as source.
template <typename T, alp::ErrorCode Code>
alp::Result<T, alp::Error<Code, OutOfMemory>> map_ogr_error(alp::Result<T, OgrError> result, Code code, const std::source_location location = std::source_location::current()) {
    auto memory_result = alp::map_code(std::move(result), OGRERR_NOT_ENOUGH_MEMORY, OutOfMemory{}, location);
    return alp::map_error<OgrError>(std::move(memory_result), std::move(code), location);
}

// Maps the error of `result` to OutOfMemory if GDAL ran out of memory, otherwise to `code`, with the GDAL error as source.
template <typename T, alp::ErrorCode Code>
alp::Result<T, alp::Error<Code, OutOfMemory>> map_cpl_error(alp::Result<T, CplError> result, Code code, const std::source_location location = std::source_location::current()) {
    auto memory_result = alp::map_code(std::move(result), CPLE_OutOfMemory, OutOfMemory{}, location);
    return alp::map_error<CplError>(std::move(memory_result), std::move(code), location);
}

// Translates the error GDAL recorded for a failed call into OutOfMemory if GDAL ran out of memory, otherwise into `code`.
// The result has `context`, and the recorded error, if any, as its source.
template <alp::ErrorCode Code>
alp::Error<Code, OutOfMemory> translate_cpl_error(Code code, const alp::Context auto &context, std::optional<CplError> cpl_error, const std::source_location location = std::source_location::current()) {
    if (cpl_error && cpl_error->code() == CPLE_OutOfMemory) {
        return alp::make_error(OutOfMemory{}, context, std::move(*cpl_error), location);
    } else {
        return alp::make_error(std::move(code), context, std::move(cpl_error), location);
    }
}

struct WarpOptionsDeleter {
    void operator()(GDALWarpOptions *options) const noexcept {
        GDALDestroyWarpOptions(options);
    }
};

struct ApproxTransformerDeleter {
    void operator()(void *transformer) const noexcept {
        GDALDestroyApproxTransformer(transformer);
    }
};

using GdalWarpOptionsPtr = UniquePointer<GDALWarpOptions, WarpOptionsDeleter>;
// TODO: let gis::Transform serve as GDAL's transformer in warps and similar calls, so these handles need no owners of their own.
using ApproxTransformer = UniquePointer<void, ApproxTransformerDeleter>;

// Maximum approximation error in source pixels.
constexpr double approximation_error = 0.125;

template <typename Pixel>
constexpr GDALDataType source_type = std::is_same_v<Pixel, float> ? GDT_Float32 : GDT_Byte;

static_assert(sizeof(glm::u8vec3) == 3 && alignof(glm::u8vec3) == 1 && std::is_trivially_copyable_v<glm::u8vec3>);
static_assert(offsetof(glm::u8vec3, x) == 0 && offsetof(glm::u8vec3, y) == 1 && offsetof(glm::u8vec3, z) == 2);

// The selected bands, followed by an alpha band exposing their effective masks unless all of them are valid everywhere.
struct Source {
    std::unique_ptr<VRTDataset> dataset;
    int32_t alpha_band = 0;
};

// Exposes `bands` of `dataset` as a source for the warp.
alp::Result<Source, alp::Error<UnsupportedMaskType, WindowReadError, OutOfMemory>> make_source(GDALDataset &dataset, const std::span<const uint32_t> bands) {
    std::vector<GDALRasterBand *> inputs;
    for (const uint32_t band : bands) {
        inputs.push_back(dataset.GetRasterBand(static_cast<int>(band)));
    }

    for (const auto [band, input] : std::views::zip(bands, inputs)) {
        if (input->GetMaskBand()->GetRasterDataType() != GDT_Byte) {
            return alp::make_error(UnsupportedMaskType{.band = band});
        }
    }

    Source source{std::make_unique<VRTDataset>(dataset.GetRasterXSize(), dataset.GetRasterYSize())};
    for (const auto [band, input] : std::views::zip(bands, inputs)) {
        const auto expose_context = alp::make_context("exposing source band {}", band);
        const CPLErr band_status = source.dataset->AddBand(input->GetRasterDataType(), nullptr);
        ALP_TRY_CONTEXT(map_cpl_error(to_result(band_status), WindowReadError{}), expose_context);
        // A VRT band added without options is a VRTSourcedRasterBand.
        auto *output = static_cast<VRTSourcedRasterBand *>(source.dataset->GetRasterBand(source.dataset->GetRasterCount()));
        const CPLErr simple_source_status = output->AddSimpleSource(input);
        ALP_TRY_CONTEXT(map_cpl_error(to_result(simple_source_status), WindowReadError{}), expose_context);
    }

    const bool is_all_valid = std::ranges::all_of(inputs, [](GDALRasterBand *input) { return (input->GetMaskFlags() & GMF_ALL_VALID) != 0; });
    if (is_all_valid) {
        return source;
    }

    // The mask of a single band, or a mask shared by all bands, is exposed once. Otherwise the alpha is the minimum of the masks, which is their AND for binary masks; GDAL's min takes at least two sources.
    // The alpha band must not have NoData, which min skips.
    const bool is_mask_shared = inputs.size() == 1 || std::ranges::all_of(inputs, [](GDALRasterBand *input) { return (input->GetMaskFlags() & GMF_PER_DATASET) != 0; });
    CPLStringList options;
    if (!is_mask_shared) {
        options.SetNameValue("subclass", "VRTDerivedRasterBand");
        options.SetNameValue("PixelFunctionType", "min");
    }
    const CPLErr alpha_status = source.dataset->AddBand(GDT_Byte, options.List());
    ALP_TRY_CONTEXT(map_cpl_error(to_result(alpha_status), WindowReadError{}), "creating the source validity band");
    source.alpha_band = source.dataset->GetRasterCount();
    auto *alpha = static_cast<VRTSourcedRasterBand *>(source.dataset->GetRasterBand(source.alpha_band));

    const std::span<GDALRasterBand *> mask_inputs = is_mask_shared ? std::span(inputs).first(1) : std::span(inputs);
    for (GDALRasterBand *input : mask_inputs) {
        const CPLErr mask_source_status = alpha->AddMaskBandSource(input);
        ALP_TRY_CONTEXT(map_cpl_error(to_result(mask_source_status), WindowReadError{}), "exposing a source band mask");
    }
    return source;
}

// Warps all source bands at once into `samples`, with the destination alpha as validity.
template <typename Pixel>
alp::Result<void, alp::Error<MissingDriver, WindowReadError, OutOfMemory>> warp(const Source &source, const ApproxTransformer &transformer, typename DatasetReader<Pixel>::Samples &samples) {
    constexpr int32_t channel_count = DatasetReader<Pixel>::channel_count;
    const glm::uvec2 size = samples.data.size();
    const int32_t width = static_cast<int32_t>(size.x);
    const int32_t height = static_cast<int32_t>(size.y);

    GDALDriver *driver = GetGDALDriverManager()->GetDriverByName("MEM");
    if (driver == nullptr) {
        return alp::make_error(MissingDriver{.name = "MEM"});
    }
    // The destination bands only borrow the sample buffers.
    auto [raw_destination, destination_error] = capture_error([&] { return driver->Create("", width, height, 0, GDT_Unknown, nullptr); });
    UniquePointer<GDALDataset, DatasetDeleter> destination(raw_destination);
    if (!destination) {
        return translate_cpl_error(WindowReadError{}, "creating the warp destination", std::move(destination_error));
    }
    const auto add_band = [&](const GDALDataType type, std::byte *data, const GSpacing pixel_offset) {
        const GSpacing line_offset = pixel_offset * size.x;
        CPLStringList options;
        options.SetNameValue("DATAPOINTER", fmt::format("{}", static_cast<const void *>(data)).c_str());
        options.SetNameValue("PIXELOFFSET", fmt::format("{}", pixel_offset).c_str());
        options.SetNameValue("LINEOFFSET", fmt::format("{}", line_offset).c_str());
        return destination->AddBand(type, options.List());
    };

    std::byte *data = samples.data.bytes().data();
    const size_t channel_size = GDALGetDataTypeSizeBytes(source_type<Pixel>);
    for (size_t channel = 0; channel < channel_count; channel++) {
        const CPLErr data_status = add_band(source_type<Pixel>, data + channel * channel_size, sizeof(Pixel));
        ALP_TRY_CONTEXT(map_cpl_error(to_result(data_status), WindowReadError{}), "attaching the warp destination data");
    }
    const CPLErr validity_status = add_band(GDT_Byte, samples.valid.bytes().data(), 1);
    ALP_TRY_CONTEXT(map_cpl_error(to_result(validity_status), WindowReadError{}), "attaching the warp destination validity");

    GdalWarpOptionsPtr options(GDALCreateWarpOptions());
    options->hSrcDS = source.dataset.get();
    options->hDstDS = destination.get();
    GDALWarpInitDefaultBandMapping(options.get(), channel_count);
    options->nSrcAlphaBand = source.alpha_band;
    options->nDstAlphaBand = channel_count + 1;
    options->eResampleAlg = GRA_Lanczos;
    options->eWorkingDataType = source_type<Pixel>;
    options->pfnTransformer = GDALApproxTransform;
    options->pTransformerArg = transformer.get();
    // A window without valid source coverage is an empty read rather than a failure.
    options->papszWarpOptions = CSLSetNameValue(options->papszWarpOptions, "INIT_DEST", "0");
    options->papszWarpOptions = CSLSetNameValue(options->papszWarpOptions, "ERROR_OUT_IF_EMPTY_SOURCE_WINDOW", "FALSE");
    // Callers parallelize across readers.
    options->papszWarpOptions = CSLSetNameValue(options->papszWarpOptions, "NUM_THREADS", "1");
    options->papszWarpOptions = CSLSetNameValue(options->papszWarpOptions, "SRC_ALPHA_MAX", "255");
    options->papszWarpOptions = CSLSetNameValue(options->papszWarpOptions, "DST_ALPHA_MAX", "255");

    GDALWarpOperation operation;
    const CPLErr initialize_status = operation.Initialize(options.get());
    ALP_TRY_CONTEXT(map_cpl_error(to_result(initialize_status), WindowReadError{}), "initializing the warp of the source bands");
    const CPLErr warp_status = operation.ChunkAndWarpImage(0, 0, width, height);
    ALP_TRY_CONTEXT(map_cpl_error(to_result(warp_status), WindowReadError{}), "warping the source bands");
    return {};
}

} // namespace

template <typename Pixel>
void DatasetReader<Pixel>::TransformerDeleter::operator()(Transformer *transformer) const noexcept {
    GDALDestroyGenImgProjTransformer(transformer);
}

template <typename Pixel>
DatasetReader<Pixel>::DatasetReader(std::shared_ptr<Dataset> dataset, const Projection projection, const std::array<uint32_t, channel_count> bands, const Pixel default_pixel, UniquePointer<Transformer, TransformerDeleter> transformer)
    : m_dataset(std::move(dataset)),
      m_projection(projection),
      m_bands(bands),
      m_default_pixel(default_pixel),
      m_transformer(std::move(transformer)) {}

template <typename Pixel>
alp::Result<DatasetReader<Pixel>, typename DatasetReader<Pixel>::MakeError> DatasetReader<Pixel>::make(
    std::shared_ptr<Dataset> dataset, const Projection projection, const std::array<uint32_t, channel_count> bands, const Pixel default_pixel) {
    ASSERT(dataset);
    if constexpr (std::is_same_v<Pixel, float>) {
        ASSERT(std::isfinite(default_pixel), "default pixel must be finite");
    }
    const auto context = alp::make_context("creating a reader for dataset {}", dataset->name());

    GDALDataset &gdal_dataset = *dataset->as_gdal();
    const uint32_t band_count = dataset->band_count();
    for (const uint32_t band : bands) {
        if (band == 0 || band > band_count) {
            return alp::make_error(MissingBand{.band = band, .band_count = band_count}, context);
        }
        const GDALDataType type = gdal_dataset.GetRasterBand(static_cast<int>(band))->GetRasterDataType();
        if (type != source_type<Pixel>) {
            return alp::make_error(UnsupportedBandType{.band = band, .type = type, .required = source_type<Pixel>}, context);
        }
    }

    ALP_TRY_CONTEXT(const GeoreferencingKind kind, dataset->georeferencing_kind(), context);
    ALP_TRY_CONTEXT(const srs::System source, dataset->srs(kind), context);
    const srs::System target = target_srs(projection);
    CPLStringList transformer_options;
    if (!source.is_same(target)) {
        const bool is_longitude_in_degrees = source.kind() == srs::Kind::Geographic && srs::has_horizontal_axes_in(source, srs::units::degree);
        if (is_longitude_in_degrees) {
            // GDAL wraps longitudes around the source centre only for sources up to about 360 degrees wide.
            // Wider ones need it too, e.g. -0.5..360.5, as reprojected longitudes lie in -180..180.
            // The centre comes from the geotransform, so GDAL has to map the pixels by it as well.
            ALP_TRY_CONTEXT(const GeoTransform geo_transform, dataset->geo_transform(), context);
            transformer_options.SetNameValue("SRC_METHOD", "GEOTRANSFORM");
            const glm::dvec2 raster_size(dataset->raster_size());
            const std::array corners{
                geo_transform.apply(glm::dvec2(0, 0)),
                geo_transform.apply(glm::dvec2(raster_size.x, 0)),
                geo_transform.apply(glm::dvec2(0, raster_size.y)),
                geo_transform.apply(raster_size),
            };
            const auto [west, east] = std::ranges::minmax(corners, {}, &glm::dvec2::x);
            const std::string centre = fmt::format("{}", (west.x + east.x) / 2);

            OGRSpatialReference centred_srs(source.as_gdal());
            const OGRErr extension_status = centred_srs.SetExtension("GEOGCS", "CENTER_LONG", centre.c_str());
            auto extension_result = map_ogr_error(to_result(extension_status), UnsupportedReprojection{});
            ALP_TRY_CONTEXT(std::move(extension_result), fmt::format("setting the central longitude of the system of dataset {}", dataset->name()));
            // CENTER_LONG is a WKT1 extension.
            const srs::System centred_source(std::move(centred_srs));
            auto wkt_result = map_ogr_error(centred_source.to_wkt(srs::WktVersion::Gdal1), UnsupportedReprojection{});
            ALP_TRY_CONTEXT(const std::string source_wkt, std::move(wkt_result), fmt::format("exporting the system of dataset {} as WKT1", dataset->name()));
            transformer_options.SetNameValue("SRC_SRS", source_wkt.c_str());
        }
        auto target_wkt_result = map_ogr_error(target.to_wkt(srs::WktVersion::Iso2019), UnsupportedReprojection{});
        ALP_TRY_CONTEXT(const std::string target_wkt, std::move(target_wkt_result), "exporting the target system as WKT2");
        transformer_options.SetNameValue("DST_SRS", target_wkt.c_str());
    }

    // Creates both transformation directions, so reads rebuild no PROJ state.
    auto [raw_transformer, gdal_error] = capture_error([&] {
        return GDALCreateGenImgProjTransformer2(&gdal_dataset, nullptr, transformer_options.List());
    });
    UniquePointer<Transformer, TransformerDeleter> transformer(static_cast<Transformer *>(raw_transformer));
    if (!transformer) {
        return translate_cpl_error(UnsupportedReprojection{}, "creating the transformer", std::move(gdal_error)).context(context);
    }
    return DatasetReader(std::move(dataset), projection, bands, default_pixel, std::move(transformer));
}

template <typename Pixel>
alp::Result<typename DatasetReader<Pixel>::Samples, typename DatasetReader<Pixel>::ReadError> DatasetReader<Pixel>::read(const radix::tile::SrsBounds &bounds, const glm::uvec2 size) {
    ASSERT(size.x > 0 && size.y > 0, "read size is empty");
    ASSERT(std::in_range<int32_t>(size.x) && std::in_range<int32_t>(size.y), "read size exceeds GDAL dimensions");
    const uint64_t pixel_count = static_cast<uint64_t>(size.x) * size.y;
    ASSERT(pixel_count <= std::vector<Pixel>().max_size(), "read size exceeds raster capacity");
    const glm::dvec2 pixel_size = (bounds.max - bounds.min) / glm::dvec2(size);
    const bool are_bounds_finite = geometry::is_finite(bounds) && ::is_finite(pixel_size);
    const bool are_bounds_nonempty = glm::all(glm::greaterThan(pixel_size, glm::dvec2(0)));
    ASSERT(are_bounds_finite && are_bounds_nonempty, "read bounds must be finite and nonempty");

    // The coefficients cannot fail to invert, as the pixel size is finite and positive.
    const std::array<double, 6> geo_transform{bounds.min.x, pixel_size.x, 0, bounds.max.y, 0, -pixel_size.y};
    GDALSetGenImgProjTransformerDstGeoTransform(this->m_transformer.get(), geo_transform.data());
    auto [raw_transformer, transformer_error] = capture_error([&] {
        return GDALCreateApproxTransformer(GDALGenImgProjTransform, this->m_transformer.get(), approximation_error);
    });
    ApproxTransformer transformer(raw_transformer);
    if (!transformer) {
        return translate_cpl_error(WindowReadError{}, "creating the approximate transformer", std::move(transformer_error));
    }

    ALP_TRY_CONTEXT(const Source source, make_source(*this->m_dataset->as_gdal(), this->m_bands), fmt::format("preparing dataset {}", this->m_dataset->name()));
    Samples result{radix::Raster<Pixel>(size), radix::Raster<uint8_t>(size)};
    ALP_TRY_CONTEXT(warp<Pixel>(source, transformer, result), fmt::format("reading dataset {}", this->m_dataset->name()));

    // Neither destination NoData nor INIT_DEST can replace this pass: GDAL alters valid pixels equal to NoData and writes pixels with zero alpha.
    for (auto [pixel, validity] : std::views::zip(result.data.buffer(), result.valid.buffer())) {
        bool is_valid = validity != 0;
        if constexpr (std::is_same_v<Pixel, float>) {
            is_valid = is_valid && std::isfinite(pixel);
        }
        if (!is_valid) {
            pixel = this->m_default_pixel;
            validity = 0;
        }
    }
    return result;
}

template <typename Pixel>
alp::Result<glm::dvec2, typename DatasetReader<Pixel>::PixelSizeError> DatasetReader<Pixel>::min_pixel_size(const radix::tile::SrsBounds &bounds) const {
    constexpr uint32_t samples_per_axis = 5;
    constexpr double step_fraction = 1e-3;

    auto coverage_result = this->m_projection == Projection::WebMercator ? this->m_dataset->mercator_coverage() : this->m_dataset->geographic_coverage();
    ALP_TRY_CONTEXT(const std::vector<radix::tile::SrsBounds> coverage, std::move(coverage_result), "computing the dataset coverage");
    // Each sample is a point followed by its x and y neighbours. Steps point inwards, so neighbours stay within the coverage and never cross the antimeridian.
    std::vector<glm::dvec2> points;
    std::vector<glm::dvec2> steps;
    for (const auto &box : coverage) {
        const radix::tile::SrsBounds region{glm::max(bounds.min, box.min), glm::min(bounds.max, box.max)};
        if (region.min.x >= region.max.x || region.min.y >= region.max.y) {
            continue;
        }
        const glm::dvec2 centre = (region.min + region.max) / 2.0;
        const glm::dvec2 step = step_fraction * (region.max - region.min);
        for (uint32_t y = 0; y < samples_per_axis; y++) {
            for (uint32_t x = 0; x < samples_per_axis; x++) {
                const glm::dvec2 point = glm::mix(region.min, region.max, glm::dvec2(x, y) / (samples_per_axis - 1.0));
                const glm::dvec2 inward_step = glm::mix(step, -step, glm::greaterThan(point, centre));
                points.insert(points.end(), {point, point + glm::dvec2(inward_step.x, 0), point + glm::dvec2(0, inward_step.y)});
                steps.push_back(inward_step);
            }
        }
    }
    if (steps.empty()) {
        return alp::make_error(OutsideCoverage{}, fmt::format("intersecting the bounds with the coverage of dataset {}", this->m_dataset->name()));
    }

    // The geotransform maps pixels into this system, so the samples are measured in it.
    ALP_TRY_CONTEXT(const srs::System source_srs, this->m_dataset->srs(GeoreferencingKind::GeoTransform), fmt::format("reading the spatial reference of dataset {}", this->m_dataset->name()));
    ALP_TRY_CONTEXT(const srs::Transform transform, srs::get_transform(target_srs(this->m_projection), source_srs), "getting the transform into the dataset's system");
    // TODO: skip samples that cannot be mapped instead of failing, so that bounds overlapping only the coverage's guard band do not fail
    ALP_TRY_CONTEXT(transform.apply_inplace(&points), "transforming the pixel size samples into the dataset's system");

    ALP_TRY(const GeoTransform geo_transform, this->m_dataset->geo_transform());
    const glm::dvec2 source_pixel_size = geo_transform.pixel_size();
    glm::dvec2 result(std::numeric_limits<double>::infinity());
    for (const auto [i, step] : enumerate(steps)) {
        // Source pixels traversed per target unit along target x and y.
        const glm::dvec2 along_x = glm::abs((points[3 * i + 1] - points[3 * i]) / (step.x * source_pixel_size));
        const glm::dvec2 along_y = glm::abs((points[3 * i + 2] - points[3 * i]) / (step.y * source_pixel_size));
        result = glm::min(result, 1.0 / glm::dvec2(glm::compAdd(along_x), glm::compAdd(along_y)));
    }
    return result;
}

template class DatasetReader<float>;
template class DatasetReader<glm::u8vec3>;

} // namespace gis
