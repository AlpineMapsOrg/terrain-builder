/*****************************************************************************
 * AlpineMaps.org
 * Copyright (C) 2026 Adam Celarek-Litofcenko
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
#include <numbers>
#include <span>

#include "conversion.h"
#include "scaling_detail.h"

namespace raster::algorithm {
namespace detail {
    template <typename S>
    S lanczos_weight(S distance, unsigned radius)
    {
        const auto sinc = [](S x) {
            const S angle = std::numbers::pi_v<S> * x;
            return x == S(0) ? S(1) : std::sin(angle) / angle;
        };
        return sinc(distance) * sinc(distance / S(radius));
    }

    template <typename S>
    void normalize_weights(std::span<S> weights)
    {
        S normalization = 0;
        for (const auto weight : weights)
            normalization += weight;
        for (std::size_t i = 0; i < weights.size(); ++i)
            weights[i] /= normalization;
    }

    template <typename S>
    std::array<S, 8> upscaling_weights(std::int64_t position, unsigned factor, unsigned radius, SampleAlignment alignment = SampleAlignment::CellCenters)
    {
        const unsigned remainder = source_phase(position, factor);
        S fraction = S(remainder) / S(factor);
        if (alignment == SampleAlignment::CellCenters) {
            const S phase = (S(remainder) + S(0.5)) / S(factor) - S(0.5);
            fraction = remainder < factor / 2 ? phase + S(1) : phase;
        }
        std::array<S, 8> weights {};
        if (radius == 1) {
            weights[0] = S(1) - fraction;
            weights[1] = fraction;
        } else if (on_source_node(position, factor, alignment)) {
            weights[radius - 1] = S(1);
        } else {
            for (unsigned i = 0; i < 2 * radius; ++i)
                weights[i] = lanczos_weight(S(i) - S(radius - 1) - fraction, radius);
            normalize_weights<S>(std::span(weights).first(2 * radius));
        }
        return weights;
    }

    template <typename InputView, typename T, typename Conversion>
    Expected<void> upscale(const InputView& source,
        unsigned halo_width,
        const ScalingGeometry& geometry,
        Resampling method,
        const Conversion& conversion,
        const View<T>& destination,
        glm::i64vec2 output_offset,
        SampleAlignment alignment)
    {
        using W = DecodedPixel<Conversion, T>;
        using S = typename PixelTraits<W>::Scalar;
        const unsigned factor = geometry.factor;
        const unsigned radius = upscaling_radius(method, alignment);
        if (radius == 0) {
            for (unsigned y = 0; y < destination.height(); ++y)
                for (unsigned x = 0; x < destination.width(); ++x) {
                    const glm::uvec2 nearest(halo_width + source_cell(output_offset.x + x, factor), halo_width + source_cell(output_offset.y + y, factor));
                    std::memcpy(std::addressof(destination.pixel({ x, y })), std::addressof(source.pixel(nearest)), sizeof(T));
                }
            return {};
        }
        // Cache only the requested horizontal coefficients, not every possible phase.
        auto horizontal = allocate_output<std::array<S, 8>>({ destination.width(), 1 });
        if (!horizontal)
            return Error::propagate(std::move(horizontal));
        for (unsigned x = 0; x < destination.width(); ++x)
            horizontal->pixel({ x, 0 }) = upscaling_weights<S>(output_offset.x + x, factor, radius, alignment);
        // Retained nodes are copied, and an axis on a source node reads only that tap.
        const auto taps
            = [&](std::int64_t position) { return on_source_node(position, factor, alignment) ? glm::uvec2(radius - 1, radius) : glm::uvec2(0, 2 * radius); };
        for (unsigned y = 0; y < destination.height(); ++y) {
            const std::int64_t position_y = output_offset.y + y;
            const auto weights_y = upscaling_weights<S>(position_y, factor, radius, alignment);
            const std::int64_t origin_y = halo_width + upscaling_origin(position_y, factor, radius, alignment);
            const auto rows = taps(position_y);
            for (unsigned x = 0; x < destination.width(); ++x) {
                const std::int64_t position_x = output_offset.x + x;
                const std::int64_t origin_x = halo_width + upscaling_origin(position_x, factor, radius, alignment);
                const auto& weights_x = horizontal->pixel({ x, 0 });
                const auto columns = taps(position_x);
                if (rows.y - rows.x == 1 && columns.y - columns.x == 1) {
                    const glm::uvec2 retained(origin_x + columns.x, origin_y + rows.x);
                    std::memcpy(std::addressof(destination.pixel({ x, y })), std::addressof(source.pixel(retained)), sizeof(T));
                    continue;
                }
                W value {};
                for (unsigned dy = rows.x; dy < rows.y; ++dy)
                    for (unsigned dx = columns.x; dx < columns.y; ++dx)
                        value += std::invoke(std::get<0>(conversion), source.pixel(glm::uvec2(origin_x + dx, origin_y + dy))) * (weights_x[dx] * weights_y[dy]);
                destination.pixel({ x, y }) = std::invoke(std::get<1>(conversion), std::as_const(value));
            }
        }
        return {};
    }

    // Copies every factor-th sample from the origin, without filtering or conversion.
    template <typename InputView, typename T>
    Expected<void> decimate(const InputView& source, glm::uvec2 origin, unsigned factor, const View<T>& destination)
    {
        for (unsigned y = 0; y < destination.height(); ++y)
            for (unsigned x = 0; x < destination.width(); ++x)
                std::memcpy(std::addressof(destination.pixel({ x, y })), std::addressof(source.pixel(origin + glm::uvec2(x, y) * factor)), sizeof(T));
        return {};
    }

    // Cell-centre taps pair the two samples of each output cell. Grid-node taps are centred on the
    // retained node; their filter is half-band, so the other even offsets vanish.
    template <typename InputView, typename T, typename Conversion>
    Expected<void> downscale(const InputView& source,
        unsigned halo_width,
        const ScalingGeometry& geometry,
        unsigned radius,
        const Conversion& conversion,
        const View<T>& destination,
        SampleAlignment alignment)
    {
        using W = DecodedPixel<Conversion, T>;
        using S = typename PixelTraits<W>::Scalar;
        std::array<S, 16> weights {};
        const unsigned count = alignment == SampleAlignment::GridNodes ? (radius == 0 ? 1 : 4 * radius - 1) : (radius == 0 ? 2 : 4 * radius);
        for (unsigned i = 0; i < count; ++i) {
            const S offset = S(i) - S(count - 1) / S(2);
            const int node_offset = int(i) - int(count - 1) / 2;
            const bool vanishing = alignment == SampleAlignment::GridNodes && node_offset != 0 && node_offset % 2 == 0;
            weights[i] = radius == 0 ? S(1) : vanishing ? S(0) : lanczos_weight(offset / S(2), radius);
        }
        normalize_weights<S>(std::span(weights).first(count));
        const auto filter = [&](const auto& input, const auto& output) -> Expected<void> {
            auto horizontal = window_transform(input, { count, 1 }, { 2, 1 }, [&](const auto& window) -> W {
                W value {};
                for (unsigned x = 0; x < count; ++x) {
                    value += std::invoke(std::get<0>(conversion), window.pixel({ x, 0 })) * weights[x];
                }
                return value;
            });
            if (!horizontal) {
                return Error::propagate(std::move(horizontal));
            }
            return window_transform(
                *horizontal,
                { 1, count },
                { 1, 2 },
                [&](const auto& window) -> T {
                    W value {};
                    for (unsigned y = 0; y < count; ++y) {
                        value += window.pixel({ 0, y }) * weights[y];
                    }
                    return std::invoke(std::get<1>(conversion), std::as_const(value));
                },
                output);
        };
        return reduce_levels(source, halo_width, geometry, radius == 0 ? 0 : 2 * radius - 1, filter, destination, alignment);
    }

    template <typename InputView, typename T, typename Conversion>
    Expected<void> scale_window(const InputView& source,
        unsigned halo_width,
        int levels,
        Resampling method,
        glm::i64vec2 offset,
        const Conversion& conversion,
        const View<T>& destination,
        SampleAlignment alignment)
    {
        auto geometry = window_geometry(source.size(), halo_width, levels, method, offset, destination.size(), alignment);
        if (!geometry)
            return Error::propagate(std::move(geometry));
        if (levels == 0)
            return copy_views(*raster::make_view(source, geometry->source_origin, geometry->source_size), destination);
        if (auto valid = validate_view_overlap(source, destination, false); !valid)
            return valid;
        if (levels > 0)
            return upscale(source, halo_width, *geometry, method, conversion, destination, offset, alignment);
        if (alignment == SampleAlignment::GridNodes && method == Resampling::Cheapest)
            return decimate(source, geometry->source_origin, geometry->factor, destination);
        const unsigned support = *required_halo(levels, method, alignment);
        const auto region = *raster::make_view(source, geometry->source_origin, geometry->source_size);
        const ScalingGeometry reduction { region.size() - glm::uvec2(2 * support), destination.size(), geometry->factor };
        return downscale(region, support, reduction, *filter_radius(method), conversion, destination, alignment);
    }
} // namespace detail

/// Select a window relative to the scaled interior; negative offsets include halo pixels.
template <detail::ViewSource Source, typename Conversion, detail::WritableViewDestination Destination>
requires detail::NumericPixel<detail::SourcePixel<Source>> && detail::ScalingConversion<Conversion, detail::SourcePixel<Source>>
    && std::same_as<detail::SourcePixel<Source>, detail::SourcePixel<Destination>>
[[nodiscard]] Expected<void> scale(Source&& source,
    unsigned halo_width,
    int levels,
    Resampling method,
    glm::i64vec2 output_offset,
    const Conversion& conversion,
    Destination&& destination,
    SampleAlignment alignment = SampleAlignment::CellCenters)
{
    return detail::scale_window(
        detail::read_only_view(raster::make_view(source)), halo_width, levels, method, output_offset, conversion, raster::make_view(destination), alignment);
}

template <detail::ViewSource Source, typename Conversion>
requires detail::NumericPixel<detail::SourcePixel<Source>> && detail::ScalingConversion<Conversion, detail::SourcePixel<Source>>
[[nodiscard]] Expected<radix::Raster<detail::SourcePixel<Source>>> scale(
    Source&& source, unsigned halo_width, int levels, Resampling method, glm::i64vec2 output_offset, glm::uvec2 output_size, const Conversion& conversion)
{
    auto geometry = detail::window_geometry(source.size(), halo_width, levels, method, output_offset, output_size);
    if (!geometry)
        return Error::propagate(std::move(geometry));
    return detail::produce_raster<detail::SourcePixel<Source>>(
        output_size, [&](const auto& destination) { return scale(source, halo_width, levels, method, output_offset, conversion, destination); });
}

template <detail::ViewSource Source, detail::WritableViewDestination Destination>
requires detail::NumericPixel<detail::SourcePixel<Source>> && std::same_as<detail::SourcePixel<Source>, detail::SourcePixel<Destination>>
[[nodiscard]] Expected<void> scale(Source&& source,
    unsigned halo_width,
    int levels,
    Resampling method,
    glm::i64vec2 output_offset,
    Destination&& destination,
    SampleAlignment alignment = SampleAlignment::CellCenters)
{
    return scale(std::forward<Source>(source),
        halo_width,
        levels,
        method,
        output_offset,
        linear_conversion<detail::SourcePixel<Source>>(),
        std::forward<Destination>(destination),
        alignment);
}

template <detail::ViewSource Source>
requires detail::NumericPixel<detail::SourcePixel<Source>>
[[nodiscard]] Expected<radix::Raster<detail::SourcePixel<Source>>> scale(
    Source&& source, unsigned halo_width, int levels, Resampling method, glm::i64vec2 output_offset, glm::uvec2 output_size)
{
    return scale(std::forward<Source>(source), halo_width, levels, method, output_offset, output_size, linear_conversion<detail::SourcePixel<Source>>());
}

/// Full-output overloads select the complete scaled interior, excluding its halo.
template <detail::ViewSource Source, typename Conversion, detail::WritableViewDestination Destination>
requires detail::NumericPixel<detail::SourcePixel<Source>> && detail::ScalingConversion<Conversion, detail::SourcePixel<Source>>
    && std::same_as<detail::SourcePixel<Source>, detail::SourcePixel<Destination>>
[[nodiscard]] Expected<void> scale(Source&& source, unsigned halo_width, int levels, Resampling method, const Conversion& conversion, Destination&& destination)
{
    auto geometry = detail::scale_geometry(source.size(), halo_width, levels, method);
    if (!geometry)
        return Error::propagate(std::move(geometry));
    if (destination.size() != geometry->output)
        return Error::fail(Error::Code::InvalidInput, "raster scaling output dimensions do not match");
    return scale(std::forward<Source>(source), halo_width, levels, method, glm::i64vec2(0), conversion, std::forward<Destination>(destination));
}

template <detail::ViewSource Source, typename Conversion>
requires detail::NumericPixel<detail::SourcePixel<Source>> && detail::ScalingConversion<Conversion, detail::SourcePixel<Source>>
[[nodiscard]] Expected<radix::Raster<detail::SourcePixel<Source>>> scale(
    Source&& source, unsigned halo_width, int levels, Resampling method, const Conversion& conversion)
{
    auto geometry = detail::scale_geometry(source.size(), halo_width, levels, method);
    if (!geometry)
        return Error::propagate(std::move(geometry));
    return scale(std::forward<Source>(source), halo_width, levels, method, glm::i64vec2(0), geometry->output, conversion);
}

template <detail::ViewSource Source, detail::WritableViewDestination Destination>
requires detail::NumericPixel<detail::SourcePixel<Source>> && std::same_as<detail::SourcePixel<Source>, detail::SourcePixel<Destination>>
[[nodiscard]] Expected<void> scale(Source&& source, unsigned halo_width, int levels, Resampling method, Destination&& destination)
{
    return scale(
        std::forward<Source>(source), halo_width, levels, method, linear_conversion<detail::SourcePixel<Source>>(), std::forward<Destination>(destination));
}

template <detail::ViewSource Source>
requires detail::NumericPixel<detail::SourcePixel<Source>>
[[nodiscard]] Expected<radix::Raster<detail::SourcePixel<Source>>> scale(Source&& source, unsigned halo_width, int levels, Resampling method)
{
    return scale(std::forward<Source>(source), halo_width, levels, method, linear_conversion<detail::SourcePixel<Source>>());
}
} // namespace raster::algorithm
