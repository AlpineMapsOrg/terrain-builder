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

#include "io/envelope.h"

#include <utility>

#include <fmt/format.h>
#include <magic_enum/magic_enum.hpp>

namespace io::envelope {

alp::Result<Header, ReadHeaderError> read_header(const std::filesystem::path &path) {
    const auto context = alp::make_context("reading the envelope header from {}", path);
    ALP_TRY_CONTEXT(const bytes::vector prefix, bytes::read_prefix(path, max_header_size), context);

    auto header_result = serde::deserialize_prefix<Header, max_header_size>(prefix);
    if (!header_result.has_value()) {
        return detail::to_envelope_error(std::move(header_result).error(), context);
    }
    ALP_TRY_CONTEXT(detail::validate_header(header_result.value().value), context);
    return std::move(header_result).value().value;
}

namespace detail {

alp::Result<void, HeaderError> validate_header(const Header &header) {
    if (header.magic != magic) {
        return alp::make_error(CorruptData{}, fmt::format("the data starts with {:#018x} instead of the envelope magic number", header.magic));
    }
    if (header.class_name.size() > max_class_name_size) {
        return alp::make_error(CorruptData{}, fmt::format("the class name has {} bytes, more than the {} the format allows", header.class_name.size(), max_class_name_size));
    }
    if (header.hash.size() > max_hash_size) {
        return alp::make_error(CorruptData{}, fmt::format("the hash has {} bytes, more than the {} the format allows", header.hash.size(), max_hash_size));
    }
    if (!magic_enum::enum_contains(header.hash_algorithm)) {
        return alp::make_error(Unsupported{}, fmt::format("the header names the unknown hash algorithm {}", std::to_underlying(header.hash_algorithm)));
    }
    if (!magic_enum::enum_contains(header.compression_algorithm)) {
        return alp::make_error(Unsupported{}, fmt::format("the header names the unknown compression algorithm {}", std::to_underlying(header.compression_algorithm)));
    }
    return {};
}

} // namespace detail

} // namespace io::envelope
