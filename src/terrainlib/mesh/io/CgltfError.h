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

#pragma once

#include <expected>
#include <string_view>

#include <cgltf.h>
#include <libassert/assert.hpp>

namespace mesh::io {

// A failure cgltf reported.
class CgltfError {
public:
    explicit CgltfError(const cgltf_result result) : _result(result) {
        ASSERT(result != cgltf_result_success);
    }

    bool operator==(const CgltfError &) const = default;

    // The result cgltf reported.
    cgltf_result result() const {
        return this->_result;
    }

private:
    cgltf_result _result;
};

// What the result means.
inline std::string_view format_as(const CgltfError error) {
    switch (error.result()) {
    case cgltf_result_data_too_short:
        return "the data is too short";
    case cgltf_result_unknown_format:
        return "the format is unknown";
    case cgltf_result_invalid_json:
        return "the JSON is invalid";
    case cgltf_result_invalid_gltf:
        return "the glTF is invalid";
    case cgltf_result_invalid_options:
        return "the options are invalid";
    case cgltf_result_file_not_found:
        return "the file was not found";
    case cgltf_result_io_error:
        return "reading or writing the file failed";
    case cgltf_result_out_of_memory:
        return "cgltf ran out of memory";
    case cgltf_result_legacy_gltf:
        return "the file is legacy glTF";
    default:
        return "an unknown glTF failure";
    }
}

// Success, or the failure cgltf reported instead.
inline std::expected<void, CgltfError> to_cgltf_result(const cgltf_result result) {
    if (result != cgltf_result_success) {
        return std::unexpected(CgltfError(result));
    } else {
        return {};
    }
}

} // namespace mesh::io
