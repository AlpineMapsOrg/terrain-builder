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

#pragma once

#include <filesystem>
#include <string>
#include <string_view>
#include <system_error>
#include <utility>

#include <fmt/std.h>

#include "common_error_codes.h"
#include "error/Context.h"
#include "error/Error.h"
#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/FileError.h"
#include "io/WriteMode.h"
#include "io/bytes.h"
#include "io/compression.h"
#include "io/envelope.h"
#include "io/filesystem_utils.h"
#include "io/limits.h"
#include "io/serde.h"
#include "store/storage_paths.h"

// A draft records the inputs that produced it, so that a later run can check whether it may reuse its nodes.
namespace store::inputs {

inline constexpr std::string_view file_name = "inputs.tmp";

// Why a run cannot reuse the nodes of a draft.
enum class ReuseError {
    // The path does not name a draft.
    NotDraft,
    // The storage records no inputs.
    MissingInputs,
    // The recorded inputs cannot be decoded.
    CorruptInputs,
    // The recorded inputs have a version this build cannot read.
    UnsupportedInputs,
    // The storage was produced from different inputs.
    DifferentInputs,
};

inline std::string_view format_as(const ReuseError error) {
    switch (error) {
    case ReuseError::NotDraft:
        return "the storage is not a draft";
    case ReuseError::MissingInputs:
        return "the storage records no inputs";
    case ReuseError::CorruptInputs:
        return "the storage's recorded inputs are corrupt";
    case ReuseError::UnsupportedInputs:
        return "the storage's recorded inputs have a version this build cannot read";
    case ReuseError::DifferentInputs:
        return "the storage was produced from different inputs";
    default:
        return "an unknown reuse failure";
    }
}

using ValidateReuseError = alp::Error<ReuseError, io::FileError, io::AllocError>;

struct Unencodable : alp::MessageCode<"the inputs cannot be encoded"> {};

using WriteError = alp::Error<io::FileError, io::AllocError, Unencodable>;

namespace detail {

// The failure writing the inputs for a failure writing their envelope, which it keeps as its source.
[[nodiscard]]
inline WriteError to_write_error(io::envelope::WriteError error) {
    return std::move(error).map<io::serde::Unserializable, io::compression::Unknown>(Unencodable{});
}

// The failure checking reuse for a failure reading the recorded inputs, which it keeps as its source.
[[nodiscard]]
inline ValidateReuseError to_validate_reuse_error(io::envelope::ReadError error) {
    return std::move(error)
        .map_code(std::errc::no_such_file_or_directory, ReuseError::MissingInputs)
        .map<io::envelope::CorruptData, io::compression::CorruptData, io::compression::Truncated, io::compression::Unknown>(ReuseError::CorruptInputs)
        .map<Unsupported>(ReuseError::UnsupportedInputs);
}

} // namespace detail

// Records `inputs` in the draft at `base_path`.
template <typename Schema>
[[nodiscard]]
alp::Result<void, WriteError> write(const std::filesystem::path& base_path, const typename Schema::Latest& inputs) {
    const io::envelope::WriteOptions options{
        .mode = io::WriteMode::Overwrite,
    };
    auto write_result = io::envelope::write<Schema>(inputs, base_path / file_name, options);
    ALP_TRY(std::move(write_result).transform_error(detail::to_write_error));
    return {};
}

// Checks that the nodes of the storage at `source_path` can be hard-linked into the storage at `output_path`.
[[nodiscard]]
inline alp::Result<void, io::FileError> validate_linkable(const std::filesystem::path& source_path, const std::filesystem::path& output_path) {
    return io::filesystem::require_same_filesystem(source_path, output_path);
}

// Checks that the nodes of the draft at `draft_path` can be reused for the storage at `output_path`.
// It must have been produced from `inputs`, and its nodes must be linkable into the output.
template <typename Schema>
[[nodiscard]]
alp::Result<void, ValidateReuseError> validate_reuse(
    const std::filesystem::path& draft_path,
    const std::filesystem::path& output_path,
    const typename Schema::Latest& inputs
) {
    const auto context = alp::make_context("checking whether the nodes of {} can be reused", draft_path);

    if (!is_draft(draft_path)) {
        return alp::make_error(ReuseError::NotDraft).context(context);
    }

    auto read_result = io::envelope::read<Schema>(draft_path / file_name);
    auto saved_result = std::move(read_result).transform_error(detail::to_validate_reuse_error);
    ALP_TRY_CONTEXT(const typename Schema::Latest saved, std::move(saved_result), context);
    if (saved != inputs) {
        return alp::make_error(ReuseError::DifferentInputs).context(context);
    }

    ALP_TRY_CONTEXT(io::filesystem::require_same_filesystem(draft_path, output_path), context);
    return {};
}

} // namespace store::inputs
