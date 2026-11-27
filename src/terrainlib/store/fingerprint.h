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

#include <algorithm>
#include <concepts>
#include <filesystem>
#include <functional>
#include <optional>
#include <string_view>
#include <vector>

#include <fmt/format.h>
#include <fmt/std.h>

#include "error/Context.h"
#include "error/Result.h"
#include "error/try.h"
#include "io/bytes.h"
#include "io/hash.h"
#include "io/serde.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store/IndexFormat.h"

namespace store {

// The identity of a storage's content.
struct Fingerprint {
    io::bytes::vector hash;

    bool operator==(const Fingerprint&) const = default;
};

// Why an index cannot be fingerprinted.
enum class FingerprintError {
    // A node with a payload has no hash.
    MissingHash,
};

inline std::string_view format_as(const FingerprintError error) {
    switch (error) {
    case FingerprintError::MissingHash:
        return "a node with a payload has no hash";
    default:
        return "an unknown fingerprint failure";
    }
}

// Fingerprints a storage's content from its index.
template <Index I, typename Compare = std::ranges::less>
    requires std::strict_weak_order<Compare, const typename I::Traits::Key&, const typename I::Traits::Key&>
[[nodiscard]]
alp::Result<Fingerprint, FingerprintError, io::serde::SerializeError> fingerprint(const I& index, Compare compare = {}) {
    using Key = typename I::Traits::Key;
    struct Entry {
        Key key;
        PayloadHashView hash;
    };

    std::vector<Entry> entries;
    for (const Key& key : index.keys()) {
        const std::optional<PayloadHashView> hash_opt = index.hash(key);
        if (!hash_opt.has_value()) {
            return alp::make_error(FingerprintError::MissingHash, fmt::format("node {} has no hash", key));
        }
        entries.push_back({key, *hash_opt});
    }

    // Sorting by key makes the result independent of the index's order.
    std::ranges::sort(entries, compare, &Entry::key);
    // The bytes are only hashed, never decoded, so they need no version; a layout change only makes recorded fingerprints stop matching.
    ALP_TRY(const io::bytes::vector bytes, io::serde::serialize(entries));

    Fingerprint result{
        .hash = io::hash::compute(bytes, io::hash::Algorithm::Xxh3_64),
    };
    return result;
}

// Fingerprints the storage at `base_path`, reading only its index.
template <Index I, typename Compare = std::ranges::less>
    requires std::strict_weak_order<Compare, const typename I::Traits::Key&, const typename I::Traits::Key&>
[[nodiscard]]
alp::Result<Fingerprint, FingerprintError, io::serde::SerializeError, ReadIndexError> fingerprint(
    const std::filesystem::path& base_path,
    const IndexFormat<I>& format,
    Compare compare = {}
) {
    const auto context = alp::make_context("fingerprinting the storage at {}", base_path);
    ALP_TRY_CONTEXT(const I index, format.read(base_path), context);
    ALP_TRY_CONTEXT(validate_index(index, true), context);
    ALP_TRY_CONTEXT(Fingerprint result, fingerprint(index, compare), context);
    return result;
}

} // namespace store
