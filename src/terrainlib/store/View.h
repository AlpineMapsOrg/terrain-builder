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

#include <concepts>
#include <filesystem>
#include <string_view>
#include <vector>

#include "common_error_codes.h"
#include "error/Result.h"
#include "store/Hash.h"
#include "store/index_checks.h"
#include "store/path_layout.h"

namespace store {

// Read access to the nodes of a storage, published or being written.
template <typename S>
concept View = requires(const S& view, const typename S::Key& key) {
    typename S::Index;
    typename S::Payload;
    { view.has(key) } -> std::same_as<bool>;
    { view.load(key) } -> std::same_as<alp::Result<typename S::Payload, typename S::LoadError>>;
    { view.hash(key) } -> std::same_as<alp::Result<PayloadHash, NotFound, Unsupported>>;
    { view.base_path() } -> std::same_as<const std::filesystem::path&>;
    { view.paths(key) } -> std::same_as<std::vector<std::filesystem::path>>;
    { view.resolver() } -> std::same_as<const path_layout::Resolver<typename S::Key>&>;
    { view.codec_id() } -> std::same_as<std::string_view>;
};

} // namespace store
