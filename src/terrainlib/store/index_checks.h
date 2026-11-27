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

#include "error/MessageCode.h"
#include "error/Result.h"
#include "error/utils.h"
#include "store/Index.h"

namespace store {

struct NotFound : alp::MessageCode<"the storage does not have the node"> {};
struct AlreadyExists : alp::MessageCode<"the storage already has the node"> {};

namespace detail {

// Fails with NotFound if the index does not list the node.
template <Index IndexType>
[[nodiscard]]
alp::Result<void, NotFound> require_node(const IndexType& index, const typename IndexType::Traits::Key& key) {
    if (!index.has(key)) {
        return alp::make_error(NotFound{}, "the index does not list the node");
    }
    return {};
}

// Fails with AlreadyExists if the index lists the node.
template <Index IndexType>
[[nodiscard]]
alp::Result<void, AlreadyExists> require_no_node(const IndexType& index, const typename IndexType::Traits::Key& key) {
    if (index.has(key)) {
        return alp::make_error(AlreadyExists{}, "the index already lists the node");
    }
    return {};
}

} // namespace detail

} // namespace store
