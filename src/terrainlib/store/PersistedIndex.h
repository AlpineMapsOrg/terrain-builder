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

#include <filesystem>
#include <optional>
#include <utility>

#include <libassert/assert.hpp>

#include "containers/UniqueResource.h"
#include "error/Result.h"
#include "error/log.h"
#include "error/try.h"
#include "error/utils.h"
#include "store/Hash.h"
#include "store/Index.h"
#include "store/IndexFormat.h"

namespace store {

// The index of a storage, which it writes into the storage's directory when it closes with unsaved changes.
template <Index IndexType>
class PersistedIndex {
public:
    using Key = typename IndexType::Traits::Key;

    // Takes the index of the storage at `base_path`, which `format` writes.
    PersistedIndex(IndexType index, IndexFormat<IndexType> format, std::filesystem::path base_path, bool dirty = true)
        : _state(State {
              .index = std::move(index),
              .format = format,
              .base_path = std::move(base_path),
              .dirty = dirty,
          }) { }

    const IndexType& index() const {
        return this->_state->index;
    }

    // Records that the node has a payload with `hash`.
    // Returns false if it already had one.
    [[nodiscard]]
    bool add(const Key& key, std::optional<PayloadHash> hash) {
        const bool added = this->_state->index.add(key, std::move(hash));
        if (added) {
            this->_state->dirty = true;
        }
        return added;
    }

    // Writes the index into the storage's directory if it has unsaved changes.
    [[nodiscard]]
    alp::Result<void, WriteIndexError> save() {
        return write(this->_state.get());
    }

private:
    struct State {
        IndexType index;
        IndexFormat<IndexType> format;
        std::filesystem::path base_path;
        bool dirty;
    };

    // Writes `state` into the storage's directory if it has unsaved changes.
    static alp::Result<void, WriteIndexError> write(State& state) {
        if (!state.dirty) {
            return {};
        }

        const auto context = alp::make_context("writing the index of the storage at {}", state.base_path);
        ALP_TRY_CONTEXT(state.format.write(state.base_path, state.index), context);

        state.dirty = false;
        return {};
    }

    // Writes the state if it has unsaved changes, logging a failure.
    struct SaveOnClose {
        void operator()(State& state) const noexcept {
            const auto result = write(state);
            if (!result.has_value()) {
                LOG_REPORT(result.error(), "closing the storage");
            }
        }
    };

    UniqueResource<State, SaveOnClose> _state;
};

} // namespace store
