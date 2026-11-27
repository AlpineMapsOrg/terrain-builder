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
#include <optional>
#include <string_view>
#include <utility>

namespace store::path_layout {

template <typename Key>
struct Layout {
    std::string_view id;
    std::filesystem::path (*locate_node)(const Key&);
    std::optional<Key> (*identify_node)(const std::filesystem::path&);

    bool operator==(const Layout&) const = default;
};

template <typename Key>
class Resolver {
public:
    Resolver(std::filesystem::path base_path, Layout<Key> layout)
        : _base_path(std::move(base_path))
        , _layout(layout) {
    }

    std::filesystem::path locate_node(const Key& key) const { return this->_base_path / this->_layout.locate_node(key); }

    std::optional<Key> identify_node(const std::filesystem::path& node_path) const {
        const std::filesystem::path relative = node_path.lexically_relative(this->_base_path);
        return this->_layout.identify_node(relative);
    }

    const std::filesystem::path& base_path() const { return this->_base_path; }
    Layout<Key> layout() const { return this->_layout; }

private:
    std::filesystem::path _base_path;
    Layout<Key> _layout;
};

} // namespace store::path_layout
