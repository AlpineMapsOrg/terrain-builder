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

namespace io {

// How a write treats a file that already exists at its path.
enum class WriteMode {
    // Fails if anything already exists at the path.
    CreateNew,
    // Writes into the existing file, following a symlink at the path, and drops its old bytes.
    Overwrite,
    // Puts a new file in place of the old one, so the path holds the old or the new bytes even after a crash.
    // Replaces of the same path must not run concurrently.
    Replace,
};

// How a write treats an existing file unless told otherwise.
inline constexpr WriteMode default_write_mode = WriteMode::CreateNew;

// Whether a write creates missing parent directories unless told otherwise.
inline constexpr bool default_make_dirs = true;

} // namespace io
