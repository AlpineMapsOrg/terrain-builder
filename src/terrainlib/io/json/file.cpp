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

#include "io/json/file.h"

#include <string>

#include "error/prelude.h"
#include "io/text.h"

namespace io::json {

alp::Result<void, WriteError> write(const Value &value, const std::filesystem::path &path, const Format format, const bytes::WriteOptions options) {
    ALP_TRY(const std::string text, to_string(value, format));
    ALP_TRY(text::write(text, path, options));
    return {};
}

} // namespace io::json
