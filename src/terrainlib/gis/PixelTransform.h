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

#include "error/Result.h"
#include "gis/Transform.h"
#include "gis/georeferencing.h"

namespace gis {

class Dataset;

// The transform from a dataset's pixels into its system, which must only be used on the thread it was created on.
class PixelTransform : public Transform {
public:
    // Creates the inverse of this transform.
    [[nodiscard]]
    PixelTransform inverse() const;

    // The kind of georeferencing this maps by.
    [[nodiscard]]
    GeoreferencingKind kind() const {
        return this->_kind;
    }

private:
    friend alp::Result<PixelTransform, InvalidGeoreferencing> make_pixel_transform(const Dataset &dataset, GeoreferencingKind kind);

    PixelTransform(Transform transform, GeoreferencingKind kind);

    GeoreferencingKind _kind;
};

// Creates the transform from `dataset`'s pixels into its system by its georeferencing of `kind`, which the dataset must have.
[[nodiscard]]
alp::Result<PixelTransform, InvalidGeoreferencing> make_pixel_transform(const Dataset &dataset, GeoreferencingKind kind);

} // namespace gis
