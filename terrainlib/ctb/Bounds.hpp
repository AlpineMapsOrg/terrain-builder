#ifndef BOUNDS_HPP
#define BOUNDS_HPP

/*******************************************************************************
 * Copyright 2014 GeoData <geodata@soton.ac.uk>
 * Copyright 2021 Adam Celarek <lastname at cg tuwien ac at>
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License.  You may obtain a copy
 * of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *******************************************************************************/

/**
 * @file Bounds.hpp
 * @brief This declares and defines the `Bounds` class
 */
#include <array>
#include <glm/glm.hpp>

#include "CTBException.hpp"

namespace ctb {

/// A representation of an extent
template <class T>
class Bounds {
public:
    glm::tvec2<T> min = {};
    glm::tvec2<T> max = {};
    /// Create an empty bounds
    Bounds()
    {
        bounds[0] = bounds[1] = bounds[2] = bounds[3] = 0;
    }

    /// Create bounds from individual extents
    Bounds(T minx, T miny, T maxx, T maxy)
    {
        setBounds(minx, miny, maxx, maxy);
    }

    /// Create bounds represented by lower left and upper right coordinates
    Bounds(const glm::tvec2<T>& lowerLeft, const glm::tvec2<T>& upperRight)
    {
        setBounds(lowerLeft, upperRight);
    }

    /// Overload the equality operator
    bool operator==(const Bounds<T>& other) const = default;

    /// Set the bounds from extents
    inline void
    setBounds(T minx, T miny, T maxx, T maxy)
    {
        min.x = minx;
        min.y = miny;
        max.x = maxx;
        max.y = maxy;
        bounds[0] = minx;
        bounds[1] = miny;
        bounds[2] = maxx;
        bounds[3] = maxy;
    }

    /// Set the bounds from lower left and upper right coordinates
    inline void
    setBounds(const glm::tvec2<T>& lowerLeft, const glm::tvec2<T>& upperRight)
    {
        setBounds(lowerLeft.x, lowerLeft.y, upperRight.x, upperRight.y);
    }

    /// Get the minimum X value
    inline T
    getMinX() const
    {
        return bounds[0];
    }

    /// Get the minimum Y value
    inline T
    getMinY() const
    {
        return bounds[1];
    }

    /// Get the maximum X value
    inline T
    getMaxX() const
    {
        return bounds[2];
    }

    /// Get the maximum Y value
    inline T
    getMaxY() const
    {
        return bounds[3];
    }

    /// Set the minimum X value
    inline void
    setMinX(T newValue)
    {
        bounds[0] = newValue;
    }

    /// Set the minimum Y value
    inline void
    setMinY(T newValue)
    {
        bounds[1] = newValue;
    }

    /// Set the maximum X value
    inline void
    setMaxX(T newValue)
    {
        bounds[2] = newValue;
    }

    /// Set the maximum Y value
    inline void
    setMaxY(T newValue)
    {
        bounds[3] = newValue;
    }

    /// Get the lower left corner
    inline glm::tvec2<T>
    getLowerLeft() const
    {
        return glm::tvec2<T>(getMinX(), getMinY());
    }

    /// Get the lower right corner
    inline glm::tvec2<T>
    getLowerRight() const
    {
        return glm::tvec2<T>(getMaxX(), getMinY());
    }

    /// Get the upper right corner
    inline glm::tvec2<T>
    getUpperRight() const
    {
        return glm::tvec2<T>(getMaxX(), getMaxY());
    }

    /// Get the upper left corner
    inline glm::tvec2<T>
    getUpperLeft() const
    {
        return glm::tvec2<T>(getMinX(), getMaxY());
    }

    /// Get the width
    inline T
    getWidth() const
    {
        return getMaxX() - getMinX();
    }

    /// Get the height
    inline T
    getHeight() const
    {
        return getMaxY() - getMinY();
    }

    T width() const { return max.x - min.x; }
    T height() { return max.y - min.y; }

private:
    /// The extents themselves as { minx, miny, maxx, maxy }
    std::array<T, 4> bounds;
};

template <typename T>
bool intersect(const Bounds<T>& a, const Bounds<T>& b)
{
    // http://stackoverflow.com/questions/306316/determine-if-two-rectangles-overlap-each-other
    return a.min.x <= b.max.x && b.min.x <= a.max.x && a.min.y <= b.max.y && b.min.y <= a.max.y;
}

template <typename T>
Bounds<T> intersection(const Bounds<T>& a, const Bounds<T>& b)
{
    Bounds<T> r;
    r.min.x = std::max(a.min.x, b.min.x);
    r.min.y = std::max(a.min.y, b.min.y);
    r.max.x = std::min(a.max.x, b.max.x);
    r.max.y = std::min(a.max.y, b.max.y);
    return r;
}

}
#endif /* BOUNDS_HPP */
