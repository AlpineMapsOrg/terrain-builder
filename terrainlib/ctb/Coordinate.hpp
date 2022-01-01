#ifndef COORDINATE_HPP
#define COORDINATE_HPP

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

#include <glm/glm.hpp>

/**
 * @file Coordinate.hpp
 * @brief This declares and defines the `Coordinate` class
 */

namespace ctb {

template <class T>
using Coordinate = glm::tvec2<T>;

}

#endif /* COORDINATE_HPP */
