/*****************************************************************************
 * Alpine Terrain Builder
 * Copyright (C) 2022 Adam Celarek <last name at cg dot tuwien dot ac dot at>
 * Copyright (C) 2022 alpinemaps.org
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

#ifndef PROGRESSINDICATOR_H
#define PROGRESSINDICATOR_H

#include <atomic>
#include <cassert>
#include <string>
#include <thread>

#include "Exception.h"

class ProgressIndicator {
    const size_t m_n_steps;
    std::atomic<size_t> m_step = 0;

public:
    ProgressIndicator(size_t n_steps);

    void taskFinished();
    [[nodiscard]] std::jthread startMonitoring() const; // join on the returned thread after the work is done!!
    [[nodiscard]] std::string progressBar() const;
    [[nodiscard]] std::string xOfYDoneMessagE() const;
};

#endif // PROGRESSINDICATOR_H
