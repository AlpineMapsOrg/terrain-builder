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

#include <catch2/reporters/catch_reporter_event_listener.hpp>
#include <catch2/reporters/catch_reporter_registrars.hpp>
#include <libassert/assert.hpp>

#include "assertion_helpers.h"

namespace {

[[noreturn]] void throw_assertion_failure(const libassert::assertion_info &info) {
    throw AssertionFailure(info.to_string(0, libassert::color_scheme::blank));
}

// Makes a failed assertion throw, so a test can expect it and any other test reports it instead of aborting the run.
class ThrowingAssertionsListener : public Catch::EventListenerBase {
public:
    using Catch::EventListenerBase::EventListenerBase;

    void testRunStarting(const Catch::TestRunInfo &) override {
        libassert::set_failure_handler(throw_assertion_failure);
    }
};

}

CATCH_REGISTER_LISTENER(ThrowingAssertionsListener)
