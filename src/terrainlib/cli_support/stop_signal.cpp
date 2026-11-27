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

#include "cli_support/stop_signal.h"

#include <atomic>
#include <csignal>
#include <memory>
#include <mutex>
#include <stop_token>
#include <thread>

#include "containers/UniqueResource.h"

namespace cli_support::stop_signal {

namespace {
    static_assert(std::atomic<int>::is_always_lock_free);
    constinit std::atomic<int> received = 0;

    // A store and a notify on a lock-free atomic are the only calls a signal handler may make.
    void record(int signal) {
        received.store(signal, std::memory_order_relaxed);
        received.notify_one();
    }

    // Never destroyed, so a stop requested while the program exits still finds it.
    std::stop_source& source() {
        static std::stop_source* const stop_source = new std::stop_source;
        return *stop_source;
    }

    // The handlers of SIGINT and SIGTERM.
    struct Handlers {
        void (*interrupt)(int);
        void (*terminate)(int);
    };

    // Puts back the handlers that an installation replaced, unless a newer installation took them over.
    struct Restore {
        void operator()(Handlers& previous) const noexcept;
    };
} // namespace

// The stop handlers that the guards keep in place, and the handlers they replaced.
struct Installation {
    UniqueResource<Handlers, Restore> previous;
};

namespace {
    constinit std::mutex installation_mutex;
    // The installation while a guard holds it.
    constinit std::weak_ptr<const Installation> guarded;
    // The installation whose stop handlers are in place, which may have lost its last guard but not yet been destroyed.
    constinit const Installation* installed = nullptr;

    void Restore::operator()(Handlers& previous) const noexcept {
        const std::lock_guard lock(installation_mutex);
        if (installed != nullptr && &installed->previous.get() == &previous) {
            std::signal(SIGINT, previous.interrupt);
            std::signal(SIGTERM, previous.terminate);
            installed = nullptr;
        }
    }
} // namespace

Guard install() {
    static std::once_flag started;
    std::call_once(started, [] {
        // Requesting the stop may lock and run callbacks, which a signal handler may not, so a thread does it once the handler has recorded a signal.
        std::thread([] {
            received.wait(0);
            source().request_stop();
        }).detach();
    });

    const std::lock_guard lock(installation_mutex);
    Guard existing = guarded.lock();
    if (existing != nullptr) {
        return existing;
    }

    Handlers previous;
    if (installed != nullptr) {
        // The stop handlers of an expiring installation are still in place, so the new one takes over what it replaced.
        previous = installed->previous.get();
    } else {
        previous.interrupt = std::signal(SIGINT, record);
        previous.terminate = std::signal(SIGTERM, record);
    }

    Guard guard = std::make_shared<const Installation>(UniqueResource<Handlers, Restore>(previous));
    installed = guard.get();
    guarded = guard;
    return guard;
}

std::stop_token token() {
    return source().get_token();
}

bool requested() {
    return received.load(std::memory_order_relaxed) != 0;
}

int exit_code() {
    const auto signal = received.load(std::memory_order_relaxed);
    return signal ? 128 + signal : 1;
}

} // namespace cli_support::stop_signal
