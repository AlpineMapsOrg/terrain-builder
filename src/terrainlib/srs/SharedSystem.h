#pragma once

#include <memory>
#include <mutex>
#include <optional>

#include <tbb/enumerable_thread_specific.h>

#include "srs/System.h"

namespace srs {

// A spatial reference that several threads use at the same time, each through its own copy.
class SharedSystem {
public:
    explicit SharedSystem(const System &system) : _shared(system), _mutex(std::make_unique<std::mutex>()) {}

    // The calling thread's copy, made on its first call.
    [[nodiscard]]
    const System &get() const {
        std::optional<System> &copy = this->_copies.local();
        if (!copy) {
            // GDAL does not allow using one spatial reference from several threads at once, so the shared system is only read under the lock.
            const std::lock_guard lock(*this->_mutex);
            copy = this->_shared;
        }
        return *copy;
    }

private:
    System _shared;
    std::unique_ptr<std::mutex> _mutex;
    mutable tbb::enumerable_thread_specific<std::optional<System>> _copies;
};

} // namespace srs
