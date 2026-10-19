#pragma once

#include <tbb/enumerable_thread_specific.h>

#include "srs/System.h"

namespace srs {

// A spatial reference that several threads use at the same time, each through its own copy.
class SharedSystem {
public:
    explicit SharedSystem(const System &system) {
        this->_shared.AssignAndSetThreadSafe(system);
    }

    // The calling thread's copy, made on its first call.
    [[nodiscard]]
    const System &get() const {
        bool exists = false;
        System &copy = this->_copies.local(exists);
        if (!exists) {
            // GDAL only promises thread safety for calls on the shared system itself, so it is only ever cloned; copying it or passing it into another GDAL call reads it without its lock.
            copy = clone(this->_shared);
        }
        return copy;
    }

private:
    System _shared;
    mutable tbb::enumerable_thread_specific<System> _copies;
};

} // namespace srs
