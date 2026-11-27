#pragma once

namespace srs {

// The kind of a system's coordinates; a compound system has the kind of its horizontal part.
enum class Kind {
    Geographic,
    Projected,
    Geocentric,
};

}
