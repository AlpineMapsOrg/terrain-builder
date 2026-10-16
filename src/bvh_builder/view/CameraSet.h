#pragma once

#include <string>
#include <vector>

#include "view/CameraOptions.h"

namespace view {

// A named list of cameras.
struct CameraSet {
    std::string name;
    std::vector<CameraOptions> cameras;

    bool operator==(const CameraSet &) const = default;
};

}
