#include <cmath>

#include "tntn/logging.h"

#include "test_common.h"

bool double_eq(double a, double b, double eps)
{
    bool result = std::abs(a - b) < eps;

    if (!result) {
        TNTN_LOG_DEBUG("DOUBLE_EQ TEST FAILED: ({} - {}) > {}",
            a, b, eps);
    }

    return result;
}

// Takes a path relative to fixture directory and returns an absolute file path
fs::path fixture_path(const fs::path& fragment)
{
    // ATB_TEST_DATA_DIR is a macro defined via CMakeLists for test suite.
    static fs::path base_fixture_path(ATB_TEST_DATA_DIR "/tntn");
    return base_fixture_path / fs::path(fragment);
}
