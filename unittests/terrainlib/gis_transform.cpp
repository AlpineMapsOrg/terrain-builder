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

#include <catch2/catch_test_macros.hpp>

#include <algorithm>
#include <cstddef>
#include <functional>
#include <memory>
#include <optional>
#include <span>
#include <string_view>
#include <vector>

#include <cpl_error.h>
#include <cpl_port.h>
#include <glm/glm.hpp>

#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "error/format.h"
#include "gis/CplError.h"
#include "gis/Transform.h"

namespace {

// Shifts points by (1, 10, 100), or back if `inverse` is set, and leaves points with a negative x unmapped.
struct ShiftTransformer {
    bool operator()(const bool inverse, const size_t count, double *xs, double *ys, double *zs, int *successes) const {
        const double sign = inverse ? -1.0 : 1.0;
        for (size_t i = 0; i < count; ++i) {
            successes[i] = xs[i] < 0 ? FALSE : TRUE;
            if (successes[i] == FALSE) {
                continue;
            }
            xs[i] += sign * 1;
            ys[i] += sign * 10;
            if (zs != nullptr) {
                zs[i] += sign * 100;
            }
        }
        return true;
    }

    std::string_view source_name() const {
        return "before";
    }

    std::string_view target_name() const {
        return "after";
    }
};

// Maps like ShiftTransformer, but reports a failure as soon as one point is unmapped, as OGR's transformations do.
struct StrictShiftTransformer {
    bool operator()(const bool inverse, const size_t count, double *xs, double *ys, double *zs, int *successes) const {
        ShiftTransformer()(inverse, count, xs, ys, zs, successes);
        return std::ranges::all_of(std::span(successes, count), std::identity());
    }

    std::string_view source_name() const {
        return "before";
    }

    std::string_view target_name() const {
        return "after";
    }
};

// Rejects every call, after recording a GDAL error if `records_error` is set.
struct RejectingTransformer {
    bool records_error;

    bool operator()(bool, size_t, double *, double *, double *, int *) const {
        if (this->records_error) {
            CPLError(CE_Failure, CPLE_AppDefined, "rejected");
        }
        return false;
    }

    std::string_view source_name() const {
        return "before";
    }

    std::string_view target_name() const {
        return "after";
    }
};

// Maps every point onto itself and records in `received_heights_opt` whether it received heights.
struct HeightRecordingTransformer {
    std::optional<bool> *received_heights_opt;

    bool operator()(bool, const size_t count, double *, double *, double *zs, int *successes) const {
        *this->received_heights_opt = zs != nullptr;
        for (size_t i = 0; i < count; ++i) {
            successes[i] = TRUE;
        }
        return true;
    }

    std::string_view source_name() const {
        return "before";
    }

    std::string_view target_name() const {
        return "after";
    }
};

// Shifts points by the offset `argument` points to, or back if `dst_to_src` is set.
int shift_by_offset(void *argument, const int dst_to_src, const int count, double *xs, double *ys, double *zs, int *successes) {
    const glm::dvec3 &offset = *static_cast<const glm::dvec3 *>(argument);
    const double sign = dst_to_src ? -1.0 : 1.0;
    for (int i = 0; i < count; ++i) {
        xs[i] += sign * offset.x;
        ys[i] += sign * offset.y;
        if (zs != nullptr) {
            zs[i] += sign * offset.z;
        }
        successes[i] = TRUE;
    }
    return TRUE;
}

} // namespace

TEST_CASE("apply maps a point", "[gis][Transform]") {
    const gis::BasicTransform transform{ShiftTransformer()};

    SECTION("2d") {
        const glm::dvec2 point = REQUIRE_VALUE(transform.apply(glm::dvec2(2, 3)));
        CHECK(point == glm::dvec2(3, 13));
    }
    SECTION("3d") {
        const glm::dvec3 point = REQUIRE_VALUE(transform.apply(glm::dvec3(2, 3, 4)));
        CHECK(point == glm::dvec3(3, 13, 104));
    }
}

TEST_CASE("apply_inplace and apply_into map every point of a range", "[gis][Transform]") {
    const gis::BasicTransform transform{ShiftTransformer()};

    SECTION("apply_inplace in 2d") {
        std::vector<glm::dvec2> points = {{2, 3}, {4, 5}};
        REQUIRE_VALUE(transform.apply_inplace(&points));
        CHECK(points == std::vector<glm::dvec2>{{3, 13}, {5, 15}});
    }
    SECTION("apply_inplace in 3d") {
        std::vector<glm::dvec3> points = {{2, 3, 4}, {5, 6, 7}};
        REQUIRE_VALUE(transform.apply_inplace(&points));
        CHECK(points == std::vector<glm::dvec3>{{3, 13, 104}, {6, 16, 107}});
    }
    SECTION("apply_into from 3d to 3d") {
        const std::vector<glm::dvec3> points = {{2, 3, 4}, {5, 6, 7}};
        std::vector<glm::dvec3> out(2);
        REQUIRE_VALUE(transform.apply_into(points, &out));
        CHECK(out == std::vector<glm::dvec3>{{3, 13, 104}, {6, 16, 107}});
    }
    SECTION("apply_into from 3d to 2d") {
        const std::vector<glm::dvec3> points = {{2, 3, 4}, {5, 6, 7}};
        std::vector<glm::dvec2> out(2);
        REQUIRE_VALUE(transform.apply_into(points, &out));
        CHECK(out == std::vector<glm::dvec2>{{3, 13}, {6, 16}});
    }
}

TEST_CASE("apply passes heights only for 3d points", "[gis][Transform]") {
    std::optional<bool> received_heights_opt;
    const gis::BasicTransform transform{HeightRecordingTransformer{&received_heights_opt}};

    SECTION("2d") {
        REQUIRE_VALUE(transform.apply(glm::dvec2(2, 3)));
        CHECK(received_heights_opt == false);
    }
    SECTION("3d") {
        REQUIRE_VALUE(transform.apply(glm::dvec3(2, 3, 4)));
        CHECK(received_heights_opt == true);
    }
}

TEST_CASE("inverse maps points back", "[gis][Transform]") {
    const gis::BasicTransform transform{ShiftTransformer()};

    SECTION("inverse") {
        const glm::dvec3 point = REQUIRE_VALUE(transform.inverse().apply(glm::dvec3(3, 13, 104)));
        CHECK(point == glm::dvec3(2, 3, 4));
    }
    SECTION("inverse of the inverse") {
        const glm::dvec3 point = REQUIRE_VALUE(transform.inverse().inverse().apply(glm::dvec3(2, 3, 4)));
        CHECK(point == glm::dvec3(3, 13, 104));
    }
}

TEST_CASE("apply_inplace_partial reports which points were mapped", "[gis][Transform]") {
    const gis::BasicTransform transform{ShiftTransformer()};
    std::vector<glm::dvec2> points = {{2, 3}, {-1, 5}, {4, 6}};

    const gis::SuccessMask mask = REQUIRE_VALUE(transform.apply_inplace_partial(&points));

    CHECK_FALSE(mask);
    CHECK(mask.is_mapped(0));
    CHECK_FALSE(mask.is_mapped(1));
    CHECK(mask.is_mapped(2));
    CHECK(points[0] == glm::dvec2(3, 13));
    CHECK(points[2] == glm::dvec2(5, 16));
}

TEST_CASE("apply_inplace_partial reports which points were mapped if the transformer reports a failure for some", "[gis][Transform]") {
    const gis::BasicTransform transform{StrictShiftTransformer()};
    std::vector<glm::dvec2> points = {{2, 3}, {-1, 5}, {4, 6}};

    const gis::SuccessMask mask = REQUIRE_VALUE(transform.apply_inplace_partial(&points));

    CHECK(mask.is_mapped(0));
    CHECK_FALSE(mask.is_mapped(1));
    CHECK(mask.is_mapped(2));
}

TEST_CASE("apply_inplace_partial fails if the transformer rejects the call", "[gis][Transform]") {
    CPLErrorHandlerPusher quiet(CPLQuietErrorHandler);
    const gis::BasicTransform transform{RejectingTransformer{true}};
    std::vector<glm::dvec2> points = {{2, 3}};

    const auto error = REQUIRE_ERROR(transform.apply_inplace_partial(&points));

    CHECK(error.holds<gis::UntransformablePoint>());
}

TEST_CASE("apply fails if a point has no image", "[gis][Transform]") {
    const gis::BasicTransform transform{ShiftTransformer()};

    std::optional<alp::Error<gis::UntransformablePoint>> error_opt;
    SECTION("apply") {
        error_opt = REQUIRE_ERROR(transform.apply(glm::dvec2(-1, 5)));
    }
    SECTION("apply_inplace") {
        std::vector<glm::dvec2> points = {{2, 3}, {-1, 5}, {4, 6}};
        error_opt = REQUIRE_ERROR(transform.apply_inplace(&points));
    }
    SECTION("apply_into") {
        const std::vector<glm::dvec2> points = {{2, 3}, {-1, 5}, {4, 6}};
        std::vector<glm::dvec2> out(3);
        error_opt = REQUIRE_ERROR(transform.apply_into(points, &out));
    }

    CHECK(error_opt.value().holds<gis::UntransformablePoint>());
}

TEST_CASE("apply fails if the transformer rejects the call", "[gis][Transform]") {
    CPLErrorHandlerPusher quiet(CPLQuietErrorHandler);

    SECTION("with a GDAL error") {
        const gis::BasicTransform transform{RejectingTransformer{true}};

        const auto error = REQUIRE_ERROR(transform.apply(glm::dvec2(2, 3)));

        CHECK(error.holds<gis::UntransformablePoint>());
        REQUIRE(error.source() != nullptr);
        CHECK(error.source()->holds_and<gis::CplError>([](const gis::CplError &e) { return e.code() == CPLE_AppDefined; }));
    }
    SECTION("without a GDAL error") {
        const gis::BasicTransform transform{RejectingTransformer{false}};
        CPLError(CE_Failure, CPLE_OpenFailed, "recorded before the call");

        const auto error = REQUIRE_ERROR(transform.apply(glm::dvec2(2, 3)));

        CHECK(error.holds<gis::UntransformablePoint>());
        CHECK(error.source() == nullptr);
    }
}

TEST_CASE("apply succeeds on an empty range without running the transformer", "[gis][Transform]") {
    const gis::BasicTransform transform{RejectingTransformer{false}};
    std::vector<glm::dvec2> points;

    SECTION("apply_inplace") {
        REQUIRE_VALUE(transform.apply_inplace(&points));
    }
    SECTION("apply_inplace_partial") {
        const gis::SuccessMask mask = REQUIRE_VALUE(transform.apply_inplace_partial(&points));
        CHECK(mask);
    }
}

TEST_CASE("make_transform maps points by GDAL's transformer function with its argument", "[gis][Transform]") {
    const gis::Transform transform = gis::make_transform(&shift_by_offset, std::make_shared<glm::dvec3>(1, 10, 100), "before", "after");

    SECTION("forward") {
        const glm::dvec3 point = REQUIRE_VALUE(transform.apply(glm::dvec3(2, 3, 4)));
        CHECK(point == glm::dvec3(3, 13, 104));
    }
    SECTION("inverse") {
        const glm::dvec3 point = REQUIRE_VALUE(transform.inverse().apply(glm::dvec3(3, 13, 104)));
        CHECK(point == glm::dvec3(2, 3, 4));
    }
}

TEST_CASE("conversion to gis::Transform keeps the direction", "[gis][Transform]") {
    const gis::BasicTransform transform{ShiftTransformer()};

    SECTION("forward") {
        const gis::Transform converted(transform);
        const glm::dvec3 point = REQUIRE_VALUE(converted.apply(glm::dvec3(2, 3, 4)));
        CHECK(point == glm::dvec3(3, 13, 104));
    }
    SECTION("inverse") {
        const gis::Transform converted(transform.inverse());
        const glm::dvec3 point = REQUIRE_VALUE(converted.apply(glm::dvec3(3, 13, 104)));
        CHECK(point == glm::dvec3(2, 3, 4));
    }
}
