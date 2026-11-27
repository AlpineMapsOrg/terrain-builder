#include "../catch2_helpers.h"
#include "../result_helpers.h"
#include "../temp_directory.h"

#include <filesystem>
#include <string>
#include <vector>

#include "json/glm.h"
#include "json/serialize.h"
#include "view/CameraOptions.h"

namespace detail {

struct Sample {
    int count;
    std::string name;
};

} // namespace detail

TEST_CASE("read_from_text rejects text whose keys do not match the value", "[json][serialize]") {
    REQUIRE_VALUE(json::read_from_text<detail::Sample>(R"({"count": 3, "name": "a"})"));

    SECTION("an unknown key") {
        REQUIRE_ERROR(json::read_from_text<detail::Sample>(R"({"count": 3, "name": "a", "size": 1})"));
    }
    SECTION("a missing key") {
        REQUIRE_ERROR(json::read_from_text<detail::Sample>(R"({"count": 3})"));
    }
}

TEST_CASE("write_to_path writes a file that read_from_path reads back equal", "[json][serialize]") {
    const TempDirectory directory("json_serialize");
    const std::filesystem::path path = directory.path() / "cameras.json";
    const std::vector<view::CameraOptions> cameras{
        {.eye = {1.0, 2.0, 3.0}, .forward = {0.0, 0.0, -1.0}, .up = {0.0, 1.0, 0.0}, .fov_y = 0.5, .aspect = 1.5, .z_near = 0.1},
        {.eye = {4.0, 5.0, 6.0}, .forward = {1.0, 0.0, 0.0}, .up = {0.0, 0.0, 1.0}, .fov_y = 1.1, .aspect = 2.0, .z_near = 3.0},
    };

    REQUIRE_VALUE(json::write_to_path(cameras, path));
    CHECK(REQUIRE_VALUE(json::read_from_path<std::vector<view::CameraOptions>>(path)) == cameras);
}
