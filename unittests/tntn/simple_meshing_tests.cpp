#include <catch2/catch.hpp>

#include "tntn/MeshIO.h"
#include "tntn/Raster.h"
#include "tntn/simple_meshing.h"

namespace tntn {
namespace unittests {

    TEST_CASE("generate_tin_dense_quadwalk meshing on artificial terrain", "[tntn]")
    {
        auto terrain_fn = [](int x, int y) -> double { return sin(x) * sin(y); };

        const int w = 11;
        const int h = 20;
        const double cellsize = 1.5;
        const double raster_xpos = 42;
        const double raster_ypos = 23;

        RasterDouble raster;
        raster.allocate(w, h);
        raster.set_pos_x(raster_xpos);
        raster.set_pos_y(raster_ypos);
        raster.set_cell_size({ cellsize, cellsize });

        for (int y = 0; y < h; y++) {
            auto row_ptr = raster.get_ptr(y);
            for (int x = 0; x < w; x++) {
                row_ptr[x] = terrain_fn(x, y);
            }
        }

        auto general_checks = [&](auto& mesh) {
            REQUIRE(mesh != nullptr);
            CHECK(mesh->check_tin_properties());

            BBox3D bbox;
            mesh->get_bbox(bbox);

            CHECK(bbox.min.x == raster_xpos);
            CHECK(bbox.min.y == raster_ypos);
            CHECK(bbox.max.x == raster_xpos + (w - 1) * cellsize);
            CHECK(bbox.max.y == raster_ypos + (h - 1) * cellsize);
        };

        SECTION("with step = 1")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 1);
            general_checks(mesh);
            CHECK(mesh->poly_count() == (w - 1) * (h - 1) * 2);
        }

        SECTION("with step = 2")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 2);
            general_checks(mesh);
            CHECK(mesh->poly_count() == (6 - 1) * (11 - 1) * 2);
        }

        SECTION("with step = 3")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 3);
            general_checks(mesh);
            CHECK(mesh->poly_count() == (5 - 1) * (8 - 1) * 2);
        }

        SECTION("with step = 4")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 4);
            general_checks(mesh);
            CHECK(mesh->poly_count() == (4 - 1) * (6 - 1) * 2);
        }

        SECTION("with vertex count = 2")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 2, 2);
            general_checks(mesh);
            CHECK(mesh->poly_count() == 2);
        }

        SECTION("with vertex count = 3")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 3, 3);
            general_checks(mesh);
            CHECK(mesh->poly_count() == 4 * 2);
        }

        SECTION("with vertex count = 5")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 5, 5);
            general_checks(mesh);
            CHECK(mesh->poly_count() == 16 * 2);
        }

        SECTION("with vertex count = 8")
        {
            auto mesh = generate_tin_dense_quadwalk(raster, 8, 8);
            general_checks(mesh);
            CHECK(mesh->poly_count() == 49 * 2);
        }
    }

} // namespace unittests
} // namespace tntn
