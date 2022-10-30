#include <catch2/catch.hpp>

#include "Tile.h"
#include "tntn/Mesh.h"
#include "tntn/Mesh2Raster.h"
#include "tntn/Raster.h"
#include "tntn/RasterIO.h"
#include "tntn/SurfacePoints.h"
#include "tntn/raster_tools.h"

#include <boost/filesystem.hpp>
#include <cstdlib>
#include <memory>

namespace tntn {
namespace unittests {

    TEST_CASE("raster default value", "[tntn]")
    {
        RasterDouble raster;
        double ndv = raster.get_no_data_value();

        double check = 0;
        detail::NDVDefault<double>::set(check);

        CHECK(ndv == check);
    }

    TEST_CASE("raster bb", "[tntn]")
    {
        RasterDouble raster(798, 212);

        raster.set_pos_x(11.024);
        raster.set_pos_y(-334.242);
        raster.set_cell_width(1.123);
        raster.set_cell_height(1.341);

        BBox2D rbb = raster.get_bounding_box();

        double w = raster.get_width() * raster.get_cell_width();
        double h = raster.get_height() * raster.get_cell_height();

        CHECK(std::abs((rbb.max.x - rbb.min.x) - w) < 0.001);
        CHECK(std::abs((rbb.max.y - rbb.min.y) - h) < 0.001);
    }

    TEST_CASE("raster coordinates", "[tntn]")
    {
        RasterDouble raster(7, 21);

        raster.set_pos_x(11);
        raster.set_pos_y(-334);
        raster.set_cell_width(1.123);
        raster.set_cell_height(1.341);

        for (int r = 0; r < raster.get_height(); r++) {
            for (int c = 0; c < raster.get_width(); c++) {
                double x = raster.col2x(c);
                double y = raster.row2y(r);

                int cn = raster.x2col(x);
                int rn = raster.y2row(y);

                CHECK(cn == c);
                CHECK(rn == r);
            }
        }
    }

    TEST_CASE("mesh 2 raster", "[tntn]")
    {

        std::vector<Triangle> trList;

        Triangle t;

        t[0].x = 0.0;
        t[0].y = 0.0;
        t[0].z = 1;

        t[1].x = 0.0;
        t[1].y = 3.0;
        t[1].z = 1;

        t[2].x = 3.0;
        t[2].y = 3.0;
        t[2].z = 1;

        trList.push_back(t);

        t[0].x = 0.0;
        t[0].y = 0.0;
        t[0].z = -1;

        t[1].x = 3.0;
        t[1].y = 3.0;
        t[1].z = -1;

        t[2].x = 3.0;
        t[2].y = 0.0;
        t[2].z = -1;

        trList.push_back(t);

        Mesh m;
        m.from_triangles(std::move(trList));
        m.generate_decomposed();

        SECTION("pixel width / height == 1")
        {
            Mesh2Raster m2r;
            RasterDouble raster_4 = m2r.rasterise(m, 4, 4);

            CHECK(raster_4.get_cell_width() == 1);
            CHECK(raster_4.get_cell_height() == 1);

            RasterDouble raster_10 = m2r.rasterise(m, 10, 10, 4, 4);

            CHECK(raster_10.get_cell_width() == 0.4);
            CHECK(raster_10.get_cell_height() == 0.4);
        }

        SECTION("pixel width / height == 1.5")
        {
            Mesh2Raster m2r;
            RasterDouble raster_4 = m2r.rasterise(m, 6, 4);

            CHECK(raster_4.get_cell_width() == Approx(0.6));
            CHECK(raster_4.get_cell_height() == Approx(1.0));

            RasterDouble raster_10 = m2r.rasterise(m, 15, 10, 6, 4);

            CHECK(raster_10.get_cell_width() == Approx((3.0 / (6.0 - 1.0)) * (6.0 / 15.0)));
            CHECK(raster_10.get_cell_height() == Approx((3.0 / (4.0 - 1.0)) * (4.0 / 10.0)));

            RasterDouble raster_15 = m2r.rasterise(m, 15, 15, 6, 4);

            CHECK(raster_15.get_cell_width() == Approx((3.0 / (6.0 - 1.0)) * (6.0 / 15.0)));
            CHECK(raster_15.get_cell_height() == Approx((3.0 / (4.0 - 1.0)) * (4.0 / 15.0)));
        }
    }

    TEST_CASE("Raster integer downsampe", "[tntn]")
    {
        RasterDouble big(8, 10);
        big.set_all(1);

        RasterDouble small_01 = raster_tools::integer_downsample_mean(big, 2);

        CHECK(small_01.get_height() == big.get_height() / 2);
        CHECK(small_01.get_width() == big.get_width() / 2);

        int ws = small_01.get_width();
        int hs = small_01.get_height();

        for (int rs = 0; rs < hs; rs++) {
            for (int cs = 0; cs < ws; cs++) {
                CHECK(small_01.value(rs, cs) == 1.0);
            }
        }
    }

    TEST_CASE("Raster crop", "[tntn]")
    {
        struct P {
            int r = -10;
            int c = -10;
        };

        Raster<P> raster;
        raster.allocate(5, 6);

        raster.set_cell_width(1);
        raster.set_cell_height(1);
        raster.set_pos_x(0);
        raster.set_pos_y(0);

        P p;
        p.r = -999;
        p.c = -888;

        raster.set_no_data_value(p);

        int w = raster.get_width();
        int h = raster.get_height();

        for (int r = 0; r < h; r++) {
            for (int c = 0; c < w; c++) {
                raster.value(r, c).r = r;
                raster.value(r, c).c = c;
            }
        }

        // control - just a copy!
        auto c1 = raster.crop(0, 0, w, h);

        CHECK(c1.get_width() == w);
        CHECK(c1.get_height() == h);
        CHECK(c1.get_no_data_value().r == -999);
        CHECK(c1.get_no_data_value().c == -888);
        CHECK(c1.get_pos_x() == 0);
        CHECK(c1.get_pos_y() == 0);

        for (int r = 0; r < h; r++) {
            for (int c = 0; c < w; c++) {
                CHECK(c1.value(r, c).r == r);
                CHECK(c1.value(r, c).c == c);
            }
        }

        // crop from top left with width=3 height=4
        // using raster coorinates with origin at top left!
        // (i.e. as data is stored)
        auto c2 = raster.crop(0, 0, 3, 4);

        CHECK(c2.get_width() == 3);
        CHECK(c2.get_height() == 4);
        CHECK(c2.get_no_data_value().r == -999);
        CHECK(c2.get_no_data_value().c == -888);
        CHECK(c2.get_pos_x() == 0);
        CHECK(c2.get_pos_y() == 2);

        for (int r = 0; r < c2.get_height(); r++) {
            for (int c = 0; c < c2.get_width(); c++) {
                CHECK(c2.value(r, c).r == r);
                CHECK(c2.value(r, c).c == c);
            }
        }

        // crop from bottom left with width=3 height=4
        // using raster coorinates with origin at top left!
        // this should be identical to c2!
        auto c3 = raster.crop_ll(0, 2, 3, 4);

        CHECK(c2.get_height() == c3.get_height());
        CHECK(c2.get_width() == c3.get_width());

        CHECK(c3.get_pos_x() == 0);
        CHECK(c3.get_pos_y() == 2);

        for (int r = 0; r < c2.get_height(); r++) {
            for (int c = 0; c < c2.get_width(); c++) {
                CHECK(c2.value(r, c).r == c3.value(r, c).r);
                CHECK(c2.value(r, c).c == c3.value(r, c).c);
            }
        }

        // test what happens if crop region is larger than original raster
        // this should return overlapping region:

        Raster<int> raster_int(10, 23);

        raster_int.set_all(-1);
        raster_int.set_pos_x(-14);
        raster_int.set_pos_y(18);
        raster_int.set_cell_width(0.3);
        raster_int.set_cell_height(0.3);
        raster_int.set_no_data_value(-9999);

        for (int r = 0; r < raster_int.get_height(); r++) {
            for (int c = 0; c < raster_int.get_width(); c++) {
                raster_int.value(r, c) = r * c;
            }
        }

        // crop is large than original in all dimensions:

        Raster<int> cropped_raster;

        raster_int.crop(-12, -5, 100, 80, cropped_raster);

        CHECK(raster_int.get_width() == cropped_raster.get_width());
        CHECK(raster_int.get_height() == cropped_raster.get_height());
        CHECK(raster_int.get_cell_size() == cropped_raster.get_cell_size());
        CHECK(raster_int.get_pos_x() == cropped_raster.get_pos_x());
        CHECK(raster_int.get_pos_y() == cropped_raster.get_pos_y());

        for (int r = 0; r < raster_int.get_height(); r++) {
            for (int c = 0; c < raster_int.get_width(); c++) {
                CHECK(raster_int.value(r, c) == cropped_raster.value(r, c));
            }
        }

        // crop is overlapping original:
        cropped_raster.clear();

        raster_int.crop(-12, -5, 20, 10, cropped_raster);

        for (int r = 0; r < cropped_raster.get_height(); r++) {
            for (int c = 0; c < cropped_raster.get_width(); c++) {
                CHECK(cropped_raster.get_no_data_value() != cropped_raster.value(r, c));
                CHECK(raster_int.value(r, c) == cropped_raster.value(r, c));
            }
        }

        cropped_raster.clear();

        int sx = 5;
        int sy = 7;
        raster_int.crop(sx, sy, 300, 500, cropped_raster);

        for (int r = 0; r < cropped_raster.get_height(); r++) {
            for (int c = 0; c < cropped_raster.get_width(); c++) {
                CHECK(cropped_raster.get_no_data_value() != cropped_raster.value(r, c));
                CHECK(raster_int.value(r + sy, c + sx) == cropped_raster.value(r, c));
            }
        }
    }

    TEST_CASE("Raster setAll", "[tntn]")
    {

        Raster<double> r;
        r.set_all(42.0);

        r.allocate(100, 100);
        r.set_all(23.0);

        CHECK(r.value(50, 50) == 23.0);

        class TestClass {
        public:
            TestClass() {};
            TestClass(int vv)
                : v(vv)
            {
            }
            int v = -1;
            double a = 0;
            double b = 0;
        };

        Raster<TestClass> r2;
        TestClass tc(123);
        r2.set_all(tc);

        r2.allocate(10, 1);
        r2.set_all(tc);

        for (int i = 0; i < r2.get_width() * r2.get_height(); i++) {
            CHECK(r2.get_ptr()[i].v == 123);
        }

        tc.v = -321;
        r2.allocate(1, 10);
        r2.set_all(tc);

        for (int i = 0; i < r2.get_width() * r2.get_height(); i++) {
            CHECK(r2.get_ptr()[i].v == -321);
        }
    }

    class FailOnCopy {
        int v = 0;

    public:
        FailOnCopy() = default;
        FailOnCopy(const FailOnCopy& other)
            : v(other.v)
        {
            CHECK(false);
        }
        FailOnCopy(FailOnCopy&& other) noexcept
            : v(other.v)
        {
            CHECK(true);
        }
        FailOnCopy& operator=(const FailOnCopy& other)
        {
            v = other.v;
            CHECK(false);
            return *this;
        }
        FailOnCopy& operator=(FailOnCopy&& other) noexcept
        {
            v = other.v;
            CHECK(true);
            return *this;
        }
        ~FailOnCopy() = default;
    };

    TEST_CASE("move-construct Raster", "[tntn]")
    {

        Raster<FailOnCopy> r;
        r.allocate(10, 10);

        Raster<FailOnCopy> r2(std::move(r));

        CHECK(r2.get_width() == 10);
        CHECK(r2.get_height() == 10);
    }

    TEST_CASE("move-construct Raster from Image", "[tntn]")
    {
        Image<FailOnCopy> r(10, 10);

        Raster<FailOnCopy> r2(std::move(r), {});

        CHECK(r2.get_width() == 10);
        CHECK(r2.get_height() == 10);
    }

    TEST_CASE("move-assign Raster", "[tntn]")
    {

        Raster<double> r;
        r.allocate(10, 10);

        Raster<double> r2;
        r2 = std::move(r);

        CHECK(r2.get_width() == 10);
        CHECK(r2.get_height() == 10);
    }

    TEST_CASE("Raster::toVertices", "[tntn]")
    {

        Raster<double> r;
        r.allocate(10, 10);
        r.set_pos_x(10);
        r.set_pos_y(10);
        r.set_cell_size({ 1.0, 1.0 });

        for (int y = 0; y < r.get_height(); y++) {
            for (int x = 0; x < r.get_width(); x++) {
                r.value(y, x) = y * r.get_width() + x;
            }
        }

        SurfacePoints pts;
        pts.reserve(r.get_height() * r.get_width());

        r.to_vertices([&pts](const double x, const double y, const double z) {
            pts.push_back({ x, y, z });
        });

        int num_found = 0;
        auto ptsrange = pts.points();
        ptsrange.for_each([&num_found](const Vertex& v) {
            if (v.x == 10.0 && v.y == 10.0) {
                // lower-left cell
                num_found++;
                CHECK(v.z == 9 * 10 + 0);
            } else if (v.x == 19.0 && v.y == 19.0) {
                // upper-right cell
                num_found++;
                CHECK(v.z == 0 * 10 + 9);
            }
        });

        CHECK(num_found == 2);
    }

} // namespace unittests
} // namespace tntn
