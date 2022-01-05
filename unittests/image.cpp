#include <bits/ranges_algo.h>
#include <catch2/catch.hpp>
#include <algorithm>

#include "Image.h"

TEST_CASE("image") {
  SECTION("iteration") {
    HeightData d{40, 60};
    REQUIRE(d.size() == 40*60);
    REQUIRE(d.height() == 60);
    REQUIRE(d.width() == 40);
    int v = 0;
    std::ranges::for_each(d, [&](auto& d) {d = float(v++); });

    v = 0;
    for (unsigned r = 0; r < d.height(); ++r) {
      for (unsigned c = 0; c < d.width(); ++c) {
        REQUIRE(d.pixel(r, c) == Approx(float(v++)));
      }
    }

  }

  SECTION("conversion") {
    HeightData d{40, 60};
    int v_init = 0;
    std::ranges::for_each(d, [&](auto& d) {d = float(v_init++); });

    Image<glm::u8vec3> image = image::transformImage(d, [&](auto v) { const auto b =  uchar(255.F * v / float(v_init)); return glm::u8vec3(b, b, b); });
    const auto max = float(v_init);
    v_init = 0;
    for (unsigned r = 0; r < d.height(); ++r) {
      for (unsigned c = 0; c < d.width(); ++c) {
        const auto t = uchar(float(v_init) * 255.F / max);
        REQUIRE(image.pixel(r, c).x == t);
        REQUIRE(image.pixel(r, c).y == t);
        REQUIRE(image.pixel(r, c).z == t);
        v_init++;
      }
    }
    image::saveImageAsPng(image, "/home/madam/Documents/work/tuw/alpinemaps/tmp/test.png");
  }
}
