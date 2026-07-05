#include <sstream>
#include <cmath>
#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include "gcode.hpp"

using namespace gcode;

TEST_CASE("Gcode Parser", "[gcode]") {

    SECTION("basic G1 line") {
        Move move = parse_line("G1 X10 Y-3.0 Z0.5 F6000.0");
        REQUIRE(move.cmd == "G1");
        REQUIRE_THAT(move.x, Catch::Matchers::WithinAbs(10.0, 1e-3));
        REQUIRE_THAT(move.y, Catch::Matchers::WithinAbs(-3.0, 1e-3));
        REQUIRE_THAT(move.z, Catch::Matchers::WithinAbs(0.5, 1e-3));
    }

    SECTION("G2 line with I and J ignored") {
        Move move = parse_line("G2 X5 Y10 I5 J0");
        REQUIRE(move.cmd == "G2");
    }
  

    SECTION("negative values") {
        Move move = parse_line("G1 X-15.5 Y-20.3 Z-0.1");
        REQUIRE_THAT(move.x, Catch::Matchers::WithinAbs(-15.5, 1e-3));
        REQUIRE_THAT(move.y, Catch::Matchers::WithinAbs(-20.3, 1e-3));
        REQUIRE_THAT(move.z, Catch::Matchers::WithinAbs(-0.1, 1e-3));
    }
   
}