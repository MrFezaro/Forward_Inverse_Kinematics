#include "mathutil.hpp"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("test add") {
    constexpr int a = 5;
    constexpr int b = 3;

    const int add2numbers = add(a, b);

    CHECK(add2numbers == a + b);
}
TEST_CASE("test subtract") {
    constexpr int a = 10;
    constexpr int b = 5;

    const int subtract2numbers = subtract(a, b);

    CHECK(subtract2numbers == a - b);
}
