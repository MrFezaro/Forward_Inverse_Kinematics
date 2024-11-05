#include "mathutil.hpp"
#include <catch2/catch_test_macros.hpp>

TEST_CASE("test add"){
    int a = 5;
    int b = 3;

    int add = add(a, b);

    CHECK(add == a + b);
}

TEST_CASE("Normalize angles") {
    struct TestCase {
        float angle;
        float expected;
    };

    std::vector<TestCase> testCases = {
        {370, 10},
        {-10, 350},
        {720, 0},
        {450, 90},
        {0, 0},
        {360, 0},
        {-360, 0},
        {1080, 0},
        {-370, 350}
    };

    for (const auto& testCase : testCases) {
        float normalized = normalizeAngle(testCase.angle);
        CHECK(normalized == Approx(testCase.expected).margin(0.01));
    }
}