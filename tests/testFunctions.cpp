#include "ChainKinematics.hpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>
#include <numbers>

using Catch::Approx;

// Helper function to initialize a ChainKinematics instance
void initializeChainKinematics(ChainKinematics &ck, const std::vector<float> &linkLengths, const std::vector<float> &jointAngles) {
    for (int i = 0; i < linkLengths.size(); ++i) {
        ck.setLinkLength(i, linkLengths[i]);
    }
    for (int i = 0; i < jointAngles.size(); ++i) {
        ck.setJointAngle(i, jointAngles[i]);
    }
}

TEST_CASE("Forward Kinematics - Valid Case", "[forwardKinematics]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0, 1.5, 1.0}, {30, 45, 60});
    auto [x, y] = ck.forwardKinematics();

    CHECK(x == Approx(1.41).epsilon(0.01));
    CHECK(y == Approx(3.16).epsilon(0.01));
}

TEST_CASE("Inverse Kinematics - Target is Reachable", "[inverseKinematicsCCD]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0, 1.5, 1.0}, {0, 0, 0});
    ck.setTarget({3.0, 1.0});

    const bool result = ck.inverseKinematicsCCD();
    REQUIRE(result == true);

    auto [x, y] = ck.forwardKinematics();
    CHECK(x == Approx(3.0).epsilon(0.01));
    CHECK(y == Approx(1.0).epsilon(0.01));
}

TEST_CASE("Inverse Kinematics - Target is Unreachable", "[inverseKinematicsCCD]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0, 1.5, 1.0}, {0, 0, 0});
    ck.setTarget({10.0, 10.0});

    const bool result = ck.inverseKinematicsCCD();
    CHECK(result == false);
}

TEST_CASE("Set Joint Angles - Invalid Joint Index", "[jointAngles]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0, 1.5, 1.0}, {0, 0, 0});
    REQUIRE_THROWS_AS(ck.setJointAngle(3, 45), std::out_of_range);
}

TEST_CASE("Get Joint Angles - Invalid Joint Index", "[jointAngles]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0, 1.5, 1.0}, {0, 0, 0});
    REQUIRE_THROWS_AS(ck.getJointAngle(3), std::out_of_range);
}

TEST_CASE("Set and Get Joint Angles - Valid Case", "[jointAngles]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0, 1.5, 1.0}, {45, 45, 45});

    for (int i = 0; i < 3; ++i) {
        CHECK(ck.getJointAngle(i) == Approx(45 * std::numbers::pi / 180));
    }
}

TEST_CASE("Set Link Lengths - Invalid Link Index", "[linkLengths]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0f, 2.0f, 2.0f}, {0, 0, 0});
    REQUIRE_THROWS_AS(ck.setLinkLength(3, 3.0f), std::out_of_range);
}

TEST_CASE("Set Link Lengths - Invalid Link Length", "[linkLengths]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0f, 2.0f, 2.0f}, {0, 0, 0});
    REQUIRE_THROWS_AS(ck.setLinkLength(1, -1.0f), std::invalid_argument);
}

TEST_CASE("Get Link Lengths - Invalid Link Index", "[linkLengths]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {2.0f, 2.0f, 2.0f}, {0, 0, 0});
    REQUIRE_THROWS_AS(ck.getLinkLength(3), std::out_of_range);
}

TEST_CASE("Set and Get Link Lengths - Valid Case", "[linkLengths]") {
    ChainKinematics ck;
    initializeChainKinematics(ck, {3.0f, 2.5f, 2.0f}, {0, 0, 0});

    for (int i = 0; i < 3; ++i) {
        CHECK(ck.getLinkLength(i) == Approx((i == 0 ? 3.0f : (i == 1 ? 2.5f : 2.0f))));
    }
}