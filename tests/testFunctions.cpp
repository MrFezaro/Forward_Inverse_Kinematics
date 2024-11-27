#include "chainKinematics.cpp"
#include <catch2/catch_approx.hpp>
#include <catch2/catch_test_macros.hpp>

using Catch::Approx;

TEST_CASE("Forward Kinematics") {
    const chainKinematics kc;
    auto [x, y] = kc.forwardKinematics();
    CHECK(x == Approx(6.0f));
    CHECK(y == Approx(0.0f));
}

TEST_CASE("Inverse Kinematics") {
    chainKinematics kc;
    constexpr point target = {2.0f, 2.0f};
    kc.setTarget(target);
    const bool success = kc.inverseKinematicsCCD();
    REQUIRE(success == true);
    CHECK(kc.getTarget().x == Approx(target.x));
    CHECK(kc.getTarget().y == Approx(target.y));
}

TEST_CASE("Get Link Lengths") {
    const chainKinematics kc;
    const std::vector<float> &linkLengths = kc.getLinkLengths();
    REQUIRE(linkLengths.size() == 3);
    CHECK(linkLengths[0] == Approx(2.0f));
    CHECK(linkLengths[1] == Approx(2.0f));
    CHECK(linkLengths[2] == Approx(2.0f));
}

TEST_CASE("Set and Get Joint Angles") {
    chainKinematics kc;
    kc.setJointAngles(0, 45);
    kc.setJointAngles(1, 45);
    kc.setJointAngles(2, 45);
    std::vector<float> jointAngles = kc.getJointAngles();
    REQUIRE(jointAngles.size() == 3);
    CHECK(jointAngles[0] == Approx(M_PI / 4));
    CHECK(jointAngles[1] == Approx(M_PI / 4));
    CHECK(jointAngles[2] == Approx(M_PI / 4));
}

TEST_CASE("Normalize Angle") {
    constexpr float angle = M_PI * 3;
    const float normalizedAngle = chainKinematics::normalizeAngle(angle);
    CHECK(normalizedAngle == Approx(M_PI));
}