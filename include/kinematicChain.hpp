#ifndef KINEMATICCHAIN_HPP
#define KINEMATICCHAIN_HPP

#include <vector>

struct Point {
    float x, y;
};

class kinematicChain {
public:
    kinematicChain();

    [[nodiscard]] Point forwardKinematics(const std::vector<float> &jointAngles) const;
    bool inverseKinematicsCCD(const Point &target, std::vector<float> &jointAngles) const;

    [[nodiscard]] const std::vector<float> &getLinkLengths() const;
    [[nodiscard]] static float getAngle(float angle);
    void updateLinkLength(int linkNumber, float newLength);

private:
    std::vector<float> linkLengths = {2.0f, 2.0f, 2.0f};
    std::vector<float> jointAngles = {0.0f, 0.0f, 0.0f};
    Point endEffectorPosition = {6.0f, 0.0f};
    // Point target = {6.0f, 0.0f};

    [[nodiscard]] static std::vector<std::vector<float>> transformationMatrix(float angle, float length);
    [[nodiscard]] static std::vector<std::vector<float>> matrixMultiply(const std::vector<std::vector<float>> &A, const std::vector<std::vector<float>> &B);
};

#endif// KINEMATICCHAIN_HPP