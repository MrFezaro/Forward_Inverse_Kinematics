#ifndef KINEMATICCHAIN_HPP
#define KINEMATICCHAIN_HPP

#include <vector>

struct Point {
    float x, y;
};

class kinematicChain {
public:
    kinematicChain(float L1, float L2, float L3);

    [[nodiscard]] Point forwardKinematics(float theta1, float theta2, float theta3) const;
    bool inverseKinematicsCCD(const Point &target, float &theta1, float &theta2, float &theta3) const;
    [[nodiscard]] static float normalizeAngle(float angle);

    void updateLinkLengths(float newL1, float newL2, float newL3);

private:
    float L1, L2, L3;

    [[nodiscard]] static std::vector<std::vector<float>> transformationMatrix(float angle, float length);
    [[nodiscard]] static std::vector<std::vector<float>> matrixMultiply(const std::vector<std::vector<float>> &A, const std::vector<std::vector<float>> &B);
};

#endif// KINEMATICCHAIN_HPP