#include "kinematicChain.hpp"
#include <cmath>

kinematicChain::kinematicChain(const float L1, const float L2, const float L3) : L1(L1), L2(L2), L3(L3) {}

std::vector<std::vector<float>> kinematicChain::transformationMatrix(const float angle, const float length) {
    return {
            {cos(angle), -sin(angle), length * cos(angle)},
            {sin(angle), cos(angle), length * sin(angle)},
            {0, 0, 1}};
}

std::vector<std::vector<float>> kinematicChain::matrixMultiply(const std::vector<std::vector<float>> &A, const std::vector<std::vector<float>> &B) {
    std::vector result(3, std::vector<float>(3, 0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
}

Point kinematicChain::forwardKinematics(const float theta1, const float theta2, const float theta3) const {
    const auto T1 = transformationMatrix(theta1, L1);
    const auto T2 = transformationMatrix(theta2, L2);
    const auto T3 = transformationMatrix(theta3, L3);

    const auto T12 = matrixMultiply(T1, T2);
    const auto T123 = matrixMultiply(T12, T3);

    return {T123[0][2], T123[1][2]};
}

bool kinematicChain::inverseKinematicsCCD(const Point &target, float &theta1, float &theta2, float &theta3) const {
    const float maxReach = L1 + L2 + L3;

    if (const float distanceToTarget = sqrt(target.x * target.x + target.y * target.y); distanceToTarget > maxReach) {
        return false;
    }

    constexpr int maxIterations = 1000;

    for (int iter = 0; iter < maxIterations; ++iter) {
        constexpr float tolerance = 1e-3f;
        auto [x, y] = forwardKinematics(theta1, theta2, theta3);

        const float errorX = target.x - x;
        const float errorY = target.y - y;

        if (const float error = sqrt(errorX * errorX + errorY * errorY); error < tolerance) return true;

        for (int joint = 2; joint >= 0; --joint) {
            float &theta = (joint == 0) ? theta1 : (joint == 1) ? theta2
                                                                : theta3;
            auto [x, y] = forwardKinematics(theta1, theta2, theta3);

            const float jointX = (joint == 0) ? 0 : (joint == 1) ? L1 * cos(theta1)
                                                                 : L1 * cos(theta1) + L2 * cos(theta1 + theta2);
            const float jointY = (joint == 0) ? 0 : (joint == 1) ? L1 * sin(theta1)
                                                                 : L1 * sin(theta1) + L2 * sin(theta1 + theta2);

            const float endEffectorX = x;
            const float endEffectorY = y;

            const float targetX = target.x - jointX;
            const float targetY = target.y - jointY;

            const float angleToTarget = atan2(targetY, targetX);
            const float angleToEndEffector = atan2(endEffectorY - jointY, endEffectorX - jointX);

            theta += angleToTarget - angleToEndEffector;
        }
    }

    return false;
}

float kinematicChain::normalizeAngle(float angle) {
    while (angle < 0) angle += 360.0f;
    while (angle >= 360.0f) angle -= 360.0f;
    return angle;
}

void kinematicChain::updateLinkLengths(const float newL1, const float newL2, const float newL3) {
    L1 = newL1;
    L2 = newL2;
    L3 = newL3;
}
