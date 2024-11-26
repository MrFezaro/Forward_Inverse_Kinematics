#include "kinematicChain.hpp"

kinematicChain::kinematicChain() = default;

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

point kinematicChain::forwardKinematics() const {
    const auto T1 = transformationMatrix(jointAngles[0], linkLengths[0]);
    const auto T2 = transformationMatrix(jointAngles[1], linkLengths[1]);
    const auto T3 = transformationMatrix(jointAngles[2], linkLengths[2]);

    const auto T12 = matrixMultiply(T1, T2);
    const auto T123 = matrixMultiply(T12, T3);

    return {T123[0][2], T123[1][2]};
}

bool kinematicChain::inverseKinematicsCCD() {
    const float maxReach = linkLengths[0] + linkLengths[1] + linkLengths[2];

    if (const float distanceToTarget = sqrt(target.x * target.x + target.y * target.y); distanceToTarget > maxReach) {
        return false;
    }

    constexpr int maxIterations = 1000;

    for (int iter = 0; iter < maxIterations; ++iter) {
        constexpr float tolerance = 1e-3f;
        auto [x, y] = forwardKinematics();

        const float errorX = target.x - x;
        const float errorY = target.y - y;

        if (const float error = sqrt(errorX * errorX + errorY * errorY); error < tolerance) return true;

        for (int joint = 2; joint >= 0; --joint) {
            float &theta = jointAngles[joint];
            auto [x, y] = forwardKinematics();

            const float jointX = (joint == 0) ? 0 : (joint == 1) ? linkLengths[0] * cos(jointAngles[0])
                                                                 : linkLengths[0] * cos(jointAngles[0]) + linkLengths[1] * cos(jointAngles[0] + jointAngles[1]);
            const float jointY = (joint == 0) ? 0 : (joint == 1) ? linkLengths[0] * sin(jointAngles[0])
                                                                 : linkLengths[0] * sin(jointAngles[0]) + linkLengths[1] * sin(jointAngles[0] + jointAngles[1]);
            const float endEffectorX = x;
            const float endEffectorY = y;

            const float targetX = target.x - jointX;
            const float targetY = target.y - jointY;

            const float angleToTarget = atan2(targetY, targetX);
            const float angleToEndEffector = atan2(endEffectorY - jointY, endEffectorX - jointX);

            theta += angleToTarget - angleToEndEffector;

            theta = normalizeAngle(theta);
        }
    }
    return false;
}

void kinematicChain::setLinkLength(const int linkNumber, const float newLength) {
    if (linkNumber >= 0 && linkNumber < linkLengths.size()) {
        linkLengths[linkNumber] = newLength;
    }
}

void kinematicChain::setJointAngles(const int jointNumber, const float newAngle) {
    if (jointNumber >= 0 && jointNumber < jointAngles.size()) {
        jointAngles[jointNumber] = newAngle * (M_PI / 180.0f);
    }
}

void kinematicChain::setTarget(const point &newTarget) {
    target = newTarget;
}

const std::vector<float> &kinematicChain::getLinkLengths() const {
    return linkLengths;
}

const std::vector<float> &kinematicChain::getJointAngles() const {
    return jointAngles;
}

point kinematicChain::getTarget() const {
    return target;
}

float kinematicChain::normalizeAngle(float angle) {
    constexpr float TWO_PI = 2.0f * M_PI;
    while (angle < 0) angle += TWO_PI;
    while (angle >= TWO_PI) angle -= TWO_PI;
    return angle;
}
