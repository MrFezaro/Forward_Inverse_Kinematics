#include "ChainKinematics.hpp"
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>

ChainKinematics::ChainKinematics() = default;

std::vector<std::vector<float>> ChainKinematics::transformationMatrix(const float angle, const float length) {
    return {
            {cos(angle), -sin(angle), length * cos(angle)},
            {sin(angle), cos(angle), length * sin(angle)},
            {0, 0, 1}};
}

std::vector<std::vector<float>> ChainKinematics::matrixMultiply(const std::vector<std::vector<float>> &A, const std::vector<std::vector<float>> &B) {
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

Point ChainKinematics::forwardKinematics() const {
    const auto T1 = transformationMatrix(jointAngles_[0], linkLengths_[0]);
    const auto T2 = transformationMatrix(jointAngles_[1], linkLengths_[1]);
    const auto T3 = transformationMatrix(jointAngles_[2], linkLengths_[2]);

    const auto T12 = matrixMultiply(T1, T2);
    const auto T123 = matrixMultiply(T12, T3);

    return {T123[0][2], T123[1][2]};
}

bool ChainKinematics::inverseKinematicsCCD() {
    const float maxReach = linkLengths_[0] + linkLengths_[1] + linkLengths_[2];

    if (const float distanceToTarget = sqrt(target_.x * target_.x + target_.y * target_.y); distanceToTarget > maxReach) {
        return false;
    }

    constexpr int maxIterations = 1000;

    for (int iter = 0; iter < maxIterations; ++iter) {
        constexpr float tolerance = 1e-3f;
        auto [x, y] = forwardKinematics();

        const float errorX = target_.x - x;
        const float errorY = target_.y - y;

        if (const float error = sqrt(errorX * errorX + errorY * errorY); error < tolerance) return true;

        for (int joint = 2; joint >= 0; --joint) {
            float &theta = jointAngles_[joint];
            auto [x, y] = forwardKinematics();

            const float jointX = (joint == 0) ? 0 : (joint == 1) ? linkLengths_[0] * cos(jointAngles_[0])
                                                                 : linkLengths_[0] * cos(jointAngles_[0]) + linkLengths_[1] * cos(jointAngles_[0] + jointAngles_[1]);

            const float jointY = (joint == 0) ? 0 : (joint == 1) ? linkLengths_[0] * sin(jointAngles_[0])
                                                                 : linkLengths_[0] * sin(jointAngles_[0]) + linkLengths_[1] * sin(jointAngles_[0] + jointAngles_[1]);

            const float endEffectorX = x;
            const float endEffectorY = y;

            const float targetX = target_.x - jointX;
            const float targetY = target_.y - jointY;

            const float angleToTarget = atan2(targetY, targetX);
            const float angleToEndEffector = atan2(endEffectorY - jointY, endEffectorX - jointX);

            theta += angleToTarget - angleToEndEffector;

            theta = normalizeAngle(theta);
        }
    }
    return false;
}

void ChainKinematics::setLinkLength(const int linkNumber, const float newLength) {
    if (linkNumber < 0 || linkNumber >= linkLengths_.size()) {
        throw std::out_of_range("Invalid link number: " + std::to_string(linkNumber));
    }
    if (newLength <= 0) {
        throw std::invalid_argument("Link " + std::to_string(linkNumber) + " length " + std::to_string(newLength) + "must be positive");
    }
    linkLengths_[linkNumber] = newLength;
}

void ChainKinematics::setJointAngle(const int jointNumber, const float newAngle) {
    if (jointNumber < 0 || jointNumber >= jointAngles_.size()) {
        throw std::out_of_range("Invalid joint number: " + std::to_string(jointNumber));
    }
    if (newAngle < 0 || newAngle > 360) {
        throw std::out_of_range("Joint angle " + std::to_string(jointNumber) + " (" + std::to_string(newAngle) + ") must be between 0 and 360 degrees ");
    }
    jointAngles_[jointNumber] = newAngle * (std::numbers::pi / 180.0f);
}

void ChainKinematics::setTarget(const Point &newTarget) {
    target_ = newTarget;
}

float ChainKinematics::getLinkLength(const int linkNumber) const {
    if (linkNumber < 0 || linkNumber >= linkLengths_.size()) {
        throw std::out_of_range("Invalid link number: " + std::to_string(linkNumber));
    }
    return linkLengths_[linkNumber];
}

float ChainKinematics::getJointAngle(const int jointNumber) const {
    if (jointNumber < 0 || jointNumber >= jointAngles_.size()) {
        throw std::out_of_range("Invalid joint number: " + std::to_string(jointNumber));
    }
    return jointAngles_[jointNumber];
}

Point ChainKinematics::getTarget() const {
    return target_;
}

float ChainKinematics::normalizeAngle(float angle) {
    constexpr float TWO_PI = 2.0f * std::numbers::pi;
    while (angle < 0) angle += TWO_PI;
    while (angle >= TWO_PI) angle -= TWO_PI;
    return angle;
}