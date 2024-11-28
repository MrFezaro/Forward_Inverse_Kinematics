#ifndef KINEMATICCHAIN_HPP
#define KINEMATICCHAIN_HPP

#include <vector>

struct Point {
    float x, y;
};

class ChainKinematics {
public:
    ChainKinematics();

    Point forwardKinematics() const;
    bool inverseKinematicsCCD();

    [[nodiscard]] static float normalizeAngle(float angle);
    [[nodiscard]] const std::vector<float> &getLinkLengths() const;
    [[nodiscard]] const std::vector<float> &getJointAngles() const;
    [[nodiscard]] Point getTarget() const;

    void setLinkLength(int linkNumber, float newLength);
    void setJointAngles(int jointNumber, float newAngle);
    void setTarget(const Point &newTarget);

private:
    std::vector<float> linkLengths = {2.0f, 2.0f, 2.0f};
    std::vector<float> jointAngles = {0.0f, 0.0f, 0.0f};
    Point endEffectorPosition = {6.0f, 0.0f};
    Point target = {6.0f, 0.0f};

    [[nodiscard]] static std::vector<std::vector<float>> transformationMatrix(float angle, float length);
    [[nodiscard]] static std::vector<std::vector<float>> matrixMultiply(const std::vector<std::vector<float>> &A, const std::vector<std::vector<float>> &B);
};

#endif// KINEMATICCHAIN_HPP