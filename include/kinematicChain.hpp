#ifndef KINEMATICCHAIN_HPP
#define KINEMATICCHAIN_HPP

#include <vector>

struct point {
    float x, y;
};

class kinematicChain {
public:
    kinematicChain();

    [[nodiscard]] point forwardKinematics() const;
    bool inverseKinematicsCCD();

    [[nodiscard]] const std::vector<float> &getLinkLengths() const;
    [[nodiscard]] const std::vector<float> &getJointAngles() const;
    [[nodiscard]] point getTarget() const;

    [[nodiscard]] static float normalizeAngle(float angle);

    void setLinkLength(int linkNumber, float newLength);
    void setJointAngles(int jointNumber, float newAngle);
    void setTarget(const point &newTarget);

private:
    std::vector<float> linkLengths = {2.0f, 2.0f, 2.0f};
    std::vector<float> jointAngles = {0.0f, 0.0f, 0.0f};
    point endEffectorPosition = {6.0f, 0.0f};
    point target = {6.0f, 0.0f};

    const float PI = 3.14159265358979323846f;

    [[nodiscard]] static std::vector<std::vector<float>> transformationMatrix(float angle, float length);
    [[nodiscard]] static std::vector<std::vector<float>> matrixMultiply(const std::vector<std::vector<float>> &A, const std::vector<std::vector<float>> &B);
};

#endif // KINEMATICCHAIN_HPP