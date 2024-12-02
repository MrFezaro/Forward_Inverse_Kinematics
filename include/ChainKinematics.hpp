#ifndef KINEMATICCHAIN_HPP
#define KINEMATICCHAIN_HPP

#include <vector>

struct Point {
    float x, y;
};

/**
 * @class ChainKinematics
 * @brief Handles the kinematic calculations for the kinematic chain.
 *
 * This class is responsible for performing forward and inverse kinematics calculations
 * for a kinematic chain, as well as managing the link lengths, joint angles, and target position.
 */

class ChainKinematics {
public:
    ChainKinematics();

    /**
     * @brief Performs forward kinematics calculations.
     *
     * This function calculates the position of the end effector based on the current joint angles and link lengths.
     *
     * @return The position of the end effector.
     */
    [[nodiscard]] Point forwardKinematics() const;

    /**
     * @brief Performs inverse kinematics calculations using the CCD algorithm.
     *
     * This function adjusts the joint angles to move the end effector towards the target position.
     *
     * @return True if the target is reached, false otherwise.
     */
    bool inverseKinematicsCCD();

    [[nodiscard]] float getLinkLength(int linkNumber) const;
    [[nodiscard]] float getJointAngle(int jointNumber) const;
    [[nodiscard]] Point getTarget() const;

    void setLinkLength(int linkNumber, float newLength);
    void setJointAngle(int jointNumber, float newAngle);
    void setTarget(const Point &newTarget);

private:
    std::vector<float> linkLengths_ = {2.0f, 2.0f, 2.0f};
    std::vector<float> jointAngles_ = {0.0f, 0.0f, 0.0f};
    Point endEffector_ = {6.0f, 0.0f};
    Point target_ = {6.0f, 0.0f};

    /**
     * @brief Normalizes an angle to the range [0, 2pi].
     *
     * @param angle The angle to normalize.
     * @return The normalized angle.
     */
    static float normalizeAngle(float angle);

    /**
     * @brief Creates a transformation matrix for a given angle and length.
     *
     * @param angle The angle of the transformation.
     * @param length The length of the transformation.
     * @return The transformation matrix.
     */
    static std::vector<std::vector<float>> transformationMatrix(float angle, float length);

    /**
     * @brief Multiplies two matrices.
     *
     * @param A The first matrix.
     * @param B The second matrix.
     * @return The result of the matrix multiplication.
     */
    static std::vector<std::vector<float>> matrixMultiply(const std::vector<std::vector<float>> &A, const std::vector<std::vector<float>> &B);
};

#endif// KINEMATICCHAIN_HPP