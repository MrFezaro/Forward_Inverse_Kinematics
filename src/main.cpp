#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"
#include <cmath>
#include <iostream>
#include <vector>

constexpr float PI = 3.14159265358979323846f;

// Structure for 2D points
struct Point {
    float x, y;
};

// Transformation matrix function
std::vector<std::vector<float>> transformationMatrix(const float angle, const float length) {
    return {
            {cos(angle), -sin(angle), length * cos(angle)},
            {sin(angle), cos(angle), length * sin(angle)},
            {0, 0, 1}};
}

// Multiply two 3x3 matrices
std::vector<std::vector<float>> matrixMultiply(const std::vector<std::vector<float>> &A,
                                               const std::vector<std::vector<float>> &B) {
    std::vector<std::vector<float>> result(3, std::vector<float>(3, 0));
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            for (int k = 0; k < 3; ++k) {
                result[i][j] += A[i][k] * B[k][j];
            }
        }
    }
    return result;
}

// Forward Kinematics: Given joint angles and link lengths, find end effector position
Point forwardKinematics(const float theta1, const float theta2, const float theta3, const float L1, const float L2, const float L3) {
    const auto T1 = transformationMatrix(theta1, L1);
    const auto T2 = transformationMatrix(theta2, L2);
    const auto T3 = transformationMatrix(theta3, L3);

    // Combine transformations
    const auto T12 = matrixMultiply(T1, T2);
    const auto T123 = matrixMultiply(T12, T3);

    // End effector position
    return {T123[0][2], T123[1][2]};
}

// Inverse Kinematics using CCD: Iterative solution to find joint angles for a desired position
bool inverseKinematicsCCD(const Point target, const float L1, const float L2, const float L3, float &theta1, float &theta2, float &theta3) {
    const float maxReach = L1 + L2 + L3;

    if (const float distanceToTarget = sqrt(target.x * target.x + target.y * target.y); distanceToTarget > maxReach) {
        std::cerr << "Target position is outside the reachable workspace.\n";
        return false;
    }

    constexpr int maxIterations = 100;

    for (int iter = 0; iter < maxIterations; ++iter) {
        constexpr float tolerance = 1e-3f;
        // Current end effector position
        auto [x, y] = forwardKinematics(theta1, theta2, theta3, L1, L2, L3);

        // Calculate the error
        const float errorX = target.x - x;
        const float errorY = target.y - y;

        if (const float error = sqrt(errorX * errorX + errorY * errorY); error < tolerance) return true;// Stop if close enough to target

        // Adjust each joint angle in reverse order
        for (int joint = 2; joint >= 0; --joint) {
            float &theta = (joint == 0) ? theta1 : (joint == 1) ? theta2
                                                                : theta3;
            auto [x, y] = forwardKinematics(theta1, theta2, theta3, L1, L2, L3);

            // Calculate the vector from the joint to the end effector
            const float jointX = (joint == 0) ? 0 : (joint == 1) ? L1 * cos(theta1)
                                                                 : L1 * cos(theta1) + L2 * cos(theta1 + theta2);
            const float jointY = (joint == 0) ? 0 : (joint == 1) ? L1 * sin(theta1)
                                                                 : L1 * sin(theta1) + L2 * sin(theta1 + theta2);

            const float endEffectorX = x;
            const float endEffectorY = y;

            // Calculate the vector from the joint to the target
            const float targetX = target.x - jointX;
            const float targetY = target.y - jointY;

            // Calculate the angle between the two vectors
            const float angleToTarget = atan2(targetY, targetX);
            const float angleToEndEffector = atan2(endEffectorY - jointY, endEffectorX - jointX);

            // Adjust the joint angle
            theta += angleToTarget - angleToEndEffector;
        }
    }

    return false;// Return false if the error is not small enough after maxIterations
}

float normalizeAngle(float angle) {
    while (angle < 0) angle += 360;
    while (angle >= 360) angle -= 360;
    return angle;
}

using namespace threepp;

namespace {
    auto createJoint(const BoxGeometry::Params &params, const Color &color) {
        const auto geometry = BoxGeometry::create(params);
        const auto material = MeshBasicMaterial::create({{"color", color}});
        return Mesh::create(geometry, material);
    }

    auto createLink(const float length, const float radius, const Color &color) {
        const auto geometry = CylinderGeometry::create(radius, radius, length, 32);
        const auto material = MeshBasicMaterial::create({{"color", color}});
        auto link = Mesh::create(geometry, material);
        link->rotation.x = math::degToRad(90);// Align along Y axis
        return link;
    }

    auto createSphere(const float radius, const Color &color) {
        const auto geometry = SphereGeometry::create(radius, 32, 32);
        const auto material = MeshBasicMaterial::create({{"color", color}});
        return Mesh::create(geometry, material);
    }

    void setLinkLength(float &linkLength, const float newLength, const std::shared_ptr<Mesh> &link, const std::shared_ptr<Mesh> &joint) {
        linkLength = newLength;
        link->scale.y = newLength;
        link->position.y = -newLength / 2.0f;
        joint->position.y = -newLength;
    }
}// namespace

int main() {
    Canvas canvas("Forward and inverse kinematic test", {{"aa", 8}});
    GLRenderer renderer(canvas.size());

    Scene scene;
    scene.background = Color::black;
    PerspectiveCamera camera(60, canvas.aspect(), 0.1f, 100);
    camera.position.z = 10;

    OrbitControls controls(camera, canvas);

    // Create base
    BoxGeometry::Params baseParams{1.0f, 1.0f, 1.0f};
    auto base = createJoint(baseParams, Color::white);
    scene.add(base);
    base->position.y = 2.0f;
    base->rotation.y = math::degToRad(180);// Rotate base to align with the world

    // Create joints
    BoxGeometry::Params jointParams{0.5f, 0.5f, 0.5f};
    auto joint1 = createJoint(jointParams, Color::green);
    auto joint2 = createJoint(jointParams, Color::red);
    auto joint3 = createJoint(jointParams, Color::blue);// New joint at the end of the arm

    // Create links (cylinders)
    auto link1 = createLink(1.0f, 0.125f, Color::yellow);
    auto link2 = createLink(1.0f, 0.125f, Color::yellow);
    auto link3 = createLink(1.0f, 0.125f, Color::yellow);// New link at the end of the arm

    // Attach joints hierarchically
    base->add(joint1);
    joint1->add(link1);                    // Attach link1 between base and joint1
    joint1->position.y = -1.0f;            // Position joint1 below base
    link1->position.y = -0.5f;             // Position link1 between base and joint1
    link1->rotation.z = math::degToRad(90);// Rotate link1 to align with joint1
    link1->rotation.y = math::degToRad(90);// Rotate link1 to align with joint1

    joint1->add(joint2);
    joint2->add(link2);// Attach link2 between joint1 and joint2
    joint2->position.y = -1.0f;
    link2->position.y = -0.5f;             // Position link2 between joint1 and joint2
    link2->rotation.z = math::degToRad(90);// Rotate link2 to align with joint2
    link2->rotation.y = math::degToRad(90);// Rotate link2 to align with joint2

    joint2->add(joint3);
    joint3->add(link3);// Attach link3 between joint2 and joint3
    joint3->position.y = -1.0f;
    link3->position.y = -0.5f;             // Position link3 between joint2 and joint3
    link3->rotation.z = math::degToRad(90);// Rotate link3 to align with joint3
    link3->rotation.y = math::degToRad(90);// Rotate link3 to align with joint3


    // Create and attach a sphere to the end of joint3
    auto sphere = createSphere(0.3f, Color::white);// Sphere with radius 0.3
    joint3->add(sphere);
    sphere->position.y = -1.0f;// Position the sphere at the end of joint3

    // Initialize rotation and length parameters for each joint and link
    float angleJoint1 = 0.0f;
    float angleJoint2 = 0.0f;
    float angleJoint3 = 0.0f;
    float lengthLink1 = 2.0f;
    float lengthLink2 = 2.0f;
    float lengthLink3 = 2.0f;

    bool paramsChanged = false;
    bool isForwardKinematics = true;         // Variable to track the current mode
    Point target = {0.0f, 0.0f};             // Target position for inverse kinematics
    Point endEffectorPosition = {0.0f, 0.0f};// Variable to store the end effector position

    auto ui = ImguiFunctionalContext(canvas.windowPtr(), [&] {
        ImGui::SetNextWindowPos({0, 0}, 0, {0, 0});
        ImGui::SetNextWindowSize({320, 0}, 0);
        ImGui::Begin("Joint Controls");

        // Check if the mouse is over the ImGui window or any ImGui item is active
        const bool isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
        const bool isInteracting = ImGui::IsAnyItemActive();

        if (ImGui::Button(isForwardKinematics ? "Switch to Inverse Kinematics" : "Switch to Forward Kinematics")) {
            isForwardKinematics = !isForwardKinematics;
            if (!isForwardKinematics) {
                target = endEffectorPosition;
                paramsChanged = true;
            }
        }

        if (isForwardKinematics) {
            ImGui::SliderFloat("Angle Joint 1", &angleJoint1, 0.0f, 360.0f);
            paramsChanged = paramsChanged || ImGui::IsItemEdited();
            ImGui::SliderFloat("Angle Joint 2", &angleJoint2, 0.0f, 360.0f);
            paramsChanged = paramsChanged || ImGui::IsItemEdited();
            ImGui::SliderFloat("Angle Joint 3", &angleJoint3, 0.0f, 360.0f);
            paramsChanged = paramsChanged || ImGui::IsItemEdited();

            // Display the end effector position
            ImGui::Text("End Effector Position:");
            ImGui::Text("X: %.2f", endEffectorPosition.x);
            ImGui::Text("Y: %.2f", endEffectorPosition.y);
        } else {
            auto targetX = static_cast<float>(target.x);
            auto targetY = static_cast<float>(target.y);
            if (ImGui::SliderFloat("Target X", &targetX, -10.0f, 10.0f)) {
                target.x = static_cast<float>(targetX);
                paramsChanged = true;
            }
            if (ImGui::SliderFloat("Target Y", &targetY, -10.0f, 10.0f)) {
                target.y = static_cast<float>(targetY);
                paramsChanged = true;
            }

            // Display the calculated joint angles
            ImGui::Text("Calculated Angles:");
            ImGui::Text("Angle Joint 1: %.2f", angleJoint1);
            ImGui::Text("Angle Joint 2: %.2f", angleJoint2);
            ImGui::Text("Angle Joint 3: %.2f", angleJoint3);
        }

        // Add sliders to change the lengths of the links
        if (ImGui::SliderFloat("Length Link 1", &lengthLink1, 1.0f, 5.0f)) {
            setLinkLength(lengthLink1, lengthLink1, link1, joint2);
            paramsChanged = true;
        }
        if (ImGui::SliderFloat("Length Link 2", &lengthLink2, 1.0f, 5.0f)) {
            setLinkLength(lengthLink2, lengthLink2, link2, joint3);
            paramsChanged = true;
        }
        if (ImGui::SliderFloat("Length Link 3", &lengthLink3, 1.0f, 5.0f)) {
            setLinkLength(lengthLink3, lengthLink3, link3, sphere);
            paramsChanged = true;
        }

        ImGui::End();

        // Disable OrbitControls if the mouse is over the ImGui window or any ImGui item is active
        controls.enabled = !(isHovered || isInteracting);
    });

    // Apply initial rotations and lengths to the joints and links
    joint1->rotation.z = math::degToRad(angleJoint1);
    joint2->rotation.z = math::degToRad(angleJoint2);
    joint3->rotation.z = math::degToRad(angleJoint3);
    link1->scale.y = lengthLink1;
    link2->scale.y = lengthLink2;
    link3->scale.y = lengthLink3;
    joint1->position.y = -1.0f;// Fix joint1 position relative to the base
    link1->position.y = -lengthLink1 / 2.0f;
    joint2->position.y = -lengthLink1;      // Position joint2 based on lengthLink1
    link2->position.y = -lengthLink2 / 2.0f;// Position link2 between joint2 and joint3
    joint3->position.y = -lengthLink2;      // Position joint3 based on lengthLink2
    link3->position.y = -lengthLink3 / 2.0f;// Position link3 between joint3 and sphere
    sphere->position.y = -lengthLink3;      // Position the sphere at the end of link3

    // Animation loop
    Clock clock;
    canvas.animate([&]() {
        renderer.render(scene, camera);

        ui.render();

        if (paramsChanged) {
            paramsChanged = false;

            if (isForwardKinematics) {
                // Apply rotations to the joints
                joint1->rotation.z = math::degToRad(angleJoint1);
                joint2->rotation.z = math::degToRad(angleJoint2);
                joint3->rotation.z = math::degToRad(angleJoint3);

                // Calculate and store the end effector position
                const float theta1 = angleJoint1 * (PI / 180.0f);
                const float theta2 = angleJoint2 * (PI / 180.0f);
                const float theta3 = angleJoint3 * (PI / 180.0f);
                endEffectorPosition = forwardKinematics(theta1, theta2, theta3, lengthLink1, lengthLink2, lengthLink3);
            } else {
                // Perform inverse kinematics to find angles
                float theta1 = angleJoint1 * (PI / 180.0f);
                float theta2 = angleJoint2 * (PI / 180.0f);
                if (float theta3 = angleJoint3 * (PI / 180.0f); inverseKinematicsCCD(target, lengthLink1, lengthLink2, lengthLink3, theta1, theta2, theta3)) {
                    // Convert angles to degrees and normalize
                    angleJoint1 = normalizeAngle(theta1 * (180.0f / PI));
                    angleJoint2 = normalizeAngle(theta2 * (180.0f / PI));
                    angleJoint3 = normalizeAngle(theta3 * (180.0f / PI));

                    // Apply rotations to the joints
                    joint1->rotation.z = math::degToRad(angleJoint1);
                    joint2->rotation.z = math::degToRad(angleJoint2);
                    joint3->rotation.z = math::degToRad(angleJoint3);
                }
            }
        }
    });
}
//Hello Lars!