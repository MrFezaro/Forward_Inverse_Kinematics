#include "geometryHelpers.hpp"
#include "kinematicChain.hpp"
#include "sceneManager.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"
#include "threepp/threepp.hpp"
#include "uiManager.hpp"

constexpr float PI = 3.14159265358979323846f;

using namespace threepp;

sceneManager scene;
kinematicChain chain{};
uiManager ui(scene, chain);

int main() {
    ui.render();

    // Create base
    const BoxGeometry::Params baseParams{1.0f, 1.0f, 1.0f};
    const auto base = createJoint(baseParams, Color::white);
    scene.scene.add(base);
    base->position.y = 2.0f;
    base->rotation.y = math::degToRad(180);// Rotate base to align with world

    // Create joints
    const BoxGeometry::Params jointParams{0.5f, 0.5f, 0.5f};
    const auto joint1 = createJoint(jointParams, Color::green);
    const auto joint2 = createJoint(jointParams, Color::red);
    const auto joint3 = createJoint(jointParams, Color::blue);

    // Create links (cylinders)
    const auto link1 = createLink(1.0f, 0.125f, Color::yellow);
    const auto link2 = createLink(1.0f, 0.125f, Color::yellow);
    const auto link3 = createLink(1.0f, 0.125f, Color::yellow);

    // Attach joints hierarchically
    base->add(joint1);
    joint1->add(link1);                    // Attach link1 between base and joint1
    joint1->position.y = -1.0f;            // Position joint1 below base
    link1->position.y = -0.5f;             // Position link1 between base and joint1
    link1->rotation.z = math::degToRad(90);// Rotate link1 to align with joint1
    link1->rotation.y = math::degToRad(90);// Rotate link1 to align with joint1

    joint1->add(joint2);
    joint2->add(link2);                    // Attach link2 between joint1 and joint2
    joint2->position.y = -1.0f;            // Position joint2 between joint1 and joint3
    link2->position.y = -0.5f;             // Position link2 between joint1 and joint2
    link2->rotation.z = math::degToRad(90);// Rotate link2 to align with joint2
    link2->rotation.y = math::degToRad(90);// Rotate link2 to align with joint2

    joint2->add(joint3);
    joint3->add(link3);                    // Attach link3 between joint2 and joint3
    joint3->position.y = -1.0f;            // Position joint3 between joint2 and sphere
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
    scene.canvas.animate([&]() {
        scene.renderer.render(scene.scene, scene.camera);

        ui.render();

        if (ui.paramsChanged) {
            ui.paramsChanged = false;

            if (ui.isForwardKinematics) {
                // Apply rotations to the joints
                std::vector<float> jointAngles = chain.getJointAngles();
                for (auto &angle: jointAngles) {
                    angle *= (180.0f / PI);
                }
                joint1->rotation.z = jointAngles[0];
                joint2->rotation.z = jointAngles[1];
                joint3->rotation.z = jointAngles[2];

                // Calculate and store the end effector position

            } else {
                // Perform inverse kinematics to find angles
                float theta1 = angleJoint1 * (PI / 180.0f);
                float theta2 = angleJoint2 * (PI / 180.0f);
                float theta3 = angleJoint3 * (PI / 180.0f);
                if (kinematicChain.inverseKinematicsCCD(target, theta1, theta2, theta3)) {
                    // Convert angles to degrees and normalize
                    angleJoint1 = kinematicChain::normalizeAngle(theta1 * (180.0f / PI));
                    angleJoint2 = kinematicChain::normalizeAngle(theta2 * (180.0f / PI));
                    angleJoint3 = kinematicChain::normalizeAngle(theta3 * (180.0f / PI));

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