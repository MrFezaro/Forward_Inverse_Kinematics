#include "threepp/threepp.hpp"

#include "threepp/extras/imgui/ImguiContext.hpp"

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

    // Create links (cylinders)
    auto link1 = createLink(1.0f, 0.125f, Color::yellow);
    auto link2 = createLink(1.0f, 0.125f, Color::yellow);

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

    // Create and attach a sphere to the end of joint3
    auto sphere = createSphere(0.3f, Color::white);// Sphere with radius 0.3
    joint2->add(sphere);
    sphere->position.y = -1.0f;// Position the sphere at the end of joint3

// Initialize rotation and length parameters for each joint and link
float angleJoint1 = 90.0f;
float angleJoint2 = 90.0f;
float angleJoint3 = 0.0f;
float lengthLink1 = 1.5f;
float lengthLink2 = 1.5f;

bool paramsChanged = false;
auto ui = ImguiFunctionalContext(canvas.windowPtr(), [&] {
    ImGui::SetNextWindowPos({0, 0}, 0, {0, 0});
    ImGui::SetNextWindowSize({320, 0}, 0);
    ImGui::Begin("Joint Controls");

    // Check if the mouse is over the ImGui window or any ImGui item is active
    const bool isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
    const bool isInteracting = ImGui::IsAnyItemActive();

    ImGui::SliderFloat("Angle Joint 1", &angleJoint1, 0.0f, 180.0f);
    paramsChanged = paramsChanged || ImGui::IsItemEdited();
    ImGui::SliderFloat("Angle Joint 2", &angleJoint2, 0.0f, 180.0f);
    paramsChanged = paramsChanged || ImGui::IsItemEdited();
    ImGui::SliderFloat("Angle Joint 3", &angleJoint3, 0.0f, 180.0f);
    paramsChanged = paramsChanged || ImGui::IsItemEdited();
    ImGui::SliderFloat("Length Link 1", &lengthLink1, 1.5f, 4.0f);
    paramsChanged = paramsChanged || ImGui::IsItemEdited();
    ImGui::SliderFloat("Length Link 2", &lengthLink2, 1.5f, 4.0f);
    paramsChanged = paramsChanged || ImGui::IsItemEdited();

    ImGui::End();

    // Disable OrbitControls if the mouse is over the ImGui window or any ImGui item is active
    controls.enabled = !(isHovered || isInteracting);
});

    // Apply initial rotations and lengths to the joints and links
    joint1->rotation.x = math::degToRad(angleJoint1 - 90.0f);
    joint1->rotation.z = math::degToRad(angleJoint2 - 90.0f);
    joint2->rotation.z = math::degToRad(angleJoint3);
    link1->scale.y = lengthLink1;
    link2->scale.y = lengthLink2;
    joint1->position.y = -1.0f; // Fix joint1 position relative to the base
    link1->position.y = -lengthLink1 / 2.0f;
    joint2->position.y = -lengthLink1; // Position joint2 based on lengthLink1
    link2->position.y = -lengthLink2 / 2.0f; // Position link2 between joint2 and joint3
    sphere->position.y = -lengthLink2; // Position the sphere at the end of link2

    // Animation loop
    Clock clock;
    canvas.animate([&]() {
        renderer.render(scene, camera);

        ui.render();

        if (paramsChanged) {
            paramsChanged = false;

            // Apply rotations and lengths to the joints and links
            joint1->rotation.x = math::degToRad(angleJoint1 - 90.0f);
            joint1->rotation.z = math::degToRad(angleJoint2 - 90.0f);
            joint2->rotation.z = math::degToRad(angleJoint3);
            link1->scale.y = lengthLink1;
            link2->scale.y = lengthLink2;
            joint1->position.y = -1.0f; // Fix joint1 position relative to the base
            link1->position.y = -lengthLink1 / 2.0f;
            joint2->position.y = -lengthLink1; // Position joint2 based on lengthLink1
            link2->position.y = -lengthLink2 / 2.0f; // Position link2 between joint2 and joint3
            sphere->position.y = -lengthLink2; // Position the sphere at the end of link2
        }
    });
}