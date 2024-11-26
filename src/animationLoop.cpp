#include "animationLoop.hpp"

using namespace threepp;

void runAnimationLoop(sceneManager &scene, kinematicChain &kinematicChain, uiManager &ui, chainGeometry &chainGeometry) {
    Clock clock;
    scene.canvas.animate([&]() {
        scene.renderer.render(scene.scene, scene.camera);

        ui.render();

        if (ui.paramsChanged) {
            ui.paramsChanged = false;

            // Retrieve the joint angles and link lengths
            const std::vector<float> &jointAngles = kinematicChain.getJointAngles();
            const std::vector<float> &linkLengths = kinematicChain.getLinkLengths();

            // Apply rotations to the joints
            chainGeometry.joint1->rotation.z = jointAngles[0];
            chainGeometry.joint2->rotation.z = jointAngles[1];
            chainGeometry.joint3->rotation.z = jointAngles[2];

            // Update the scale of the links
            chainGeometry.link1->scale.y = linkLengths[0];
            chainGeometry.link2->scale.y = linkLengths[1];
            chainGeometry.link3->scale.y = linkLengths[2];

            // Update the positions of the joints and links
            chainGeometry.joint1->position.y = -1.0f;
            chainGeometry.link1->position.y = -linkLengths[0] / 2.0f;
            chainGeometry.joint2->position.y = -linkLengths[0];
            chainGeometry.link2->position.y = -linkLengths[1] / 2.0f;
            chainGeometry.joint3->position.y = -linkLengths[1];
            chainGeometry.link3->position.y = -linkLengths[2] / 2.0f;
            chainGeometry.sphere->position.y = -linkLengths[2];
        }
    });
}