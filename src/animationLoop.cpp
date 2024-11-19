#include "animationLoop.hpp"
using namespace threepp;

void runAnimationLoop(sceneManager &scene, kinematicChain &chain, uiManager &ui, GeometryHelpers &geometryHelpers) {
    Clock clock;
    scene.canvas.animate([&]() {
        scene.renderer.render(scene.scene, scene.camera);

        ui.render();

        if (ui.paramsChanged) {
            ui.paramsChanged = false;

            if (ui.isForwardKinematics) {
                // Apply rotations to the joints
                const std::vector<float> &jointAngles = chain.getJointAngles();
                geometryHelpers.joint1->rotation.z = jointAngles[0];
                geometryHelpers.joint2->rotation.z = jointAngles[1];
                geometryHelpers.joint3->rotation.z = jointAngles[2];
            } else {
                if (chain.inverseKinematicsCCD()) {
                    const std::vector<float> &jointAngles = chain.getJointAngles();
                    // Apply rotations to the joints
                    geometryHelpers.joint1->rotation.z = jointAngles[0];
                    geometryHelpers.joint2->rotation.z = jointAngles[1];
                    geometryHelpers.joint3->rotation.z = jointAngles[2];
                }
            }
        }
    });
}