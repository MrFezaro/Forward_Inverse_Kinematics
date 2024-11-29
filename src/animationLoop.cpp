#include "animationLoop.hpp"

using namespace threepp;

void runAnimationLoop(SceneManager &scene, ChainKinematics &kinematicChainInstance, UiManager &ui, ChainGeometry &chainGeometry) {
    Clock clock;
    scene.canvas.animate([&]() {
        scene.renderer.render(scene.scene, scene.camera);

        ui.render();

        if (ui.checkParamsChanged()) {
            const std::vector<float> &jointAngles = kinematicChainInstance.getJointAngles();
            const std::vector<float> &linkLengths = kinematicChainInstance.getLinkLengths();

            chainGeometry.joint1->rotation.z = jointAngles[0];
            chainGeometry.joint2->rotation.z = jointAngles[1];
            chainGeometry.joint3->rotation.z = jointAngles[2];

            chainGeometry.link1->scale.y = linkLengths[0];
            chainGeometry.link2->scale.y = linkLengths[1];
            chainGeometry.link3->scale.y = linkLengths[2];

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