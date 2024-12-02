#include "animationLoop.hpp"

using namespace threepp;

void runAnimationLoop(SceneManager &scene, const ChainKinematics &chainKinematics, UiManager &ui, const ChainGeometry &chainGeometry) {
    Clock clock;
    scene.canvas.animate([&]() {
        scene.renderer.render(scene.scene, scene.camera);
        const float dt = clock.getDelta();
        ui.render(dt);

        if (ui.checkParamsChanged()) {
            for (int i = 0; i < 3; ++i) {
                chainGeometry.joints[i]->rotation.z = chainKinematics.getJointAngle(i);
                chainGeometry.links[i]->scale.y = chainKinematics.getLinkLength(i);
            }
            chainGeometry.joints[0]->position.y = -1.0f;
            chainGeometry.links[0]->position.y = -chainKinematics.getLinkLength(0) / 2.0f;
            chainGeometry.joints[1]->position.y = -chainKinematics.getLinkLength(0);
            chainGeometry.links[1]->position.y = -chainKinematics.getLinkLength(1) / 2.0f;
            chainGeometry.joints[2]->position.y = -chainKinematics.getLinkLength(1);
            chainGeometry.links[2]->position.y = -chainKinematics.getLinkLength(2) / 2.0f;
            chainGeometry.sphere->position.y = -chainKinematics.getLinkLength(2);
        }
    });
}