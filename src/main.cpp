#include "animationLoop.hpp"
#include "chainGeometry.hpp"
#include "chainKinematics.hpp"
#include "sceneManager.hpp"
#include "uiManager.hpp"

sceneManager scene;
chainKinematics kinematicChain{};
uiManager ui(scene, kinematicChain);
chainGeometry chainGeometry(scene.scene, kinematicChain);

int main() {
    ui.render();
    chainGeometry.create();
    runAnimationLoop(scene, kinematicChain, ui, chainGeometry);
}