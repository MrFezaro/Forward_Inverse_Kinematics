#include "animationLoop.hpp"
#include "chainGeometry.hpp"
#include "sceneManager.hpp"
#include "uiManager.hpp"

sceneManager scene;
kinematicChain kinematicChain{};
uiManager ui(scene, kinematicChain);
chainGeometry chainGeometry(scene.scene, kinematicChain);

int main() {
    ui.render();
    chainGeometry.createKinematicChain();
    runAnimationLoop(scene, kinematicChain, ui, chainGeometry);
}