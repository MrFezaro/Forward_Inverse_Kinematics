#include "animationLoop.hpp"
#include "ChainGeometry.hpp"
#include "ChainKinematics.hpp"
#include "SceneManager.hpp"
#include "UiManager.hpp"

int main() {
    SceneManager scene;
    ChainKinematics kinematicChain{};
    UiManager ui(scene, kinematicChain);
    ChainGeometry chainGeometry(scene.scene, kinematicChain);

    ui.render();
    chainGeometry.create();
    runAnimationLoop(scene, kinematicChain, ui, chainGeometry);
}