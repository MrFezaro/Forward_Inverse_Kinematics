#include "geometryHelpers.hpp"
#include "sceneManager.hpp"
#include "uiManager.hpp"
#include "animationLoop.hpp"

sceneManager scene;
kinematicChain chain{};
uiManager ui(scene, chain);
GeometryHelpers geometryHelpers(scene.scene, chain);

int main() {
    ui.render();
    geometryHelpers.createKinematicChain();
    runAnimationLoop(scene, chain, ui, geometryHelpers);
}