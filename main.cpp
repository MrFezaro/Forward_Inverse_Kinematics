#include "ChainGeometry.hpp"
#include "ChainKinematics.hpp"
#include "SceneManager.hpp"
#include "UiManager.hpp"
#include "animationLoop.hpp"
#include <iostream>

int main() {
    try {
        SceneManager scene;
        ChainKinematics kinematicChain{};
        UiManager ui(scene, kinematicChain);
        ChainGeometry chainGeometry(scene.scene, kinematicChain);

        ui.render();
        chainGeometry.create();
        runAnimationLoop(scene, kinematicChain, ui, chainGeometry);
    } catch (const std::out_of_range &e) {
        std::cerr << "Out of range error: " << e.what() << std::endl;
    } catch (const std::invalid_argument &e) {
        std::cerr << "Invalid argument error: " << e.what() << std::endl;
    }
    return 0;
}