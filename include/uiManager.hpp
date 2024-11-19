#ifndef UIMANAGER_HPP
#define UIMANAGER_HPP

#include "kinematicChain.hpp"
#include "sceneManager.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"

class uiManager {
public:
    uiManager(sceneManager &scene, kinematicChain &kinematicChainInstance);
    void render();
    bool paramsChanged = false;
    bool isForwardKinematics = true;

private:
    sceneManager &scene;
    kinematicChain &kinematicChainInstance;
    ImguiFunctionalContext ui;
};

#endif // UIMANAGER_HPP