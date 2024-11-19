#ifndef UIMANAGER_HPP
#define UIMANAGER_HPP

#include "sceneManager.hpp"
#include "kinematicChain.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"

class uiManager {
public:
    uiManager(sceneManager &scene, kinematicChain &kinematicChainInstance);
    uiManager(sceneManager &scene, kinematicChain &kinematicChainInstance, bool &paramsChanged, bool &isForwardKinematics);
    void render();

private:
    sceneManager &scene;
    kinematicChain &kinematicChainInstance;
    ImguiFunctionalContext ui;
    bool paramsChanged = false;
    bool isForwardKinematics = true;
};

#endif // UIMANAGER_HPP