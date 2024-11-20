#ifndef UIMANAGER_HPP
#define UIMANAGER_HPP

#include "kinematicChain.hpp"
#include "sceneManager.hpp"
#include "threepp/extras/imgui/ImguiContext.hpp"

class uiManager {
public:
    uiManager(sceneManager &scene, kinematicChain &kinematicChainInstance);
    void render();
    void setupUI();
    void handleKinematics();
    void handleLinkLengths();
    void handleReset();
    bool paramsChanged = false;
    bool isForwardKinematics = true;

private:
    sceneManager &scene;
    kinematicChain &kinematicChainInstance;
    ImguiFunctionalContext ui;
    bool isHovered = false;
    bool isInteracting = false;
};

#endif// UIMANAGER_HPP