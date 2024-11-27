#ifndef UIMANAGER_HPP
#define UIMANAGER_HPP
#define _USE_MATH_DEFINES

#include <cmath>
#include <threepp/extras/imgui/ImguiContext.hpp>
#include "chainKinematics.hpp"
#include "sceneManager.hpp"

class uiManager {
public:
    uiManager(sceneManager &scene, chainKinematics &kinematicChainInstance);
    void render();
    void setupUI();
    void handleKinematics();
    void handleLinkLengths();
    void handleReset();
    bool paramsChanged = false;
    bool isForwardKinematics = true;

private:
    sceneManager &scene;
    chainKinematics &kinematicChainInstance;
    ImguiFunctionalContext ui;
    bool isHovered = false;
    bool isInteracting = false;
};

#endif// UIMANAGER_HPP