#ifndef UIMANAGER_HPP
#define UIMANAGER_HPP

#include "ChainKinematics.hpp"
#include "SceneManager.hpp"
#include <threepp/extras/imgui/ImguiContext.hpp>

class UiManager {
public:
    UiManager(SceneManager &scene, ChainKinematics &kinematicChainInstance);
    void render();
    void setupUI();
    void handleKinematics();
    void handleLinkLengths();
    void handleReset();
    bool paramsChanged = false;
    bool isForwardKinematics = true;

private:
    SceneManager &scene;
    ChainKinematics &kinematicChainInstance;
    ImguiFunctionalContext ui;
    bool isHovered = false;
    bool isInteracting = false;
};

#endif// UIMANAGER_HPP