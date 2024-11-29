#ifndef UIMANAGER_HPP
#define UIMANAGER_HPP

#include <threepp/extras/imgui/ImguiContext.hpp>
#include "ChainKinematics.hpp"
#include "SceneManager.hpp"

class UiManager {
public:
    UiManager(SceneManager &scene, ChainKinematics &kinematicChainInstance);
    void render();
    void setupUI();
    void handleKinematics();
    void handleLinkLengths();
    void handleReset();
    [[nodiscard]] bool checkParamsChanged() const;

private:
    SceneManager &scene;
    ChainKinematics &kinematicChainInstance;
    ImguiFunctionalContext ui;
    bool paramsChanged = false;
    bool isForwardKinematics = true;
    bool isHovered = false;
    bool isInteracting = false;
};

#endif// UIMANAGER_HPP