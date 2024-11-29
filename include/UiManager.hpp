#ifndef UIMANAGER_HPP
#define UIMANAGER_HPP

#include "ChainKinematics.hpp"
#include "SceneManager.hpp"
#include <threepp/extras/imgui/ImguiContext.hpp>

/**
 * @class UiManager
 * @brief Manages the user interface for the kinematic chain simulation.
 *
 * This class is responsible for setting up and rendering the user interface, handling user interactions,
 * and updating the kinematic chain parameters based on user input.
 */
class UiManager {
public:
    UiManager(SceneManager &scene, ChainKinematics &chainKinematics);
    void render();
    void setupUI();
    void handleKinematics();
    void handleLinkLengths();
    void handleAnimations();
    void handleReset();
    void openLink(std::string &url);
    bool checkParamsChanged();

private:
    SceneManager &scene_;
    ChainKinematics &chainKinematics_;
    ImguiFunctionalContext ui_;
    bool paramsChanged_ = true;
    bool isForwardKinematics_ = true;
    bool isHovered_ = false;
    bool isInteracting_ = false;
};

#endif// UIMANAGER_HPP