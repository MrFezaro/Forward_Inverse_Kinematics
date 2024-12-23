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

    void setupUI();
    void render(float dt);
    bool checkParamsChanged();

private:
    SceneManager &scene_;
    ChainKinematics &chainKinematics_;
    ImguiFunctionalContext ui_;

    void handleReset();
    void handleKinematics();
    void handleLinkLengths();
    void handleAnimations();
    void updateAnimation();

    bool paramsChanged_ = true;
    bool isForwardKinematics_ = true;
    bool isHovered_ = false;
    bool isInteracting_ = false;
    bool isAnimating_ = false;
    int currentAnimation_ = 0;
    float animationTime_ = 0.0f;
    float dt_ = 0.0f;
};

#endif// UIMANAGER_HPP