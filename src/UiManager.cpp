#include "uiManager.hpp"
#include <cmath>
#include <cstdlib>
#include <numbers>
#include <string>

UiManager::UiManager(SceneManager &scene, ChainKinematics &chainKinematics)
    : scene_(scene), chainKinematics_(chainKinematics), ui_(scene.canvas.windowPtr(), [&] {
          setupUI();
          handleKinematics();
          handleLinkLengths();
          handleReset();
          handleAnimations();
          scene.controls.enabled = !(isHovered_ || isInteracting_);
          ImGui::End();
      }) {}

void UiManager::render(const float dt) {
    dt_ = dt;
    ui_.render();
}

bool UiManager::checkParamsChanged() {
    if (paramsChanged_) {
        paramsChanged_ = false;
        return true;
    }
    return false;
}

void UiManager::setupUI() {
    ImGui::SetNextWindowPos({0, 0}, 0, {0, 0});
    ImGui::SetNextWindowSize({320, 0}, 0);
    ImGui::Begin("Joint Controls");

    isHovered_ = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
    isInteracting_ = ImGui::IsAnyItemActive();
}

void UiManager::handleKinematics() {
    if (ImGui::Button(isForwardKinematics_ ? "Switch to Inverse Kinematics" : "Switch to Forward Kinematics")) {
        isForwardKinematics_ = !isForwardKinematics_;
        paramsChanged_ = true;
    }

    if (isForwardKinematics_) {
        for (int i = 0; i < 3; ++i) {
            float angle = chainKinematics_.getJointAngle(i) * (180.0f / std::numbers::pi_v<float>);
            if (ImGui::SliderFloat(("Angle Joint " + std::to_string(i + 1)).c_str(), &angle, 0.0f, 360.0f)) {
                chainKinematics_.setJointAngle(i, angle);
                paramsChanged_ = true;
            }
        }
        auto [x, y] = chainKinematics_.forwardKinematics();
        chainKinematics_.setTarget({x, y});
        ImGui::Text("End Effector Position:");
        ImGui::Text("X: %.2f", x);
        ImGui::Text("Y: %.2f", y);

    } else {
        Point target = chainKinematics_.getTarget();
        if (ImGui::SliderFloat("Target X", &target.x, -10.0f, 10.0f)) {
            chainKinematics_.setTarget(target);
            paramsChanged_ = true;
        }

        if (ImGui::SliderFloat("Target Y", &target.y, -10.0f, 10.0f)) {
            chainKinematics_.setTarget(target);
            paramsChanged_ = true;
        }

        ImGui::Text("Calculated Angles:");
        for (int i = 0; i < 3; ++i) {
            ImGui::Text("Angle Joint %d: %.2f", i + 1, chainKinematics_.getJointAngle(i) * (180.0f / std::numbers::pi_v<float>) );
        }

        if (!chainKinematics_.inverseKinematicsCCD()) {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "Target is out of bounds!");
        }
    }
}

void UiManager::handleLinkLengths() {
    for (int i = 0; i < 3; ++i) {
        float length = chainKinematics_.getLinkLength(i);
        if (ImGui::SliderFloat(("Length Link " + std::to_string(i + 1)).c_str(), &length, 1.0f, 5.0f)) {
            chainKinematics_.setLinkLength(i, length);
            paramsChanged_ = true;
        }
    }
}

void UiManager::handleReset() {
    if (ImGui::Button("Reset")) {
        for (int i = 0; i < 3; ++i) {
            chainKinematics_.setJointAngle(i, 0.0f);
            chainKinematics_.setLinkLength(i, 2.0f);
        }
        chainKinematics_.setTarget({6.0f, 0.0f});
        paramsChanged_ = true;
    }
}

void UiManager::handleAnimations() {
    if (!isForwardKinematics_) {
        const char *items[] = {"None", "Square", "Sinus", "Circle", "Special"};
        ImGui::Combo("##AnimationType", &currentAnimation_, items, IM_ARRAYSIZE(items));

        if (ImGui::Button(isAnimating_ ? "Stop Animation" : "Play Animation")) {
            isAnimating_ = !isAnimating_;
            if (!isAnimating_) {
                animationTime_ = 0.0f; //Reset animation time when stopping
            }
        }
        if (isAnimating_) {
            updateAnimation();
        }
    }
}

void UiManager::updateAnimation() {
    float chainLength = 0.0f;
    for (int i = 0; i < 3; ++i) {
        chainLength += chainKinematics_.getLinkLength(i) / 2.0f; //Halved to avoid the chain going out of bounds
    }
    auto [x, y] = chainKinematics_.getTarget();

    const std::string url = "https://www.youtube.com/watch?v=dQw4w9WgXcQ&pp=ygUXbmV2ZXIgZ29ubmEgZ2l2ZSB5b3UgdXA%3D";

    animationTime_ += dt_; //Animation speed

    switch (currentAnimation_) {
        case 0: //None
            break;
        case 1: //Square
            if (animationTime_ < 1.0f) {
                const float t = animationTime_;
                chainKinematics_.setTarget({chainLength * (1.0f - t) + (-chainLength * t),
                                            chainLength});
            } else if (animationTime_ < 2.0f) {
                const float t = animationTime_ - 1.0f;
                chainKinematics_.setTarget({-chainLength,
                                            chainLength * (1.0f - t) + (-chainLength * t)});
            } else if (animationTime_ < 3.0f) {
                const float t = animationTime_ - 2.0f;
                chainKinematics_.setTarget({-chainLength * (1.0f - t) + (chainLength * t),
                                            -chainLength});
            } else if (animationTime_ < 4.0f) {
                const float t = animationTime_ - 3.0f;
                chainKinematics_.setTarget({chainLength,
                                            -chainLength * (1.0f - t) + (chainLength * t)});
            } else {
                animationTime_ = 0.0f;
            }
            break;
        case 2: //Sinus
            chainKinematics_.setTarget({std::sin(animationTime_) * chainLength, animationTime_ - std::numbers::pi_v<float>});
            if (y > 3.14f) {
                animationTime_ = 0.0f;
            }
            break;
        case 3: //Circle
            chainKinematics_.setTarget({(chainLength * std::cos(animationTime_)),
                                        (chainLength * std::sin(animationTime_))});
            break;
        case 4: //Special
#ifdef _WIN32
            std::system(("start " + url).c_str());
#elif __APPLE__
            std::system(("open " + url).c_str());
#elif __linux__
            std::system(("xdg-open " + url).c_str());
#endif
            currentAnimation_ = 0;
            break;
        default:
            break;
    }
    paramsChanged_ = true;
}