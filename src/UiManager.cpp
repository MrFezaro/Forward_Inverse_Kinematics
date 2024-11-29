#include "uiManager.hpp"
#include <numbers>
#include <string>
#include <windows.h>

UiManager::UiManager(SceneManager &scene, ChainKinematics &chainKinematics)
    : scene_(scene), chainKinematics_(chainKinematics), ui_(scene.canvas.windowPtr(), [&] {
          setupUI();
          handleKinematics();
          handleLinkLengths();
          handleReset();
          handleAnimations();
          ImGui::End();
          scene.controls.enabled = !(isHovered_ || isInteracting_);
      }) {}

void UiManager::render() {
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
            float angle = chainKinematics_.getJointAngle(i) * (180.0f / std::numbers::pi);
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
            ImGui::Text("Angle Joint %d: %.2f", i + 1, chainKinematics_.getJointAngle(i) * (180.0f / std::numbers::pi));
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
    if (ImGui::Button("Open Rick Roll")) {
        const std::string url = "https://www.youtube.com/watch?v=dQw4w9WgXcQ&pp=ygUXbmV2ZXIgZ29ubmEgZ2l2ZSB5b3UgdXA%3D";
        ShellExecute(nullptr, nullptr, url.c_str(), nullptr, nullptr, SW_SHOW);
        paramsChanged_ = true;
    }
}