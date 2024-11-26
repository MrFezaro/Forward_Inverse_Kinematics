#include "uiManager.hpp"
#include "chainGeometry.hpp"
#include "kinematicChain.hpp"

uiManager::uiManager(sceneManager &scene, kinematicChain &kinematicChainInstance)
    : scene(scene), kinematicChainInstance(kinematicChainInstance), ui(scene.canvas.windowPtr(), [&] {
          setupUI();
          handleKinematics();
          handleLinkLengths();
          handleReset();
          scene.controls.enabled = !(isHovered || isInteracting);
      }) {}

void uiManager::render() {
    ui.render();
}

void uiManager::setupUI() {
    ImGui::SetNextWindowPos({0, 0}, 0, {0, 0});
    ImGui::SetNextWindowSize({320, 0}, 0);
    ImGui::Begin("Joint Controls");

    isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
    isInteracting = ImGui::IsAnyItemActive();
}

void uiManager::handleKinematics() {
    if (ImGui::Button(isForwardKinematics ? "Switch to Inverse Kinematics" : "Switch to Forward Kinematics")) {
        isForwardKinematics = !isForwardKinematics;
        paramsChanged = true;
    }
    if (isForwardKinematics) {
        float angle1 = kinematicChainInstance.getJointAngles()[0] * (180.0f / M_PI);
        float angle2 = kinematicChainInstance.getJointAngles()[1] * (180.0f / M_PI);
        float angle3 = kinematicChainInstance.getJointAngles()[2] * (180.0f / M_PI);

        if (ImGui::SliderFloat("Angle Joint 1", &angle1, 0.0f, 360.0f)) {
            kinematicChainInstance.setJointAngles(0, angle1);
            paramsChanged = true;
        }
        if (ImGui::SliderFloat("Angle Joint 2", &angle2, 0.0f, 360.0f)) {
            kinematicChainInstance.setJointAngles(1, angle2);
            paramsChanged = true;
        }
        if (ImGui::SliderFloat("Angle Joint 3", &angle3, 0.0f, 360.0f)) {
            kinematicChainInstance.setJointAngles(2, angle3);
            paramsChanged = true;
        }
        auto [x, y] = kinematicChainInstance.forwardKinematics();
        kinematicChainInstance.setTarget({x, y});
        ImGui::Text("End Effector Position:");
        ImGui::Text("X: %.2f", x);
        ImGui::Text("Y: %.2f", y);
    } else {
        point target = kinematicChainInstance.getTarget();
        if (ImGui::SliderFloat("Target X", &target.x, -10.0f, 10.0f)) {
            kinematicChainInstance.setTarget(target);
            paramsChanged = true;
        }
        if (ImGui::SliderFloat("Target Y", &target.y, -10.0f, 10.0f)) {
            kinematicChainInstance.setTarget(target);
            paramsChanged = true;
        }
        if (!kinematicChainInstance.inverseKinematicsCCD()) {
            ImGui::TextColored(ImVec4(1, 0, 0, 1), "Target is out of bounds!");
        } else {
            const std::vector<float> &jointAngles = kinematicChainInstance.getJointAngles();
            ImGui::Text("Calculated Angles:");
            ImGui::Text("Angle Joint 1: %.2f", jointAngles[0] * (180.0f / M_PI));
            ImGui::Text("Angle Joint 2: %.2f", jointAngles[1] * (180.0f / M_PI));
            ImGui::Text("Angle Joint 3: %.2f", jointAngles[2] * (180.0f / M_PI));
        }
    }
}

void uiManager::handleLinkLengths() {
    float link1 = kinematicChainInstance.getLinkLengths()[0];
    float link2 = kinematicChainInstance.getLinkLengths()[1];
    float link3 = kinematicChainInstance.getLinkLengths()[2];

    if (ImGui::SliderFloat("Length Link 1", &link1, 1.0f, 5.0f)) {
        kinematicChainInstance.setLinkLength(0, link1);
        paramsChanged = true;
    }
    if (ImGui::SliderFloat("Length Link 2", &link2, 1.0f, 5.0f)) {
        kinematicChainInstance.setLinkLength(1, link2);
        paramsChanged = true;
    }
    if (ImGui::SliderFloat("Length Link 3", &link3, 1.0f, 5.0f)) {
        kinematicChainInstance.setLinkLength(2, link3);
        paramsChanged = true;
    }
}

void uiManager::handleReset() {
    if (ImGui::Button("Reset")) {
        kinematicChainInstance.setJointAngles(0, 0.0f);
        kinematicChainInstance.setJointAngles(1, 0.0f);
        kinematicChainInstance.setJointAngles(2, 0.0f);
        kinematicChainInstance.setLinkLength(0, 2.0f);
        kinematicChainInstance.setLinkLength(1, 2.0f);
        kinematicChainInstance.setLinkLength(2, 2.0f);
        kinematicChainInstance.setTarget({6.0f, 0.0f});
        paramsChanged = true;
    }
    ImGui::End();
}
