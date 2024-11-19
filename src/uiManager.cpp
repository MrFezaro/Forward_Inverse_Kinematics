#include "uiManager.hpp"
#include "geometryHelpers.hpp"
#include "kinematicChain.hpp"

uiManager::uiManager(sceneManager &scene, kinematicChain &kinematicChainInstance)
    : scene(scene), kinematicChainInstance(kinematicChainInstance), ui(scene.canvas.windowPtr(), [&] {
          ImGui::SetNextWindowPos({0, 0}, 0, {0, 0});
          ImGui::SetNextWindowSize({320, 0}, 0);
          ImGui::Begin("Joint Controls");

          const bool isHovered = ImGui::IsWindowHovered(ImGuiHoveredFlags_AnyWindow);
          const bool isInteracting = ImGui::IsAnyItemActive();

          if (ImGui::Button(isForwardKinematics ? "Switch to Inverse Kinematics" : "Switch to Forward Kinematics")) {
              isForwardKinematics = !isForwardKinematics;
              if (!isForwardKinematics) {
                  kinematicChainInstance.target = endEffectorPosition;
                  paramsChanged = true;
              }
          }

          if (isForwardKinematics) {
              ImGui::SliderFloat("Angle Joint 1", &angleJoint1, 0.0f, 360.0f);
              paramsChanged = paramsChanged || ImGui::IsItemEdited();
              ImGui::SliderFloat("Angle Joint 2", &angleJoint2, 0.0f, 360.0f);
              paramsChanged = paramsChanged || ImGui::IsItemEdited();
              ImGui::SliderFloat("Angle Joint 3", &angleJoint3, 0.0f, 360.0f);
              paramsChanged = paramsChanged || ImGui::IsItemEdited();

              ImGui::Text("End Effector Position:");
              ImGui::Text("X: %.2f", endEffectorPosition.y * -1);
              ImGui::Text("Y: %.2f", endEffectorPosition.x * -1);
          } else {
              auto targetX = target.x;
              auto targetY = target.y;
              if (ImGui::SliderFloat("Target X", &targetX, -10.0f, 10.0f)) {
                  target.x = targetX;
                  paramsChanged = true;
              }
              if (ImGui::SliderFloat("Target Y", &targetY, -10.0f, 10.0f)) {
                  target.y = targetY;
                  paramsChanged = true;
              }

              ImGui::Text("Calculated Angles:");
              ImGui::Text("Angle Joint 1: %.2f", angleJoint1);
              ImGui::Text("Angle Joint 2: %.2f", angleJoint2);
              ImGui::Text("Angle Joint 3: %.2f", angleJoint3);

              if (!kinematicChainInstance.inverseKinematicsCCD(target, angleJoint1, angleJoint2, angleJoint3)) {
                  ImGui::TextColored(ImVec4(1, 0, 0, 1), "Target is out of bounds!");
              }
          }

          if (ImGui::SliderFloat("Length Link 1", &lengthLink1, 1.0f, 5.0f)) {
              kinematicChainInstance.updateLinkLength(0, lengthLink1);
              paramsChanged = true;
          }
          if (ImGui::SliderFloat("Length Link 2", &lengthLink2, 1.0f, 5.0f)) {
              kinematicChainInstance.updateLinkLength(1, lengthLink2);
              paramsChanged = true;
          }
          if (ImGui::SliderFloat("Length Link 3", &lengthLink3, 1.0f, 5.0f)) {
              kinematicChainInstance.updateLinkLength(2, lengthLink3);
              setLinkLength(2, lengthLink3, link3, joint3);
              paramsChanged = true;
          }

          if (ImGui::Button("Reset")) {
              angleJoint1 = 0.0f;
              angleJoint2 = 0.0f;
              angleJoint3 = 0.0f;
              target = {6.0f, 0.0f};
              kinematicChainInstance.updateLinkLength(0, 2.0f);
              kinematicChainInstance.updateLinkLength(1, 2.0f);
              kinematicChainInstance.updateLinkLength(2, 2.0f);
              paramsChanged = true;
          }

          ImGui::End();

          scene.controls.enabled = !(isHovered || isInteracting);
      }) {}

void uiManager::render() {
    ui.render();
}