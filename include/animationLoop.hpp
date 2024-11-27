#ifndef ANIMATIONLOOP_HPP
#define ANIMATIONLOOP_HPP

#include "chainGeometry.hpp"
#include "chainKinematics.hpp"
#include "sceneManager.hpp"
#include "uiManager.hpp"

void runAnimationLoop(sceneManager &scene, chainKinematics &kinematicChain, uiManager &ui, chainGeometry &chainGeometry);

#endif// ANIMATIONLOOP_HPP