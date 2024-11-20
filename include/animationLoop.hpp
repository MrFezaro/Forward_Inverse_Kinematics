#ifndef ANIMATIONLOOP_HPP
#define ANIMATIONLOOP_HPP

#include "chainGeometry.hpp"
#include "kinematicChain.hpp"
#include "sceneManager.hpp"
#include "uiManager.hpp"

void runAnimationLoop(sceneManager &scene, kinematicChain &kinematicChain, uiManager &ui, chainGeometry &chainGeometry);

#endif// ANIMATIONLOOP_HPP