#ifndef ANIMATIONLOOP_HPP
#define ANIMATIONLOOP_HPP

#include "ChainGeometry.hpp"
#include "ChainKinematics.hpp"
#include "SceneManager.hpp"
#include "UiManager.hpp"

void runAnimationLoop(SceneManager &scene, ChainKinematics &kinematicChain, UiManager &ui, ChainGeometry &chainGeometry);

#endif// ANIMATIONLOOP_HPP