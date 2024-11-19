#ifndef ANIMATIONLOOP_HPP
#define ANIMATIONLOOP_HPP

#include "geometryHelpers.hpp"
#include "kinematicChain.hpp"
#include "sceneManager.hpp"
#include "uiManager.hpp"
#include "threepp/threepp.hpp"

void runAnimationLoop(sceneManager &scene, kinematicChain &chain, uiManager &ui, GeometryHelpers &geometryHelpers);

#endif // ANIMATIONLOOP_HPP