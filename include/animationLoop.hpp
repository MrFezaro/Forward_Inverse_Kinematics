#ifndef ANIMATIONLOOP_HPP
#define ANIMATIONLOOP_HPP

#include "ChainGeometry.hpp"
#include "ChainKinematics.hpp"
#include "SceneManager.hpp"
#include "UiManager.hpp"

/**
 * @brief Runs the main animation loop.
 *
 * This function is responsible for updating the scene, handling user input, and performing kinematic calculations
 * in each frame of the animation loop.
 *
 * @param scene Reference to the SceneManager instance managing the 3D scene.
 * @param chainKinematics Reference to the ChainKinematics instance handling the kinematic calculations.
 * @param ui Reference to the UiManager instance managing the user interface.
 * @param chainGeometry Reference to the ChainGeometry instance managing the geometry of the kinematic chain.
 */

void runAnimationLoop(SceneManager &scene, const ChainKinematics &chainKinematics, UiManager &ui, const ChainGeometry &chainGeometry);

#endif// ANIMATIONLOOP_HPP