#ifndef SCENEMANAGER_HPP
#define SCENEMANAGER_HPP

#include <threepp/threepp.hpp>

/**
 * @class SceneManager
 * @brief Manages the 3D scene, including the canvas, renderer, camera, and controls.
 *
 * This class is responsible for setting up and managing the 3D scene, including the canvas for rendering,
 * the renderer, the scene graph, the camera, and the orbit controls for user interaction.
 */
class SceneManager {
public:
    SceneManager();

    threepp::Canvas canvas;
    threepp::GLRenderer renderer;
    threepp::Scene scene;
    threepp::PerspectiveCamera camera;
    threepp::OrbitControls controls;
};

#endif// SCENEMANAGER_HPP