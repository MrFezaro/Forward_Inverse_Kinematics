#ifndef SCENEMANAGER_HPP
#define SCENEMANAGER_HPP

#include "threepp/threepp.hpp"

class sceneManager {
public:
    sceneManager();

    threepp::Canvas canvas;
    threepp::GLRenderer renderer;
    threepp::Scene scene;
    threepp::PerspectiveCamera camera;
    threepp::OrbitControls controls;
};

#endif// SCENEMANAGER_HPP