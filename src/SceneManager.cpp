#include "SceneManager.hpp"

SceneManager::SceneManager()
    : canvas("Forward and inverse kinematic test", {{"aa", 8}}),
      renderer(canvas.size()),
      camera(60, canvas.aspect(), 0.1f, 100),
      controls(camera, canvas) {
    scene.background = threepp::Color::black;
    camera.position.z = 10;
}
