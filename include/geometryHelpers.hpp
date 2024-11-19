#ifndef GEOMETRYHELPERS_HPP
#define GEOMETRYHELPERS_HPP

#include "threepp/threepp.hpp"
#include "kinematicChain.hpp"

class GeometryHelpers {
public:
  GeometryHelpers(threepp::Scene &scene, kinematicChain &chain);

  void createKinematicChain();
  std::shared_ptr<threepp::Object3D> joint1;
  std::shared_ptr<threepp::Object3D> joint2;
  std::shared_ptr<threepp::Object3D> joint3;

private:
  threepp::Scene &scene;
  kinematicChain &chain;

  std::shared_ptr<threepp::Mesh> createJoint(const threepp::BoxGeometry::Params &params, const threepp::Color &color);
  std::shared_ptr<threepp::Mesh> createLink(float length, float radius, const threepp::Color &color);
  std::shared_ptr<threepp::Mesh> createSphere(float radius, const threepp::Color &color);
};

#endif // GEOMETRYHELPERS_HPP