#ifndef GEOMETRYHELPERS_HPP
#define GEOMETRYHELPERS_HPP

#include "kinematicChain.hpp"
#include "threepp/threepp.hpp"

class chainGeometry {
public:
    chainGeometry(threepp::Scene &scene, kinematicChain &chain);

    void createKinematicChain();

    std::shared_ptr<threepp::Object3D> joint1;
    std::shared_ptr<threepp::Object3D> joint2;
    std::shared_ptr<threepp::Object3D> joint3;
    std::shared_ptr<threepp::Object3D> link1;
    std::shared_ptr<threepp::Object3D> link2;
    std::shared_ptr<threepp::Object3D> link3;
    std::shared_ptr<threepp::Object3D> sphere;

private:
    threepp::Scene &scene;
    kinematicChain &chain;

    static std::shared_ptr<threepp::Mesh> createJoint(const threepp::BoxGeometry::Params &params, const threepp::Color &color);
    static std::shared_ptr<threepp::Mesh> createLink(float length, float radius, const threepp::Color &color);
    static std::shared_ptr<threepp::Mesh> createSphere(float radius, const threepp::Color &color);
};

#endif// GEOMETRYHELPERS_HPP