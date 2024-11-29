#ifndef GEOMETRYHELPERS_HPP
#define GEOMETRYHELPERS_HPP

#include "ChainKinematics.hpp"
#include <threepp/threepp.hpp>

class ChainGeometry {
public:
    ChainGeometry(threepp::Scene &scene, ChainKinematics &chainKinematics);
    void create();

    std::shared_ptr<threepp::Object3D> base;
    std::shared_ptr<threepp::Object3D> joints[3];
    std::shared_ptr<threepp::Object3D> links[3];
    std::shared_ptr<threepp::Object3D> sphere;

private:
    threepp::Scene &scene_;
    ChainKinematics &chain_;

    void createBase();
    void createJointsAndLinks();
    void createSphereAtEnd();
    void applyInitialTransformations();

    static std::shared_ptr<threepp::Mesh> createJoint(const threepp::BoxGeometry::Params &params, const threepp::Color &color);
    static std::shared_ptr<threepp::Mesh> createLink(float length, float radius, const threepp::Color &color);
    static std::shared_ptr<threepp::Mesh> createSphere(float radius, const threepp::Color &color);
};

#endif // GEOMETRYHELPERS_HPP