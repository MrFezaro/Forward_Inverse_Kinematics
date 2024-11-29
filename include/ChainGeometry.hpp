#ifndef GEOMETRYHELPERS_HPP
#define GEOMETRYHELPERS_HPP

#include "ChainKinematics.hpp"
#include <threepp/threepp.hpp>

/**
 * @class ChainGeometry
 * @brief Manages the geometry of the kinematic chain in the 3D scene.
 *
 * This class is responsible for creating and managing the 3D objects that represent the joints and links
 * of the kinematic chain in the scene.
 */

class ChainGeometry {
public:
    ChainGeometry(threepp::Scene &scene, ChainKinematics &chainKinematics);

    /**
     * @brief Creates the geometry for the kinematic chain.
     *
     * This function initializes the 3D objects representing the joints and links of the kinematic chain.
     */
    void create();

    std::shared_ptr<threepp::Object3D> joint1;
    std::shared_ptr<threepp::Object3D> joint2;
    std::shared_ptr<threepp::Object3D> joint3;
    std::shared_ptr<threepp::Object3D> link1;
    std::shared_ptr<threepp::Object3D> link2;
    std::shared_ptr<threepp::Object3D> link3;
    std::shared_ptr<threepp::Object3D> sphere;

private:
    threepp::Scene &scene_;
    ChainKinematics &chain_;

    static std::shared_ptr<threepp::Mesh> createJoint(const threepp::BoxGeometry::Params &params, const threepp::Color &color);
    static std::shared_ptr<threepp::Mesh> createLink(float length, float radius, const threepp::Color &color);
    static std::shared_ptr<threepp::Mesh> createSphere(float radius, const threepp::Color &color);
};

#endif// GEOMETRYHELPERS_HPP