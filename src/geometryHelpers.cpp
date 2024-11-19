#include "geometryHelpers.hpp"
#include "kinematicChain.hpp"

std::shared_ptr<Mesh> createJoint(const BoxGeometry::Params &params, const Color &color) {
    const auto geometry = BoxGeometry::create(params);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    return Mesh::create(geometry, material);
}

std::shared_ptr<Mesh> createLink(const float length, const float radius, const Color &color) {
    const auto geometry = CylinderGeometry::create(radius, radius, length, 32);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    auto link = Mesh::create(geometry, material);
    link->rotation.x = math::degToRad(90);// Align along Y axis
    return link;
}

std::shared_ptr<Mesh> createSphere(const float radius, const Color &color) {
    const auto geometry = SphereGeometry::create(radius, 32, 32);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    return Mesh::create(geometry, material);
}

void setLinkLength(kinematicChain &chain, const int linkIndex, const float newLength, const std::shared_ptr<Mesh> &link, const std::shared_ptr<Mesh> &joint) {
    if (linkIndex >= 0 && linkIndex < chain.getLinkLengths().size()) {
        chain.setLinkLength(linkIndex, newLength);
        link->position.y = -newLength / 2.0f;
        joint->position.y = -newLength;
    }
}
