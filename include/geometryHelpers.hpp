#ifndef GEOMETRYHELPERS_HPP
#define GEOMETRYHELPERS_HPP

#include "threepp/threepp.hpp"

using namespace threepp;

std::shared_ptr<Mesh> createJoint(const BoxGeometry::Params &params, const Color &color);
std::shared_ptr<Mesh> createLink(float length, float radius, const Color &color);
std::shared_ptr<Mesh> createSphere(float radius, const Color &color);
void setLinkLength(float &linkLength, float newLength, const std::shared_ptr<Mesh> &link, const std::shared_ptr<Mesh> &joint);

#endif // GEOMETRYHELPERS_HPP