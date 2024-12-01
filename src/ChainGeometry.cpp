#include "chainGeometry.hpp"

using namespace threepp;

ChainGeometry::ChainGeometry(Scene &scene, ChainKinematics &chainKinematics) : scene_(scene), chain_(chainKinematics) {}

std::shared_ptr<Mesh> ChainGeometry::createJoint(const BoxGeometry::Params &params, const Color &color) {
    const auto geometry = BoxGeometry::create(params);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    return Mesh::create(geometry, material);
}

std::shared_ptr<Mesh> ChainGeometry::createLink(const float length, const float radius, const Color &color) {
    const auto geometry = CylinderGeometry::create(radius, radius, length, 32);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    auto link = Mesh::create(geometry, material);
    link->rotation.x = math::degToRad(90); //Align along Y axis
    return link;
}

std::shared_ptr<Mesh> ChainGeometry::createSphere(const float radius, const Color &color) {
    const auto geometry = SphereGeometry::create(radius, 32, 32);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    return Mesh::create(geometry, material);
}

void ChainGeometry::createBase() {
    const BoxGeometry::Params baseParams{1.0f, 1.0f, 1.0f};
    base = createJoint(baseParams, Color::white);
    scene_.add(base);
    base->position.y = 2.0f;
    base->rotation.y = math::degToRad(180); //Rotate base to align with world
}

void ChainGeometry::createJointsAndLinks() {
    const BoxGeometry::Params jointParams{0.5f, 0.5f, 0.5f};
    const Color jointColors[] = {Color::green, Color::red, Color::blue};
    const Color linkColor = Color::yellow;

    for (int i = 0; i < 3; ++i) {
        joints[i] = createJoint(jointParams, jointColors[i]);
        links[i] = createLink(1.0f, 0.125f, linkColor);
        links[i]->rotation.z = math::degToRad(90);
        links[i]->rotation.y = math::degToRad(90);
    }

    base->add(joints[0]);
    joints[0]->add(links[0]);
    joints[0]->position.y = -1.0f;
    links[0]->position.y = -0.5f;

    for (int i = 1; i < 3; ++i) {
        joints[i - 1]->add(joints[i]);
        joints[i]->add(links[i]);
        joints[i]->position.y = -1.0f;
        links[i]->position.y = -0.5f;
    }
}

void ChainGeometry::createSphereAtEnd() {
    sphere = createSphere(0.3f, Color::white);
    joints[2]->add(sphere);
    sphere->position.y = -1.0f;
}

void ChainGeometry::applyInitialTransformations() const {
    for (int i = 0; i < 3; ++i) {
        joints[i]->rotation.z = chain_.getJointAngle(i);
        links[i]->scale.y = chain_.getLinkLength(i);
    }

    joints[0]->position.y = -1.0f;
    links[0]->position.y = -chain_.getLinkLength(0) / 2.0f;
    joints[1]->position.y = -chain_.getLinkLength(0);
    links[1]->position.y = -chain_.getLinkLength(1) / 2.0f;
    joints[2]->position.y = -chain_.getLinkLength(1);
    links[2]->position.y = -chain_.getLinkLength(2) / 2.0f;
    sphere->position.y = -chain_.getLinkLength(2);
}

void ChainGeometry::create() {
    createBase();
    createJointsAndLinks();
    createSphereAtEnd();
    applyInitialTransformations();
}