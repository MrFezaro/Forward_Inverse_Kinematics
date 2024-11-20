#include "chainGeometry.hpp"

using namespace threepp;

chainGeometry::chainGeometry(Scene &scene, kinematicChain &chain) : scene(scene), chain(chain) {}

std::shared_ptr<Mesh> chainGeometry::createJoint(const BoxGeometry::Params &params, const Color &color) {
    const auto geometry = BoxGeometry::create(params);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    return Mesh::create(geometry, material);
}

std::shared_ptr<Mesh> chainGeometry::createLink(const float length, const float radius, const Color &color) {
    const auto geometry = CylinderGeometry::create(radius, radius, length, 32);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    auto link = Mesh::create(geometry, material);
    link->rotation.x = math::degToRad(90);// Align along Y axis
    return link;
}

std::shared_ptr<Mesh> chainGeometry::createSphere(const float radius, const Color &color) {
    const auto geometry = SphereGeometry::create(radius, 32, 32);
    const auto material = MeshBasicMaterial::create({{"color", color}});
    return Mesh::create(geometry, material);
}

void chainGeometry::createKinematicChain() {
    // Create base
    const BoxGeometry::Params baseParams{1.0f, 1.0f, 1.0f};
    const auto base = createJoint(baseParams, Color::white);
    scene.add(base);
    base->position.y = 2.0f;
    base->rotation.y = math::degToRad(180);// Rotate base to align with world

    // Create joints
    const BoxGeometry::Params jointParams{0.5f, 0.5f, 0.5f};
    joint1 = createJoint(jointParams, Color::green);
    joint2 = createJoint(jointParams, Color::red);
    joint3 = createJoint(jointParams, Color::blue);

    // Create links (cylinders)
    link1 = createLink(1.0f, 0.125f, Color::yellow);
    link2 = createLink(1.0f, 0.125f, Color::yellow);
    link3 = createLink(1.0f, 0.125f, Color::yellow);

    // Attach joints hierarchically
    base->add(joint1);
    joint1->add(link1);                    // Attach link1 between base and joint1
    joint1->position.y = -1.0f;            // Position joint1 below base
    link1->position.y = -0.5f;             // Position link1 between base and joint1
    link1->rotation.z = math::degToRad(90);// Rotate link1 to align with joint1
    link1->rotation.y = math::degToRad(90);// Rotate link1 to align with joint1

    joint1->add(joint2);
    joint2->add(link2);                    // Attach link2 between joint1 and joint2
    joint2->position.y = -1.0f;            // Position joint2 between joint1 and joint3
    link2->position.y = -0.5f;             // Position link2 between joint1 and joint2
    link2->rotation.z = math::degToRad(90);// Rotate link2 to align with joint2
    link2->rotation.y = math::degToRad(90);// Rotate link2 to align with joint2

    joint2->add(joint3);
    joint3->add(link3);                    // Attach link3 between joint2 and joint3
    joint3->position.y = -1.0f;            // Position joint3 between joint2 and sphere
    link3->position.y = -0.5f;             // Position link3 between joint2 and joint3
    link3->rotation.z = math::degToRad(90);// Rotate link3 to align with joint3
    link3->rotation.y = math::degToRad(90);// Rotate link3 to align with joint3

    // Create and attach a sphere to the end of joint3
    sphere = createSphere(0.3f, Color::white);// Sphere with radius 0.3
    joint3->add(sphere);
    sphere->position.y = -1.0f;// Position the sphere at the end of joint3

    // Apply initial rotations and lengths to the joints and links
    std::vector<float> jointAngles = chain.getJointAngles();
    std::vector<float> linkLengths = chain.getLinkLengths();

    joint1->rotation.z = jointAngles[0];
    joint2->rotation.z = jointAngles[1];
    joint3->rotation.z = jointAngles[2];
    link1->scale.y = linkLengths[0];
    link2->scale.y = linkLengths[1];
    link3->scale.y = linkLengths[2];
    joint1->position.y = -1.0f;// Fix joint1 position relative to the base
    link1->position.y = -linkLengths[0] / 2.0f;
    joint2->position.y = -linkLengths[0];      // Position joint2 based on lengthLink1
    link2->position.y = -linkLengths[1] / 2.0f;// Position link2 between joint2 and joint3
    joint3->position.y = -linkLengths[1];      // Position joint3 based on lengthLink2
    link3->position.y = -linkLengths[2] / 2.0f;// Position link3 between joint3 and sphere
    sphere->position.y = -linkLengths[2];      // Position the sphere at the end of link3
}