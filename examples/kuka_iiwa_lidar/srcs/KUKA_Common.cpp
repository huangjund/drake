//
// Created by Junda Huang on 9/29/19.
//
#include "drake/examples/kuka_iiwa_lidar/includes/KUKA_Common.h"
#include "drake/examples/kuka_iiwa_lidar/includes/AddTerrainToWorld.h"

namespace drake {
namespace examples {
namespace KUKA {

template <typename Scalar>
std::unique_ptr<RigidBodyTree<Scalar>>
getKUKATree() {
    auto tree = std::make_unique<RigidBodyTree<Scalar>>();
    std::string filepath;
    filepath = "drake/manipulation/models/iiwa_description/urdf/iiwa14_polytope_collision.urdf";
    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
            FindResourceOrThrow(filepath), multibody::joints::kFixed, tree.get());

    Terrain::AddBasicTerrain(tree.get(), 100, 100, 2);
    return tree;
}

std::unique_ptr<RigidBodyTree<double>>
getKUKATreed() {
    return getKUKATree<double>();
}

//std::unique_ptr<multibody::MultibodyPlant<double>>
//MakeKukaPlant(bool finalize,
//              geometry::SceneGraph<double>* scene_graph = nullptr) {
//    auto plant = std::make_unique<multibody::MultibodyPlant<double>>();
//
//}

}
}
}

