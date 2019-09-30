//
// Created by Junda Huang on 9/29/19.
//

#pragma once

#include <memory>
#include <string>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace examples {
namespace KUKA {

template <typename Scalar>
std::unique_ptr<RigidBodyTree<Scalar>>
getKUKATree();

std::unique_ptr<RigidBodyTree<double>>
getKUKATreed();

//std::unique_ptr<multibody::MultibodyPlant<double>>
//MakeKukaPlant(bool finalize,
//                 geometry::SceneGraph<double>* scene_graph = nullptr);

}
}
}
