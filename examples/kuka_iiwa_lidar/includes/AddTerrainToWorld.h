//
// Created by Junda Huang on 9/29/19.
//

#pragma once

#include <string>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/common/find_resource.h"

namespace drake {
namespace examples {
namespace Terrain {
    void AddBasicTerrain(RigidBodyTree<double>* tree, double width, double length, double depth);
} // namespace Terrain
} // namespace examples
} // namespace drake
