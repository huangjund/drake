//
// Created by Junda on 8/7/19.
//
#pragma once

#include <memory>
#include <string>

#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/common/find_resourse.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/scalar_conversion_traits.h"

namespace drake {
namespace examples {
namespace compass_gait {
    // kFilePath like drake/examples/compass_gait/simulator1.obj
    void AddTerrainToWorld(RigidBodyTree<double>* tree, std::string kFilePath,
                           double x_bias = 0, double y_bias = 0, double z_bias = 0);
}
}  // namespace examples
}  // namespace drake

