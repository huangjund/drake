//
// Created by Junda Huang on 10/30/19.
//


#pragma once

#include <memory>
#include <string>
#include <vector>

#include "drake/math/rotation_matrix.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/multibody/rigid_body_tree_construction.h"
#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/primitives/constant_vector_source.h"

namespace drake {
    namespace examples {
        namespace kkk {

            template <typename Scalar>
            std::unique_ptr<RigidBodyTree<Scalar>>
            getKCGTree();

            std::unique_ptr<RigidBodyTree<double>>
            getKCGTreed();

            template <typename T>
            void setDefaultContactParams(systems::RigidBodyPlant<T>& plant);

            VectorX<double> KCGFixedPointState();

            VectorX<double> KCGFixedPointTorque();
        }
    }
}

