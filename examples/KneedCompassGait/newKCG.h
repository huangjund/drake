#pragma once

#include "drake/multibody/rigid_body_plant/rigid_body_plant.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/basic_vector.h"
#include "drake/systems/framework/witness_function.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/math/rotation_matrix.h"
#include "drake/common/find_resource.h"
#include "drake/multibody/joints/floating_base_types.h"

#include <memory>

namespace drake{
namespace examples{
namespace newKCG{
    using drake::systems::RigidBodyPlant;
    using drake::multibody::joints::FloatingBaseType;

    constexpr int kKCGPositions = 9;
    constexpr int kKCGVelocities = 9;
    constexpr int kKCGStates = 18;
    constexpr int kKCGJoints = 3;
    constexpr int kKCGActuators = 3;

    typedef Eigen::Matrix<double, kKCGPositions, 1> KCGPosition;
    typedef Eigen::Matrix<double, kKCGVelocities, 1> KCGVelocity;
    typedef Eigen::Matrix<double, kKCGStates, 1> KCGState;
    typedef Eigen::Matrix<double, kKCGJoints, 1> KCGJointVector;

    typedef Eigen::Matrix<double, 12, 1> FloatingBaseState;
    typedef Eigen::Matrix<double, 12, 12> FloatingBaseCov;

    enum KCGURDF{
        k1 = 0,
        k2 = 1
    };

    VectorX<double> KCGFixedPointState(FloatingBaseType floating_base =
            FloatingBaseType::kRollPitchYaw);

    VectorX<double> KCGFixedPointTorque();

    FloatingBaseState BaseStateFromFullState(const KCGState&);

    std::unique_ptr<RigidBodyTree<double>>
    getKCGTreed(KCGURDF urdf_id = k1,
            FloatingBaseType floating_base = FloatingBaseType::kRollPitchYaw);

    template <typename Scalar>
    std::unique_ptr<RigidBodyTree<Scalar>>
    getKCGTree(KCGURDF urdf_id = k1,
            FloatingBaseType floating_base = FloatingBaseType::kRollPitchYaw);

    template <typename T>
    void setDefaultContactParams(RigidBodyPlant<T>& plant);

}  // namespace newKCG
}  // namespace examples
}  // namespace drake
