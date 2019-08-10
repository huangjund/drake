//
// Created by Junda on 8/9/19.
//
#include "drake/examples/KneedCompassGait/KCG_common.h"

namespace drake {
namespace examples {
namespace kkk {
    template <typename Scalar>
    std::unique_ptr<RigidBodyTree<Scalar>>
    getKCGTree() {
        auto tree = std::make_unique<RigidBodyTree<Scalar>>();
        std::string urdf_path;
        urdf_path =
                "drake/examples/KneedCompassGait/KneedCompassGait.urdf";

        drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                FindResourceOrThrow(urdf_path),
                multibody::joints::kRollPitchYaw, tree.get());

        // terrain
        drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
        return tree;
    }

    std::unique_ptr<RigidBodyTree<double>>
    getKCGTreed(){
        return getKCGTree<double>();
    }

    template <typename T>
    void setDefaultContactParams(systems::RigidBodyPlant<T>& plant) {
        const double kYoung = 1e8; // Pa
        const double kDissipation = 5.0; // s/m
        const double kStaticFriction = 1;
        const double kDynamicFriction = 1;

        drake::systems::CompliantMaterial default_material;
        default_material.set_youngs_modulus(kYoung)
                .set_dissipation(kDissipation)
                .set_friction(kStaticFriction, kDynamicFriction);
        plant.set_default_compliant_material(default_material);

        const double kStictionSlipTolerance = 0.01; // m/s
        const double kContactRadius = 2e-3; //m
        drake::systems::CompliantContactModelParameters model_parameters;
        model_parameters.characteristic_radius = kContactRadius;
        model_parameters.v_stiction_tolerance = kStictionSlipTolerance;
        plant.set_contact_model_parameters(model_parameters);
    }

    //explicit initiate the setDefaultContactParams template
    template void setDefaultContactParams<double>(systems::RigidBodyPlant<double>&);

    VectorX<double> KCGFixedPointState() {
        VectorX<double> ret(18);
        ret << 0, 0, 1.0, 0, 0, 0,
                0, -0.5, -0.1,
                0, 0, 0, 0, 0, 0,
                0, 0, 0;
        return ret;
    }

    VectorX<double> KCGFixedPointTorque(){
        VectorX<double> ff_torque(3);
        ff_torque << 0, 0, 0;
        return ff_torque;
    }

}  // namespace kkk
}  // namespace examples
}  // namespace drake
