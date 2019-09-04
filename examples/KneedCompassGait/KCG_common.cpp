//
// Created by Junda on 8/9/19.
//
#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/examples/KneedCompassGait/system.h"


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
        const double kStaticFriction = 10;
        const double kDynamicFriction = 10;

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
        VectorX<double> ret(22);
        // left leg stance
        ret <<  -1.00, 0, 0.8707, 0, 0.7268, 0,
                0, -0.6283, -0.8361, 0, -0.8424,
                -1, 0, 0, 0, 0, 0,
                0, 0, -0.3, 0, 0;
//        ret <<  -0.9723, 0, 0.8707, 0, 0.0985, 0,
//                0, 0.6283, -0.8324, 0, -0.8361,
//                -1, 0, 0, 0, 0, 0,
//                0, 0, 0, 0, 0;
// qp test states
//        ret << -1.19137856, 0, 0.83038519, 0, -0.02372, 0,
//                0, 0.69030429, -0.821146, 0, -1.19141,
//                -1.73838435, 0, -0.5438, 0, -2.56655479, 0,
//                0, 1.4851, 1.18572348, 0, -1.177;

//        ret << -1.07, 0, 0.89416862, 0, 0, 0,
//                0, 0.58612748, -0.68462465, 0, -0.84274030,
//                -1.1, 0, -0.16243628, 0, -1.7165947, 0,
//                0, 1.53875353, 1.23234924, 0, -1.94284887;
        return ret;
    }

    VectorX<double> KCGFixedPointTorque(){
        VectorX<double> ff_torque(5);
        ff_torque << 0, 0, 0, 0, 0;
        return ff_torque;
    }

}  // namespace kkk
}  // namespace examples
}  // namespace drake
