#include "drake/examples/KneedCompassGait/newKCG.h"
#include "drake/multibody/joints/revolute_joint.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree_construction.h"

#include <memory>
#include <string>

namespace drake{
namespace examples{
namespace newKCG{

    template <typename Scalar>
    std::unique_ptr<RigidBodyTree<Scalar>>
    getKCGTree(KCGURDF urdf_id, FloatingBaseType floating_base) {
        auto tree = std::make_unique<RigidBodyTree<Scalar>>();
        std::string urdf_path;

        switch (urdf_id){
            case k1:
                urdf_path =
                    "drake/examples/KneedCompassGait/KneedCompassGait.urdf";
                break;
            case k2:
                urdf_path =
                    "drake/examples/KneedCompassGait/KneedCompassGait.urdf";
                break;
        }

        drake::parsers::urdf::AddModelInstanceFromUrdfFile(
                FindResourceOrThrow(urdf_path), floating_base,
                nullptr, tree.get()
                );

        // terrain
        drake::multibody::AddFlatTerrainToWorld(tree.get(), 100., 10.);
        std::cout << "tree excuted" << std::endl;
        return tree;
    }

    std::unique_ptr<RigidBodyTree<double>>
    getKCGTreed(KCGURDF urdf_id, FloatingBaseType floating_base){
        return getKCGTree<double>(urdf_id, floating_base);
    }

    template <typename T>
    void setDefaultContactParams(RigidBodyPlant<T>& plant) {
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
        std::cout << "contact params excuted" << std::endl;
    }

    //explicit initiate the setDefaultContactParams template
    template void setDefaultContactParams<double>(RigidBodyPlant<double>&);

    VectorX<double> KCGFixedPointState(FloatingBaseType floating_base) {
        VectorX<double> ret(kKCGStates);
        ret << 0, 0, 1, 0, 0, 0,
        0, 1.57, -0.5,
        0, 0, 0, 0, 0, 0,
        0, 0, 0;
        std::cout << "point state excuted" << std::endl;
        switch (floating_base) {
            case FloatingBaseType::kRollPitchYaw:
                return ret;
            default:
                throw std::invalid_argument(
                        "only RPY supported.\n");
        }
    }

    VectorX<double> KCGFixedPointTorque(){
        VectorX<double> ff_torque(kKCGActuators);
        ff_torque << 0, 0, 0;
        return ff_torque;
    }

    FloatingBaseState BaseStateFromFullState(const KCGState &full_state) {
        FloatingBaseState base_state;
        base_state << full_state.segment(0, 6), full_state.segment(9, 6);
        return base_state;
    }

}  //namespace newKCG
}  // namespace examples
}  // namespace drake

