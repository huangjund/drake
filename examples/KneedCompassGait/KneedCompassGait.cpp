#include "drake/examples/KneedCompassGait/KneedCompassGait.h"

#include <algorithm>

#include "drake/common/default_scalars.h"

namespace drake {
namespace examples {
namespace KneedCompassGait {

template <typename T>
KneedCompassGait<T>::KneedCompassGait(
        RigidBodyTree<T> *tree) : systems::LeafSystem<T>(
        systems::SystemTypeTag<
                examples::KneedCompassGait::KneedCompassGait>{}) {
    // Four Continuous State
    this->DeclareContinuousState(KneedcompassgaitContinuousstate<T>(), 4, 4, 0);

    // Discrete state for left toe distance along the plane.
    this->DeclareDiscreteState(1);

    // Abstract state for indicating the left leg is the stance leg. This
    // is only used for the visualization output.
    bool left_stance = true;
    this->DeclareAbstractState(AbstractValue::Make(left_stance));

    // The minimal state of the system; ###this version will be deprecated
    this->DeclareVectorOutputPort(KneedcompassgaitContinuousstate<T>(),
                                &KneedCompassGait::MinimalStateOut);

    // The floating-base state of the system(useful for visual)
    this->DeclareVectorOutputPort(systems::BasicVector<T>(18),
                                &KneedCompassGait::FloatingBaseStateOut);

    // Natural property
    this->DeclareNumericParameter(KneedcompassgaitParams<T>());

    rigidtree = tree;

    // Create the witness function
    foot_collision_ = this->MakeWitnessFunction(
            "foot collision",
            systems::WitnessFunctionDirection::kPositiveThenNonPositive,
            &KneedCompassGait::FootCollision,
            &KneedCompassGait::CollisionDynamics
            );
}

//    template <typename T>
//    void KneedCompassGait<T>::SetKneedCWParams(
//            drake::systems::Parameters<T> *parameters) const {
//        auto* p = dynamic_cast<KneedcompassgaitParams<T>*>(parameters);
//        DRAKE_ASSERT(p != nullptr);
//        p->length_leg(0.5);
//    }

//  template <typename T>
//  T KneedCompassGait<T>::DoCalcKineticEnergy(
//        const drake::systems::Context<T> &context) const {
//
//    const KneedcompassgaitContinuousstate<T>& cg_state =
//            get_continuous_state(context);
//    const KneedcompassgaitParams<T>& params = get_parameters(context);
//
//    const T m = params.mass_leg();
//    const T mh = params.mass_hip();
//    const T l = params.length_leg();
//    const T a1 = params.length_leg() - params.length_hip2upper_leg();
//    const T b1 = params.length_hip2upper_leg();
//    const T a2 = params.length_leg() - params.length_knee2lower_leg();
//    const T b2 = params.length_knee2lower_leg();
//    const T wls = cg_state.angledot_stance_knee();
//    const T wrs = cg_state.angledot_swing_knee();
//    const T wlt = cg_state.angledot_hip();
//    const T wrt = cg_state.angledot_thigh();
//
//    // Sum 1/2 m*v^2 for each of the point masses.
//    return .5 * (mh * l * l + m * a1 * a1) * vst * vst +
//           .5 * m * (l * l * vst * vst + b1 * b1 * vsw * vsw) -
//           m * l * b1 * vst * vsw * cos(cg_state.swing() - cg_state.stance());
//
//  }

//  template <typename T>
//  T KneedCompassGait<T>::DoCalcPotentialEnergy(
//        const systems::Context<T>& context) const {
//    const KneedcompassgaitContinuousstate<T>& cs =
//            get_continuous_state(context);
//    const KneedcompassgaitParams<T>& p = get_parameters(context);
//
//    const T m = p.mass_leg();
//    const T mh = p.mass_hip();
//    const T l = p.length_leg();
//    const T a1 = p.length_leg() - p.length_hip2upper_leg();
//    const T b1 = p.length_hip2upper_leg();
//    const T a2 = p.length_leg() - p.length_knee2lower_leg();
//    const T b2 = p.length_knee2lower_leg();
//    const T g = p.gravity();
//    const T theta1 = cs.angle_stance_knee() + cs.angle_hip();
//    const T theta2 = cs.angle_hip();
//    const T theta3 = cs.angle_thigh() + cs.angle_hip();
//    const T theta4 = theta3 + cs.angle_stance_knee();
//
//    using std::cos;
//    const T E = m*g*((a2+3*l)*cos(theta1) + (a1+2*l)*cos(theta2)
//            - (l+b1)*cos(theta3) - b2*cos(theta4))+mh*g*l*(cos(theta1)+cos(theta2));
//    return E;
//  }

// wittness trigger function
template <typename T>
T KneedCompassGait<T>::FootCollision(
          const systems::Context<T>& context) const {
    const KneedcompassgaitContinuousstate<T>& cs =
            get_continuous_state(context);
    const KneedcompassgaitParams<T>& p = get_parameters(context);

    using std::cos;
    // Compute angle through geometric constraints
    const T theta1 = cs.angle_thigh() + cs.angle_hip();
    const T theta2 = theta1 + cs.angle_swing_knee();
    const T theta3 = cs.angle_thigh() + cs.angle_stance_knee();
    const T swing_height = p.length_leg()*(cos(theta1) + cos(theta2));
    const T stance_height = p.length_leg()*(cos(theta3) + cos(cs.angle_hip()));

    const T collision = stance_height - swing_height;

    return collision;
}

 // wittness callback function
//template <typename T>
//void KneedCompassGait<T>::CollisionDynamics(
//        const systems::Context<T>& context,
//        const systems::UnrestrictedUpdateEvent<T>& ,
//        systems::State<T>* state) const {
//    const KneedcompassgaitContinuousstate<T>& cs =
//            get_continuous_state(context);
//    KneedcompassgaitContinuousstate<T>& next_state =
//            get_mutable_continuous_state(
//            &(state->get_mutable_continuous_state()));
//    const KneedcompassgaitParams<T>& p = get_parameters(context);
//
//    using std::cos;
//    using std::sin;
//
//    // Shorthand used in the textbook.
//    const T l = p.length_leg();
//    const T sst = -sin(cs.angle_stance_knee()+cs.angle_hip());
//    const T ssw = sin(cs.angle_swing_knee()+cs.angle_hip()+cs.angle_thigh());
//    const T shp = sin(cs.angle_hip());
//    const T stg = -sin(cs.angle_thigh()+cs.angle_hip());
//    const T cst = -cos(cs.angle_stance_knee()+cs.angle_hip());
//    const T csw = cos(cs.angle_swing_knee()+cs.angle_hip()+cs.angle_thigh());
//    const T chp = cos(cs.angle_hip());
//    const T ctg = -cos(cs.angle_thigh()+cs.angle_hip());
//    const T toelen = l*(sst+ssw+shp+stg);
//
//     // TODO(Junda): write this part into a private method
//    // attain the generelized coordination nq, nv
//    systems::BasicVector<T> x_temp(18);
//    this->FloatingBaseStateOut(context, &x_temp);
//    VectorX<T> x = x_temp.CopyToVector();
//    int num = x.size()/2;
//    Eigen::Matrix<T, 9, 1> nq, nv;
//    for (int i = 0; i < num; ++i) {
//     nq[i] = x[i];
//     nv[i] = x[i+num];
//    }
//    std::cout << "nq: \n" << nq << std::endl;
//    std::cout << "nv:\n" << nv << std::endl;

    //do Kinematics
//     KinematicsCache<T> kinsol = rigidtree->
//             template CreateKinematicsCacheWithType<T>();
//    kinsol.initialize(nq, nv);
//    auto kinsol = rigidtree->doKinematics(nq, nv, true);
//
//    auto H = rigidtree->massMatrix(kinsol); // mass matrix H(q)9*9
//
//    //find the swing leg and regard it as rigidbody
//    RigidBody<T>* swing_leg;
//    if(left_leg_is_stance(context))
//        swing_leg = rigidtree->FindBody("right_lower_leg");
//    else
//        swing_leg = rigidtree->FindBody("left_lower_leg");
//    Eigen::Isometry3d Trans = Eigen::Isometry3d::Identity();
//    Trans.pretranslate(Eigen::Vector3d(0.5, 0, 0));
//    //define the rigid frame
//    RigidBodyFrame<T> swing_point("swing_point", swing_leg, Trans);

//     auto stance_J = rigidtree->CalcFrameSpatialVelocityJacobianInWorldFrame(
//         kinsol, swing_point);
//    Eigen::Matrix<double, 3, 9> J; // Jacobian
//    J << stance_J;
//
//    auto M_floating_base = H.inverse()*J.transpose()*(
//            J*H.inverse()*J.inverse()).inverse()*J;
//
//    // Floating-base velocity after contact.
//    auto v_post = nv - M_floating_base*nv;

//    auto v_post = nv;
//    // Also switch stance and swing legs:
//    next_state.set_angle_stance_knee(cs.angle_swing_knee());
//    next_state.set_angle_swing_knee(cs.angle_stance_knee());
//    next_state.set_angle_hip(cs.angle_hip()+cs.angle_thigh());
//    next_state.set_angle_thigh(-cs.angle_thigh());
//    next_state.set_angledot_stance_knee(v_post(3));
//    next_state.set_angledot_swing_knee(v_post(2));
//    next_state.set_angledot_hip(v_post(4)+v_post(5));
//    next_state.set_angledot_thigh(-v_post(5));
//
//    // toe += step length.
//    set_toe_position(get_toe_position(context) - toelen, state);
//
//    // Switch stance foot from left to right (or back).
//    set_left_leg_is_stance(!left_leg_is_stance(context), state);
//}

template <typename T>
void KneedCompassGait<T>::MinimalStateOut(
        const systems::Context<T>& context,
        KneedcompassgaitContinuousstate<T>* output) const {
    output->SetFromVector(get_continuous_state(context).CopyToVector());
}

template <typename T>
void KneedCompassGait<T>::FloatingBaseStateOut(
    const systems::Context<T>& context,
    systems::BasicVector<T>* output) const {
const KneedcompassgaitContinuousstate<T>& cs =
        get_continuous_state(context);
const KneedcompassgaitParams<T>& p = get_parameters(context);
const T ltoe = get_toe_position(context);
const bool left_stance = left_leg_is_stance(context);

// x, y, z.
const T x = ltoe + p.length_leg()*sin(cs.angle_hip()+cs.angle_stance_knee())
        + p.length_leg()*sin(cs.angle_stance_knee());
const T y = 0.0;
const T z = p.length_leg()*cos(cs.angle_hip()+cs.angle_stance_knee()) +
        p.length_leg()*cos(cs.angle_hip());

output->SetAtIndex(0, x);
output->SetAtIndex(1, y);
output->SetAtIndex(2, z);

// const T right = left_stance ? cs.swing() : cs.stance();
// left and right here is talking about angle of left knee and right knee
const T left = left_stance ?
        cs.angle_stance_knee() :
        cs.angle_swing_knee();
const T left_thigh = left_stance ? cs.angle_hip() :
        cs.angle_thigh() + cs.angle_hip();
//    const T right_thigh = left_stance ? cs.angle_hip()+cs.angle_thigh() :
//            cs.angle_hip();

// roll, pitch, yaw.
output->SetAtIndex(3, 0.);
// Left leg is attached to the floating base.
output->SetAtIndex(4, left/2 + left_thigh);
output->SetAtIndex(5, 0.);

// Hip angle .
output->SetAtIndex(6, -cs.angle_thigh());
// left and right knee angle
output->SetAtIndex(7, cs.angle_stance_knee());
output->SetAtIndex(8, cs.angle_swing_knee());

// x, y, z derivatives.
// TODO(Junda): need to be modified
const T x_dot = cs.angledot_stance_knee();
const T y_dot = 0.0;
const T z_dot = cs.angledot_stance_knee();
output->SetAtIndex(9, x_dot);
output->SetAtIndex(10, y_dot);
output->SetAtIndex(11, z_dot);

const T leftdot =
        left_stance ? cs.angledot_stance_knee()+cs.angledot_hip() :
        cs.angledot_swing_knee()+cs.angledot_thigh()+cs.angledot_hip();

// roll, pitch, yaw derivatives.
output->SetAtIndex(12, 0.);
output->SetAtIndex(13, leftdot);
output->SetAtIndex(14, 0.);

// Hip angle derivative.
output->SetAtIndex(15, -cs.angle_thigh());

output->SetAtIndex(16, cs.angledot_stance_knee());
output->SetAtIndex(17, cs.angledot_swing_knee());
}

  // cs means continuous states
  // p means parameters
template <typename T>
Vector4 <T> KneedCompassGait<T>::DynamicsBiasTerm(
        const systems::Context<T>& context) const {
    const KneedcompassgaitContinuousstate<T>& cs =
            get_continuous_state(context);
    const KneedcompassgaitParams<T>& p = get_parameters(context);

    using std::sin;

    // Shorthand used in the textbook.
    const T m = p.mass_leg();
    const T mh = p.mass_hip();
    const T l = p.length_leg();
    const T a1 = p.length_leg() - p.length_hip2upper_leg();
    const T b1 = p.length_hip2upper_leg();
//    const T a2 = p.length_leg() - p.length_knee2lower_leg();
//    const T b2 = p.length_knee2lower_leg();
    const T wls = cs.angledot_stance_knee();
    const T wrs = cs.angledot_swing_knee();
//    const T wlt = cs.angledot_hip();
//    const T wrt = cs.angledot_thigh();
    const T g = p.gravity();

    Vector4<T> bias{
            -m * l * b1 * wls * wls  -
            (mh * l + m * (a1 + l)) * g ,
            m * l * b1 * wrs * wrs + m * b1 * g,
            1.0,
            1.0};

    return bias;
}


template <typename T>
void KneedCompassGait<T>::DoCalcTimeDerivatives(
        const systems::Context<T>& context,
        systems::ContinuousState<T>* derivatives) const {
    const KneedcompassgaitContinuousstate<T>& cs =
            get_continuous_state(context);

    // get generalized coordination
    VectorX<T> state(18);
    state << this->get_output_port(1).Eval(context);
    int num = 18;
    VectorX<T> nq(9), nv(9);
    for (int i = 0; i < num; ++i) {
        nq[i] = state[i];
        nv[i] = state[i+num];
    }

    // get Kinematic cache and initialize it
    auto kinsol = rigidtree->CreateKinematicsCache();
    kinsol.initialize(nq, nv);
    rigidtree->doKinematics(kinsol, true);
//    std::cout << context.num_continuous_states() << std::endl;
//    auto kinsol = rigidtree->CreateKinematicsCache();
//    rigidtree->doKinematics(kinsol);

//    Eigen::Matrix<T, 9, 9> H = rigidtree->massMatrix(kinsol);
//    Eigen::Matrix<T, 9, 1> bias = rigidtree->dynamicsBiasTerm(kinsol, {});

    Eigen::Matrix <T, 8, 1> xdot;
    xdot << cs.angledot_stance_knee(),
    cs.angledot_swing_knee(), 1.0, 0, 0, 0, 0, 0;
//    -H.inverse() * bias;

    derivatives->SetFromVector(xdot);
}


//template <typename T>
//void KneedCompassGait<T>::DoGetWitnessFunctions(
//        const systems::Context<T>&,
//        std::vector<const systems::WitnessFunction<T>*>* witnesses) const {
//    witnesses->push_back(foot_collision_.get());
//}

}  // namespace KneedCompassGait
}  // namespace examples
}  // namespace drake


DRAKE_DEFINE_CLASS_TEMPLATE_INSTANTIATIONS_ON_DEFAULT_SCALARS(
        class ::drake::examples::KneedCompassGait::KneedCompassGait)
