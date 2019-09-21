#include <memory>
#include <vector>
#include <string>

#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_ContinuousState.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_Params.h"
#include "drake/examples/KneedCompassGait/linear_system.h"
#include "drake/systems/framework/event.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/scalar_conversion_traits.h"
#include "drake/systems/framework/witness_function.h"
#include "drake/multibody/parsers/urdf_parser.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/math/rotation_matrix.h"
#include "drake/common/find_resource.h"
#include "drake/math/rotation_matrix.h"
#include "drake/systems/framework/diagram_builder.h"
#include "drake/multibody/rigid_body_plant/drake_visualizer.h"
#include "drake/lcm/drake_lcm.h"
#include "drake/multibody/kinematics_cache.h"
#include "drake/multibody/plant/contact_jacobians.h"



namespace drake {
namespace examples {
namespace KneedCompassGait {
namespace {
int DoMain() {
    auto tree = new RigidBodyTree<double>();
    //   auto tree = std::make_unique<RigidBodyTree<double>>();

    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(
            "drake/examples/KneedCompassGait/KneedCompassGait.urdf"),
                    multibody::joints::kRollPitchYaw, tree);

    using std::cout;
    using std::cin;
    using std::endl;
//  TODO(Junda): this part is doKinematics, but have problem
//  to call for doKinematics, because of the shard.

//    Vector3<double> toe_collision_bias(0, 0, -0.5);
//    drake::WrenchVector<double> external_wrench;
//    VectorX<double> q_in(9), qd_in(9);
//    q_in << 0, 0, 1, 1, 0, 0, 0, 0, 0;
//    qd_in << 0, 1, 0, 0, 0, 0, 0, 0, 0;
//    auto kinematics = tree->doKinematics(q_in, qd_in);

    VectorX <double> q(4);
    VectorX <double> qd(5);
    q[1] = qd[1];

    Eigen::MatrixXd Jp;
    Eigen::VectorXi idxA, idxB;
    Eigen::Matrix3d xA, xB;
  //  tree->computeContactJacobians(kinematics, idxA, idxB, xA, xB, Jp);

  Eigen::Matrix<double, 3, 3> H;
  H << 1, 2, 3, 4, 5, 6, 7, 8, 9;
  Eigen::Matrix<double, 2, 3> h(H.bottomRows(2));
  H.setIdentity();h.setZero();



  linear_system::System test_sys;

//    Eigen::Matrix<double, 3, 1> collision;
//    collision << 0, //0,
//                0, //1,
//                1; //0;
//
//    Eigen::MatrixXd ll;
//    ll = tree->transformPointsJacobian(kinematics, collision, tree->FindBodyIndex("left_lower_leg"), 1, 0);
//    cout << ll << endl;
   // cout << tree->positionConstraintsJacobian(kinematics) << endl;

//    auto lb = 5*Eigen::VectorXf::Ones(3);
//    auto ub = 1e6*Eigen::VectorXf::Ones(3);
//    cout << lb << ub << endl;

//    Eigen::Matrix<double, 9, 1> w_qdd;
//    w_qdd << 10, 10, 10, 1000, 1000, 1000, 1, 1, 1;
//    auto weight = Eigen::MatrixXd(w_qdd.asDiagonal());
//    Eigen::Matrix3d I1;
//    I1.setIdentity();
//    Eigen::Matrix<double, 6, 3> I2;
//    Eigen::Matrix<double, 9, 3> I;
//    I2.setZero();
//    I << I2, I1;
//    cout << I << endl;
//    auto toe_l = tree->transformPoints(kinematics, toe_collision_bias, tree->FindBodyIndex("left_lower_leg"), 0);
//    cout << toe_l << endl;

//    cout << kinematics.getQ() << endl;
//    cout << tree->transformQDotToVelocity(
//            kinematics, qd_in) << endl;

//    auto kinematics = tree->CreateKinematicsCache();
//    tree->doKinematics(q_in, qd_in, true);

//    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
//    T.pretranslate(Eigen::Vector3d(0.5, 0, 0));
//    auto H = tree->massMatrix(kinematics);
//    auto C = tree->dynamicsBiasTerm(kinematics, {});
//    auto lshin = tree->FindBody("left_lower_leg");
//    RigidBodyFrame<double> stance_point("stance_leg", lshin, T);
//    auto stance_J = tree->CalcFrameSpatialVelocityJacobianInWorldFrame(
//            kinematics, stance_point);
//    cout << stance_J << "\n" << endl;
//    Eigen::Matrix<double, 3, 9> reduced_J;
//    reduced_J << stance_J;
//    cout << reduced_J << endl;
//    auto v_wf = tree->CalcFrameSpatialVelocityInWorldFrame(
//            kinematics, stance_point);
//    cout << "v_wf" << v_wf << endl;
//    auto rshin = tree->FindBody("right_lower_leg");


//    cout << tree->CalcBodySpatialVelocityInWorldFrame(
//            kinematics, *lshin).matrix() << endl;

//    Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity( );
//    Eigen::AngleAxisd rotation_vector( M_PI / 2, Eigen::Vector3d( 0, 0, 1 ) );
//    cout << "Vector3d = \n" << Eigen::Vector3d(0, 0, 1) << endl;
//    rotation_matrix = rotation_vector.toRotationMatrix( );

//    Eigen::Vector3d v( 1, 0, 0 );
//    Eigen::Vector3d v_rotated = rotation_vector * v;
//    cout << "( 1, 0, 0 ) after rotation = \n" << v_rotated << endl;
//    v_rotated = rotation_matrix * v;
//    cout << "( 1, 0, 0 ) after rotation = \n" << v_rotated << endl;

//    Eigen::Isometry3d T = Eigen::Isometry3d::Identity( );  // 三维变换矩阵
//    T.rotate( rotation_vector );  // 旋转部分赋值
//    T.pretranslate( Eigen::Vector3d( 1, 0, 0 ) );  // 设置平移向量
//    cout << "Transform matrix = \n" << T.matrix( ) << endl;
//    cout << tree->centerOfMassJacobianDotTimesV(kinematics) << endl;
//    cout << tree->centerOfMassJacobian(kinematics) << endl;
//    cout << X_BF.matrix() << endl;
//    cout << tree->CalcFrameSpatialVelocityJacobianInWorldFrame(
//            kinematics, body, X_BF) << endl;
//    cout << "num_pos" << kinematics.get_num_positions() << endl;
//    cout << "num_vel:" << kinematics.get_num_velocities() << endl;
//    cout << "num_cache_element" << kinematics.get_num_cache_elements()<< endl;
//    cout << "Q: " << kinematics.getQ() << endl;
//    cout << "hasV? " << kinematics.hasV() << endl;
//    cout << "X: " << kinematics.getX() << endl;

//    kinematics.CreateCacheElement(6, 6);
// cout << "num_pos" << kinematics.get_num_positions() << endl;9
// cout << "num_vel:" << kinematics.get_num_velocities() << endl;9
// cout << "num_cache_element" << kinematics.get_num_cache_elements()<< endl;10
// cout << "Q: " << kinematics.getQ() << endl;9*0
// cout << "hasV? " << kinematics.hasV() << endl;0
// cout << "X: " << kinematics.getX() << endl;9*0

//    auto actuator = tree->actuators;
//    cout << actuator.size() << endl;
//  cout << "hip_pin: "<<  tree->FindIndexOfChildBodyOfJoint("hip_pin")<< endl;
//  cout << tree->FindIndexOfChildBodyOfJoint("left_leg_weld")<< endl;
//  cout << "lkp:"<< tree->FindIndexOfChildBodyOfJoint("left_knee_pin")<< endl;
//  cout << "rp:"<< tree->FindIndexOfChildBodyOfJoint("right_knee_pin")<< endl;
//    auto rigid = tree->FindBody("right_upper_leg");
//
//    rigid->set_position_start_index(100);
//
//    tree->doKinematics(kinematics);
//    cout << tree->massMatrix(kinematics) << endl;
//    Eigen::Matrix<double, 9, 1> q;
//    q << 1, 2, 3, 4;
//    auto kinsol = tree->doKinematics(q);
//    auto tree2 = new RigidBodyTree<double>();
//
//    parsers::urdf::AddModelInstanceFromUrdfFileToWorld(FindResourceOrThrow(
//            "drake/examples/KneedCompassGait/cassie.urdf"),
//                    multibody::joints::kRollPitchYaw, tree2);

//    drake::systems::BasicVector<double> q(4);
//    drake::systems::BasicVector<double> qd(4);
//
//    auto sq = q.CopyToVector();
//    auto sqd = qd.CopyToVector();
//
//
//    qd << 1.2, 2, 3, 4;
//    auto kinematics = tree->CreateKinematicsCache();
//    auto kinematics = tree->doKinematics(q, qd);

//    drake::multibody::collision::ElementId elementid = 1;
//    auto rigidbody = tree->FindBody(-1);
//    drake::eigen_aligned_std_unordered_map<
//            rigidbody, drake::WrenchVector<double>> eWrench;


//    auto num = tree->get_num_positions() + tree->get_num_velocities();
//    for (int i = 0; i < num; ++i) {
//            cout << tree->getStateName(i) << endl;
//           // cout << tree->get_velocity_name(i) << endl;
//    }

//    Eigen::Matrix<double, 6, 9> M;
//    for (int i = 0; i < 54; ++i) {
//        M(i) = 1.;
//    }
//
//    cout << tree->get_num_velocities() << endl;
//    cout << tree->get_num_positions() << endl;
    return 0;
}
}
}  // namespace KneedCompassGait
}
}

int main(void) {
    return drake::examples::KneedCompassGait::DoMain();
}
