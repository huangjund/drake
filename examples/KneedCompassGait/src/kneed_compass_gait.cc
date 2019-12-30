#include "drake/examples/KneedCompassGait/include/kneed_compass_gait.h"

#define QUAT 25
#define ACCURACY 1e-6

using namespace std;
using namespace drake::multibody;

namespace kneed_compass_gait
{

  KneedCompassGait::KneedCompassGait()
  {  
    DeclareDiscreteState(QUAT);
    DeclareVectorOutputPort("floating_base_state", drake::systems::BasicVector<double>(QUAT),
                            &KneedCompassGait::CopyDiscreteStateOut);
    DeclarePeriodicDiscreteUpdate(0.001);
  }

  void KneedCompassGait::InitSystem(
      std::string urdf_path,
      std::vector<Eigen::Matrix<double, 9, 1>> COM_l,
      std::vector<Eigen::Matrix<double, 9, 1>> l_foot_l,
      std::vector<Eigen::Matrix<double, 9, 1>> r_foot_l){
      drake::multibody::Parser parser(&KCG);
      parser.AddAllModelsFromFile(drake::FindResourceOrThrow(urdf_path));
      KCG.Finalize();
      
      COM_list = COM_l;
      l_foot_list = l_foot_l;
      r_foot_list = r_foot_l;
  }

  void KneedCompassGait::DoCalcDiscreteVariableUpdates(
      const drake::systems::Context<double>& context,
      const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
      drake::systems::DiscreteValues<double>* updates) const {
    t = context.get_time();

    while (time < t - h)
    {
      drake::multibody::InverseKinematics kinsol(KCG);
      // Position Constraint
      const auto& world = KCG.world_frame();
      const auto& right_lower_leg = KCG.GetFrameByName("right_lower_leg");

      // psudo com constrain
      const Eigen::Vector3d BQ1(0, 0, 0);
      const Eigen::Vector3d AQ_lower1(COM_list[step](0, 0), 
                                      COM_list[step](1, 0), 
                                      COM_list[step](2, 0));
      const Eigen::Vector3d AQ_upper1(COM_list[step](0, 0), 
                                      COM_list[step](1, 0), 
                                      COM_list[step](2, 0));
      const Eigen::Ref<const Eigen::Vector3d>& p_BQ1 = BQ1;
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_lower1 = AQ_lower1;
      const Eigen::Ref<const Eigen::Vector3d>& p_AQ_upper1 = AQ_upper1;
      const auto& hip = KCG.GetFrameByName("base_link");
      auto pos1 = kinsol.AddPositionConstraint(hip, p_BQ1, 
          world, p_AQ_lower1, p_AQ_upper1);

      // Get mathematical program
      auto prog = kinsol.get_mutable_prog();
      auto& q = prog->decision_variables();

      // right foot trajectory verification
      auto kcg_context = KCG.CreateDefaultContext();
      kcg_context->get_mutable_continuous_state_vector().SetFromVector(context.get_discrete_state_vector().CopyToVector());
      auto transform = KCG.CalcRelativeTransform(*kcg_context, right_lower_leg, world);
      Eigen::Vector4d p_foot(0, 0, -0.25, 1);
      auto r_foot = transform.GetAsMatrix34()*p_foot;

      // Angle constrain
      prog->AddBoundingBoxConstraint(-M_PI, 0, {q.segment(10,1),q.segment(12,1)});

      // Solve IK
      Eigen::VectorXd q_guess = Eigen::VectorXd::Constant(KCG.num_positions(), 0.1);
      const auto result = drake::solvers::Solve(*prog, q_guess);
      const Eigen::VectorXd result_vec = result.GetSolution();

      const Eigen::VectorXd v = Eigen::VectorXd::Constant(12,0);

      Eigen::VectorXd state(QUAT);

      state << result_vec.segment(3,4),result_vec.segment(0,3),result_vec.segment(7,6), v;

      cout << r_foot[0] << '\t' << r_foot[1] << '\t' << r_foot[2] << endl;

      updates->get_mutable_vector().SetFromVector(state);

      time += h;
      step += 1;
    }
  }

  void KneedCompassGait::CopyDiscreteStateOut(
      const drake::systems::Context<double>& context,
      drake::systems::BasicVector<double>* output) const 
  {
    auto d_state = context.get_discrete_state().get_vector().CopyToVector();
    output->SetFromVector(d_state);
  }

}  // namespace kneed_compass_gait