//
// Created by Junda Huang on 9/2/19.
//
#pragma once

#include <memory>
#include <iostream>
#include <iomanip>

#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_ContinuousState.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_Params.h"
#include "drake/examples/KneedCompassGait/gen/KCG_Input.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/solvers/mathematical_program_result.h"
#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/constraint.h"
#include "drake/solvers/solve.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/framework/vector_system.h"
#include "drake/systems/framework/discrete_values.h"
#include "drake/lcm/drake_lcm.h"

#define NQ 11
#define NV 11
#define NU 5
#define DIM 3
#define ND 4 // friction cone approx
#define NC 2 // 2 on the left 2 on the right
#define NF 8 // number of contact force variables
#define m_surface_tangents 4
#define mu 1.0
#define COM 6 // state of COM
#define alpha 0
#define ZH 0.1 // the desired z height of foot toe
#define contact_phi 1e-3
#define W1 100
#define W2 0
#define W3 0
#define W4 1
#define W5 1e8
#define W6 1e5
#define KPXDDX 64
#define KDXDDX 16
#define KPXDDY 100
#define KDXDDY 10
#define KPXDDZ 100
#define KDXDDZ 10
#define Kpx 11 // the Kp of foot coordinate x
#define Kdx 5 // the Kd of foot coordinate x
#define Kepx 0.1
#define Kpy 16 // the Kp of foot coordinate y
#define Kdy 4 // the Kd of foot coordinate y
#define Kepy 0.1
#define KpzUP 25 // the Kp of foot coordinate z
#define KdzUP 5 // the Kd of foot coordinate z
#define KpzDOWN 16
#define KdzDOWN 1
#define Kepz 0.1
#define THRESH 0.5 // the threshhold of zdot decrease
#define Kpyaw 1 // the Kp of base yaw

namespace drake {
    namespace examples {
        namespace qpControl {
            class qpController : public systems::LeafSystem<double> {
            public:
                qpController() {
                    // get rigidbodytree of kneed compass gait
                    this->rtree_=getKCGTree<double>();

                    this->neps = NC*DIM; // number of slack variables for contact

                    this->DeclareVectorInputPort("KCG_states", systems::BasicVector<double>(NQ+NV));
                    this->DeclareVectorInputPort("trajectory", systems::BasicVector<double>(COM));
                    this->DeclareVectorInputPort("discrete_state", systems::BasicVector<double>(8)); // 1 is left and 0 is right
                    // (stance_x, stance_y, stance_z,
                    // next_stance_x, next_stance_y, next_stance_z,
                    // current_stance_leg, next stance_leg)
                    this->DeclareVectorOutputPort(KneedCompassGait::KcgInput<double>(),
                                                  &qpController::CopyCommandOutSim);
                    this->DeclareDiscreteState(NU);

                    // constexpr double print_period = 0.05;
                    // double last_print = -print_period;
                    // auto last_v = Eigen::VectorXd::Zero(3); // initialization

                    this->DeclarePeriodicDiscreteUpdate(0.0005);
                    VectorX<double> q_d(NQ);
                    q_d.setZero();

                    this->lfoot_ = this->rtree_->FindBody("left_lower_leg");
                    this->rfoot_ = this->rtree_->FindBody("right_lower_leg");

                    // set motor effort limit
                    VectorX<double> umin(NU), umax(NU);
                    auto motor = rtree_->actuators;
                    for (int i = 0; i < NU; ++i) {
                        umin[i] = motor[i].effort_limit_min_;
                        umax[i] = motor[i].effort_limit_max_;
                    }

                    // constexpr double slack_limit = 10.0;
                    this->initialized = false;

                    Eigen::Matrix<double, NU, 1> t_pre;
                    t_pre.setZero();

                    // add decision variables--------------------------------------------------------------------
                    auto vdot = prog.NewContinuousVariables(NV, "joint acceleration"); // qddot var
                    auto u = prog.NewContinuousVariables(NU, "input");  // torque var
                    auto beta = prog.NewContinuousVariables(NF, "friction forces"); // contact force var
                    auto eps = prog.NewContinuousVariables(2, "slack variables"); // 2 knee slack variables
                    auto yita_st = prog.NewContinuousVariables(3, "stance leg slack"); // 3 stance leg slack variables
                    auto yita_sw = prog.NewContinuousVariables(3, "swing leg slack"); // 3 swing leg slack variables
                    auto x_yaw = prog.NewContinuousVariables(1, "yaw");  // robot yaw
                    //   int nparams = prog.num_vars();

                    // Problem Constrains -----------------------------------------------------------------------
                    auto con_u_lim = prog.AddBoundingBoxConstraint(umin,umax, u).evaluator();
                    auto con_fric_lim = prog.AddBoundingBoxConstraint(5, 1000000, beta).evaluator();

                    this->Aeq_slack << 1, 0, -1, 0,
                            0, 1, 0, -1;
                    Eigen::Matrix<double, 2, 4> Aeq_temp;
                    Eigen::Vector2d beq_slack;
                    beq_slack.setZero();Aeq_temp.setZero();
                    auto con_slack = prog.AddLinearEqualityConstraint(
                            Aeq_temp, beq_slack, {vdot.segment(8,1), vdot.segment(10,1), eps}).evaluator();

                    Eigen::Matrix<double, 3, NQ+3> Aeq_stance;
                    Eigen::Matrix<double, 3, 1> beq_stance;
                    Aeq_stance.setZero();beq_stance.setZero();
                    auto con_slack2 = prog.AddLinearEqualityConstraint(Aeq_stance, beq_stance, {vdot, yita_st}).evaluator();

                    Eigen::Matrix<double, 3, NQ+3> Aeq_swing;
                    Eigen::Matrix<double, 3, 1> beq_swing;
                    Aeq_swing.setZero();beq_swing.setZero();
                    auto con_slack3 = prog.AddLinearEqualityConstraint(Aeq_swing, beq_swing, {vdot, yita_sw}).evaluator();

                    Eigen::Matrix<double, NQ-NU, NQ+NF> idyn_conf;
                    idyn_conf.setZero();
                    auto con_dynf = prog.AddLinearEqualityConstraint(
                            idyn_conf, Eigen::VectorXd::Zero(NQ-NU), {vdot, beta}).evaluator();

                    Eigen::Matrix<double, NU, NQ+NU+NF> idyn_cona;
                    idyn_cona.setZero();
                    auto con_dyna = prog.AddLinearEqualityConstraint(
                            idyn_cona, Eigen::VectorXd::Zero(NU), {vdot, u, beta}).evaluator();

                    Vector1<double> Aeq_yaw(0);
                    Vector1<double> beq_yaw(0);
                    auto con_yaw = prog.AddLinearEqualityConstraint(Aeq_yaw, beq_yaw, x_yaw).evaluator();

                    Eigen::Matrix2d Aeq_tau;
                    Aeq_tau.setIdentity();
                    Eigen::Vector2d beq_tau;
                    beq_tau.setZero();
                    auto con_tau = prog.AddLinearEqualityConstraint(Aeq_tau, beq_tau, {u.segment(0,1), u.segment(3,1)});
                    // Problem Cost -----------------------------------------------------------------------------

                    // set the Kp and Kd for COM xddot
                    this->Kp_xdd_.setOnes();
                    this->Kd_xdd_.setOnes();
                    this->Kp_xdd_(0,0) *= KPXDDX;
                    this->Kd_xdd_(0,0) *= KDXDDX;
                    this->Kp_xdd_(1,0) *= KPXDDY;
                    this->Kd_xdd_(1,0) *= KDXDDY;
                    this->Kp_xdd_(2,0) *= KPXDDZ;
                    this->Kd_xdd_(2,0) *= KDXDDZ;

                    // the first ID term
                    Eigen::Matrix<double, NQ, NQ> Q_xddot;
                    Eigen::Matrix<double, NQ, 1> b_xddot;
                    b_xddot.setZero();Q_xddot.setIdentity();
                    auto cost_xddot = prog.AddQuadraticCost(Q_xddot, b_xddot, vdot).evaluator();

                    // the second term
                    this->weight_t_.setIdentity();
                    this->weight_t_ *= W2;
                    Eigen::Matrix<double, NU, NU> Q_t;
                    Eigen::Matrix<double, NU, 1> b_t;
                    Q_t.setIdentity();b_t.setZero();
                    auto cost_t = prog.AddQuadraticCost(Q_t, b_t, u).evaluator();

                    // the third term
                    this->weight_yaw_.setIdentity();
                    this->weight_yaw_ *= W3;
                    Eigen::Matrix<double, 1, 1> Q_yaw(1);
                    Vector1<double> b_yaw(0);
                    auto cost_yaw = prog.AddQuadraticCost(Q_yaw, b_yaw, x_yaw).evaluator();

                    // the forth term
                    this->weight_slack_.setIdentity();
                    this->weight_slack_ *= W4;
                    Eigen::Matrix<double, 2, 2> Q_slack;
                    Eigen::Vector2d b_slack;
                    Q_slack.setIdentity();b_slack.setZero();
                    auto cost_slack = prog.AddQuadraticCost(Q_slack, b_slack, eps).evaluator();

                    this->weight_slack2_.setIdentity();
                    this->weight_slack2_ *= W5;
                    Eigen::Matrix<double, 3, 3> Q_slack2;
                    Eigen::Matrix<double, 3, 1> b_slack2;
                    b_slack2.setZero();Q_slack2.setIdentity();
                    auto cost_slack2 = prog.AddQuadraticCost(Q_slack2, b_slack2, yita_st).evaluator();

                    this->weight_slack3_.setIdentity();
                    this->weight_slack3_ *= W6;
                    Eigen::Matrix<double, 3, 3> Q_slack3;
                    Eigen::Matrix<double, 3, 1> b_slack3;
                    Q_slack3.setIdentity();b_slack3.setZero();
                    auto cost_slack3 = prog.AddQuadraticCost(Q_slack3, b_slack3, yita_sw).evaluator();

                    // solver setting -----------------------------------------------------------------------------
                    this->prog.SetSolverOption(this->solver.solver_id(), "Method", 2);

                    // save variables ----------------------------------------------------------------------------
                    this->q_des_ = q_d;
                    this->con_slack_ = con_slack;
                    this->con_slack2_ = con_slack2;
                    this->con_slack3_ = con_slack3;
                    this->con_dynf_ = con_dynf;
                    this->con_dyna_ = con_dyna;
                    this->con_yaw_ = con_yaw;
                    this->con_fric_lim_ = con_fric_lim;
                    this->cost_xddot_ = cost_xddot;
                    this->cost_t_ = cost_t;
                    this->cost_yaw_ = cost_yaw;
                    this->cost_slack_ = cost_slack;
                    this->cost_slack2_ = cost_slack2;
                    this->cost_slack3_ = cost_slack3;
                }

                void DoCalcDiscreteVariableUpdates(
                        const drake::systems::Context<double>& context,
                        const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
                        drake::systems::DiscreteValues<double>* updates) const override {
                    using std::cout;
                    using std::endl;
                    static Eigen::Matrix<double, NU, 1> t_pre;
                    t_pre.setZero();

                    //  auto utime = context.get_time()*1000000;

                    // get the current state and current disired trajectory
                    VectorX<double> x = this->EvalVectorInput(context, 0)->CopyToVector();
                    VectorX<double> traj = this->EvalVectorInput(context, 1)->CopyToVector();
                    VectorX<double> ds_state = this->EvalVectorInput(context, 2)->CopyToVector();
                    VectorX<double> xcom_des(COM), x_yaw_des(1); // qd_des(NV),
                    VectorX<double> q(NQ), qd(NV);
                    VectorX<double> this_stance(3),next_stance(3);

                    for (int i = 0; i < NQ; ++i) {
                        if (i < COM)
                            xcom_des[i] = traj[i];
                        if (i < 3){
                            this_stance[i] = ds_state[i];
                            next_stance[i] = ds_state[i+3];
                        }
                        q[i] = x[i];
                        qd[i] = x[i+NQ];
                    }
                    this->left_is_stance = ds_state(6,0);
                    x_yaw_des[1] = 0;

                    // create the current kinematic cache
                    auto kinsol = (this->rtree_)->doKinematics(q, qd);
                    Vector3<double> toe_collision_bias(0, 0, -0.5);

                    // setup some dynamic terms
                    auto H = rtree_->massMatrix(kinsol);
                    auto C = rtree_->dynamicsBiasTerm(kinsol, {}, true);
                    auto B = rtree_->B;
                    Eigen::MatrixXd JB;
                    contactJacobianBV(*(this->rtree_), kinsol, *(this->lfoot_), *(this->rfoot_), false, JB);

                    // floating-base seperating
                    Eigen::Matrix<double, NQ-NU, NQ> Hf(H.topRows(NQ-NU));
                    Eigen::Matrix<double, NU, NQ> Ha(H.bottomRows(NU));
                    Eigen::Matrix<double, NQ-NU, 1> Cf(C.topRows(NQ-NU));
                    Eigen::Matrix<double, NU, 1> Ca(C.bottomRows(NU));
                    Eigen::Matrix<double, NQ-NU, NF> JBf(JB.topRows(NQ-NU));
                    Eigen::Matrix<double, NU, NF> JBa(JB.bottomRows(NU));
                    Eigen::Matrix<double, NQ-NU, NU> Bf;
                    Eigen::Matrix<double, NU, NU> Ba;
                    Bf.setZero();Ba.setIdentity();

                    // COM dynamics
                    auto com = rtree_->centerOfMass(kinsol);
                    auto Jcom = rtree_->centerOfMassJacobian(kinsol);
                    auto Jcomdot_times_v = rtree_->centerOfMassJacobianDotTimesV(kinsol);
                    auto vcom = Jcom*qd;
                    Eigen::VectorXd phi;
                    contactDistances(*(this->rtree_), kinsol, *(this->lfoot_), *(this->rfoot_), phi);
//            cout << "phi1:" << phi[0] << "\tphi2:" << phi[1] << endl;

                    // foot dynamics
                    auto left_toe_jaco = rtree_->transformPointsJacobian(kinsol, toe_collision_bias, 7, 0, true);
                    auto right_toe_jaco = rtree_->transformPointsJacobian(kinsol, toe_collision_bias, 11, 0, true);
                    auto left_toe_jacodotv = rtree_->transformPointsJacobianDotTimesV(kinsol, toe_collision_bias, 7, 0);
                    auto right_toe_jacodotv = rtree_->transformPointsJacobianDotTimesV(kinsol, toe_collision_bias, 11, 0);
                    auto left_toe_pos = rtree_->transformPoints(kinsol, toe_collision_bias, 7, 0);
                    auto right_toe_pos = rtree_->transformPoints(kinsol, toe_collision_bias, 11, 0);
                    Eigen::Matrix<double, 3, 1> left_toe_Jqdot, right_toe_Jqdot;
                    left_toe_Jqdot << left_toe_jaco*qd;
                    right_toe_Jqdot << right_toe_jaco*qd;

                    // Problem Constraints ----------------------------------------------------------------------
                    Eigen::Matrix<double, NF, 1> l_contact;
                    Eigen::Matrix<double, NF, 1> u_contact;
                    l_contact.setOnes();u_contact.setOnes();
                    l_contact *= 0;
                    u_contact *= 1e6;
                    int count=0; // if count == 1; then is swing and stance phase; if count == 2, then is impact phase
                    for (int i = 0; i < NC; ++i) {
                        if(phi[i] > contact_phi){
                            count++;
                            for (int j = 0; j < ND; ++j) {
                                l_contact(4*i+j,0) = 0;
                                u_contact(4*i+j,0) = 0;
                            }
                        }
                    }
                    this->con_fric_lim_->set_bounds(l_contact, u_contact);

                    Eigen::Vector2d beq_slack;
                    beq_slack[0] = -5*tan(M_PI*((q[8]+3.14)/3.14 - 0.5));
                    beq_slack[1] = -5*tan(M_PI*((q[10]+3.14)/3.14 - 0.5));
                    this->con_slack_->UpdateCoefficients(this->Aeq_slack, beq_slack);

                    Eigen::Matrix<double, NQ-NU, NQ+NF> dyn_conf;
                    dyn_conf.setZero();
                    dyn_conf << Hf, -JBf;
                    this->con_dynf_->UpdateCoefficients(dyn_conf, -Cf);

                    Eigen::Matrix<double, NU, NQ+NU+NF> dyn_cona;
                    dyn_cona.setZero();
                    dyn_cona << Ha, -Ba, -JBa;
                    this->con_dyna_->UpdateCoefficients(dyn_cona, -Ca);

                    Vector1<double> Aeq_yaw(1);
                    Vector1<double> beq_yaw(q[5]);
                    this->con_yaw_->UpdateCoefficients(Aeq_yaw, beq_yaw);


                    // Problem Costs ---------------------------------------------------------------------------
                    auto kp = this->Kp_xdd_;
                    auto kd = this->Kd_xdd_;
                    auto xddot_des = kp.cwiseProduct(xcom_des.segment(0, 3)-com) +
                                     kd.cwiseProduct(xcom_des.segment(3,3)-vcom);


                    auto Q_xdd = 2*W1*Jcom.transpose()*Jcom;
                    auto b_xdd = 2*W1*Jcom.transpose()*(Jcomdot_times_v-xddot_des);
                    this->cost_xddot_->UpdateCoefficients(Q_xdd, b_xdd);


                    auto Q_t = 2*this->weight_t_;
                    auto b_t = 2*this->weight_t_.transpose()*(-t_pre);
                    this->cost_t_->UpdateCoefficients(Q_t, b_t);

                    auto Q_yaw = 2*this->weight_yaw_;
                    auto b_yaw = -2*this->weight_yaw_*x_yaw_des;
                    this->cost_yaw_->UpdateCoefficients(Q_yaw, b_yaw);

                    auto Q_slack = 2*this->weight_slack_;
                    Vector2<double> b_slack(0,0);
                    this->cost_slack_->UpdateCoefficients(Q_slack, b_slack);

                    // stance foot constraint and cost------------------------------------------------------------
                    Eigen::Matrix<double, 3, 3> I;
                    Eigen::Matrix<double, 3, 1> b_slack2;
                    Eigen::Matrix<double, 3, 1> b_slack3;
                    Eigen::Matrix<double, 3, NQ> zero3n;
                    Eigen::Matrix<double, 3, 3> zero33;
                    Eigen::Matrix<double, 3, 1> zero31;
                    zero3n.setZero();zero31.setZero();zero33.setZero();b_slack2.setZero();b_slack3.setZero();
                    I.setIdentity();
                    // stance parameters
                    Eigen::Matrix<double, 3, NQ> JStance;
                    Eigen::Matrix<double, 3, 1> JdotqdotStance;
                    Eigen::Matrix<double, 3, 1> JqdotStance;
                    Eigen::Matrix<double, 3, 1> beq_slack2;
                    Eigen::Matrix<double, 3, NQ+3> Aeq_slack2;
                    Eigen::Matrix<double, 3, 3> Q_slack2;
                    // swing parameters
                    Eigen::Matrix<double, 3, 1> Xddot;
                    Eigen::Matrix<double, 3, NQ> JSwing;
                    Eigen::Matrix<double, 3, 1> JdotqdotSwing;
                    Eigen::Matrix<double, 3, 1> beq_slack3;
                    Eigen::Matrix<double, 3, NQ+3> Aeq_slack3;
                    Eigen::Matrix<double, 3, 3> Q_slack3;
                    double xddot, yddot, zddot;

                    if (left_is_stance) {
                        // stance leg constraint
                        std::cout << "=======left is stance========" << std::endl;
                        JStance << left_toe_jaco;
                        JdotqdotStance << left_toe_jacodotv;
                        JqdotStance << alpha*left_toe_Jqdot;
                        beq_slack2 << -JdotqdotStance-JqdotStance;
                        Aeq_slack2 << JStance, -I;
                        this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                        // stance leg cost
                        Q_slack2.setIdentity();
                        Q_slack2 *= W5;
                        this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                        // swing leg constraint
                        double dis_x = next_stance[0] - right_toe_pos[0];
                        double dis_y = next_stance[1] - right_toe_pos[1];
                        xddot = Kpx*dis_x-Kdx*right_toe_Jqdot[0];
                        yddot = Kpy*dis_y-Kdy*right_toe_Jqdot[1];
                        if (fabs(dis_x) + fabs(dis_y) >= THRESH){
                            zddot = KpzUP * (ZH - right_toe_pos[2]) - KdzUP * right_toe_Jqdot[2];
                            Xddot << xddot, yddot, zddot;
                        }
                        else{
                            zddot = -KpzDOWN*right_toe_pos[2] - KdzDOWN*right_toe_Jqdot[2];
                            if (phi[1] < contact_phi)
                                Xddot << 0, 0, zddot;
                            else
                                Xddot << xddot, yddot, zddot;
                        }
                        JSwing << right_toe_jaco;
                        JdotqdotSwing << right_toe_jacodotv;
                        Aeq_slack3 << JSwing, -I;
                        beq_slack3 << Xddot-JdotqdotSwing;
                        this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                        // swing leg cost
                        Q_slack3.setIdentity();
                        Q_slack3 *= W6;
                        this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                    } else {
                        // stance leg constraint
                        cout << "======right is stance========" << endl;
                        JStance << right_toe_jaco;
                        JdotqdotStance << right_toe_jacodotv;
                        JqdotStance << alpha*right_toe_Jqdot;
                        beq_slack2 << -JdotqdotStance-JqdotStance;
                        Aeq_slack2 << JStance, -I;
                        this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                        // stance leg cost
                        Q_slack2.setIdentity();
                        Q_slack2 *= W5;
                        this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                        // swing leg constraint
                        double dis_x = next_stance[0] - left_toe_pos[0];
                        double dis_y = next_stance[1] - left_toe_pos[1];
                        xddot = Kpx*dis_x-Kdx*left_toe_Jqdot[0];
                        yddot = Kpy*dis_y-Kdy*left_toe_Jqdot[1];
                        if (fabs(dis_x)+fabs(dis_y) >= THRESH) {
                            zddot = KpzUP * (ZH - left_toe_pos[2]) - KdzUP * left_toe_Jqdot[2];
                            Xddot << xddot, yddot, zddot;
                        }
                        else{
                            zddot = -KpzDOWN*left_toe_pos[2] - KdzDOWN*left_toe_Jqdot[2];
                            if (phi[0] < contact_phi)
                                Xddot << 0, 0, zddot;
                            else
                                Xddot << xddot, yddot, zddot;
                        }
                        JSwing << left_toe_jaco;
                        JdotqdotSwing << left_toe_jacodotv;
                        Aeq_slack3 << JSwing, -I;
                        beq_slack3 << Xddot-JdotqdotSwing;
                        this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                        // swing leg cost
                        Q_slack3.setIdentity();
                        Q_slack3 *= W6;
                        this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);
                    }

                    // solve the problem --------------------------------------------------------------------------
                    drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(this->prog);
                    auto result_vec = result.GetSolution();
                    Eigen::Matrix<double, NU, 1> u(result_vec.middleRows(NQ, NU));

                    std::cout << "|=======com_des========\t" << "|========COM======\t"
                              << "|=======COMddot========\t" << "|=====COMdot_des==\t"
                              << "|==right toe pos=======\t" << "|==right toe velocity=\t"
                              << "|==left toe pos========\t" << "|===left toe velocity=\t"
                              << std::endl;
                    for (int k = 0; k < 3; ++k) {
                        std::cout << "|"<<std::fixed<<std::setprecision(8) << xcom_des.segment(0,3)[k] << "\t\t";
                        std::cout << "|"<<std::fixed<<std::setprecision(8)<< com[k] << "\t\t";
                        std::cout << "|"<<std::fixed<<std::setprecision(8)<< xddot_des[k] << "\t\t";
                        cout << "|" << std::fixed<<std::setprecision(8) << xcom_des.segment(3,3)[k] << "\t\t";
                        cout << "|" << std::fixed<<std::setprecision(8) << right_toe_pos[k] << "\t\t";
                        cout << "|" << std::fixed<<std::setprecision(8) << right_toe_Jqdot[k] << "\t\t";
                        cout << "|" << std::fixed<<std::setprecision(8) << left_toe_pos[k] << "\t\t";
                        cout << "|" << std::fixed<<std::setprecision(8) << left_toe_Jqdot[k] << "\t\t";
                        std::cout << "\n";
                    }

                    // print all dicision variables out ------------------------------------------------------------
                    std::cout <<"|==========q=======\t"<<"|========v=========\t" <<
                              "|=========vdot=========\t" << "|==========u==========\t"
                              << "|=========beta=========\t" <<  "|=====yita stance====\t"
                              << "|======yita swing======\t" << "|=====Xfootddot========\t" <<  std::endl;
                    for (int k = 0; k < NQ; ++k) {
                        cout << "|" << std::fixed<<std::setprecision(8) << q[k] << "\t\t";
                        cout << "|" << std::fixed<<std::setprecision(8) << qd[k] << "\t\t";
                        std::cout << "|"<<std::fixed<<std::setprecision(8) << result_vec.middleRows(0,NQ)[k] << "\t\t";
                        if (k<NU) // u
                            std::cout <<"|"<<std::fixed<<std::setprecision(8)<< result_vec.middleRows(NQ,NU)[k] << "\t\t";
                        else
                            std::cout <<"|"<< "\t\t\t";
                        if (k<NF) // beta
                            std::cout <<"|"<<std::fixed<<std::setprecision(8)<< result_vec.middleRows(NQ+NU,NF)[k] << "\t\t";
                        else
                            std::cout <<"|"<< "\t\t\t";
//                if (k<2) //eps
//                    std::cout <<"|"<<std::fixed<<std::setprecision(8)<< result_vec.middleRows(NQ+NU+NF,2)[k] << "\t\t";
//                else
//                    std::cout << "|" << "\t\t\t";
                        if (k<3) // yita_stance
                            std::cout <<"|"<<std::fixed<<std::setprecision(8)<< result_vec.middleRows(NQ+NU+NF+2,3)[k] << "\t\t";
                        else
                            std::cout <<"|"<< "\t\t\t";
                        if (k<3) // yita_swing
                            std::cout <<"|"<<std::fixed<<std::setprecision(8)<< result_vec.middleRows(NQ+NU+NF+5,3)[k] << "\t\t";
                        else
                            std::cout <<"|"<< "\t\t\t";
                        if (k<3)
                            std::cout <<"|"<<std::fixed<<std::setprecision(8)<< Xddot[k] << "\t\t";
                        else
                            cout << "|" << "\t\t\t";
                        if (k<2) // discrete state
                            std::cout <<"|"<<std::fixed<<std::setprecision(8)<< result_vec.middleRows(NQ+NU+NF,2)[k] << "\t\t";
                        else
                            std::cout << "|" << "\t\t\t";
                        std::cout << "\n";
                    }
                    updates->get_mutable_vector().SetFromVector(u);
                    t_pre = u;
                }

                void CopyCommandOutSim(const systems::Context<double>& context,
                                       drake::examples::KneedCompassGait::KcgInput<double>* output) const {
                    auto dis_state = context.get_discrete_state().get_vector().CopyToVector();
                    output->set_hrtau(dis_state[0]);
                    output->set_htau(dis_state[1]);
                    output->set_ltau(dis_state[2]);
                    output->set_hytau(dis_state[3]);
                    output->set_rtau(dis_state[4]);
                }


                Eigen::MatrixXd contactJacobian(const RigidBodyTree<double> &r,
                                                KinematicsCache<double> &cache,
                                                RigidBody<double> &lfoot, RigidBody<double> &rfoot,
                                                bool in_terms_of_qdot) const {
                    // std::cout << "ok3 " << std::endl;
                    Eigen::Matrix3Xd l_pts_foot;
                    Eigen::Matrix3Xd r_pts_foot;

                    r.getTerrainContactPoints(lfoot, &l_pts_foot);
                    r.getTerrainContactPoints(rfoot, &r_pts_foot);

                    Eigen::MatrixXd l_J = r.transformPointsJacobian(
                            cache, l_pts_foot, lfoot.get_body_index(), 0, in_terms_of_qdot);
                    Eigen::MatrixXd r_J = r.transformPointsJacobian(
                            cache, r_pts_foot, rfoot.get_body_index(), 0, in_terms_of_qdot);

                    Eigen::MatrixXd J(l_J.rows() + r_J.rows(), l_J.cols());
                    J << l_J, r_J;

                    return J;
                }

                Eigen::Matrix<double, 3, m_surface_tangents> surfaceTangents(Eigen::Vector3d normal) const {
                    //   std::cout << " ok4 " << std::endl;
                    const double kEpsilon = 1e-8;
                    Eigen::Vector3d t1;
                    Eigen::Vector3d t2;

                    if (1 - normal[2] < kEpsilon) {
                        t1 << 1, 0, 0;
                    } else if (1 + normal[2] < kEpsilon) {
                        t1 << -1, 0, 0;
                    } else {
                        t1 << normal[1], -normal[0], 0;
                        t1.normalize();
                    }

                    t2 = t1.cross(normal);
                    Eigen::Matrix<double, 3, m_surface_tangents> d;
                    for (int ii = 0; ii < m_surface_tangents; ii++) {
                        double theta = ii * 2 * M_PI / m_surface_tangents;
                        d.col(ii) = cos(theta) * t1 + sin(theta) * t2;
                    }
                    return d;
                }

                void contactJacobianBV(const RigidBodyTree<double> &r,
                                       KinematicsCache<double> &cache, RigidBody<double> &lfoot,
                                       RigidBody<double> &rfoot, bool in_terms_of_qdot,
                                       Eigen::MatrixXd &JB) const {
                    //   std::cout << " ok5 " << std::endl;
                    Eigen::MatrixXd J = contactJacobian(r, cache, lfoot, rfoot, in_terms_of_qdot);
                    // assume flat terrain at z=0 for now
                    Eigen::Vector3d normal;
                    normal << 0, 0, 1;
                    Eigen::Matrix<double, 3, m_surface_tangents> d = surfaceTangents(normal);

                    int inc = J.rows() / 3;
//            std::cout << "===================contact point number:" << inc << std::endl;
                    int ncols = inc * m_surface_tangents;
                    int nrows;
                    if (in_terms_of_qdot) {
                        nrows = r.get_num_positions();
                    } else {
                        nrows = r.get_num_velocities();
                    }

                    Eigen::Matrix3Xd B(3, ncols);  // Friction polyhedron basis vectors
                    //  B.resize(3, ncols);
                    JB.resize(nrows,ncols);  // Jacobian mapping forces along friction basis vectors

                    double norm = sqrt(1.0 + mu * mu);

                    for (int ii = 0; ii < inc; ii++) {
                        Eigen::Matrix3Xd sub_J(3, nrows);
                        sub_J << J.row(3 * ii), J.row(3 * ii + 1), J.row(3 * ii + 2);
                        for (int jj = 0; jj < m_surface_tangents; jj++) {
                            B.col(ii * m_surface_tangents + jj) = (normal + mu * d.col(jj)) / norm;
                            JB.col(ii * m_surface_tangents + jj) =
                                    sub_J.transpose() * B.col(ii * m_surface_tangents + jj);
                        }
                    }
                }

                void contactDistances(const RigidBodyTree<double> &r,
                                      KinematicsCache<double> &cache,
                                      RigidBody<double> &lfoot,
                                      RigidBody<double> &rfoot,
                                      Eigen::VectorXd &phi) const {
                    Eigen::Matrix3Xd pts = forwardKinTerrainPoints(r, cache, lfoot, rfoot);
                    int nc = pts.cols();

                    phi.resize(nc);
                    phi = pts.row(2);
                }

                Eigen::Matrix3Xd forwardKinTerrainPoints(const RigidBodyTree<double> &r,
                                                         KinematicsCache<double> &cache,
                                                         RigidBody<double> &lfoot,
                                                         RigidBody<double> &rfoot) const {
                    Eigen::Matrix3Xd l_pts_foot;
                    Eigen::Matrix3Xd r_pts_foot;

                    r.getTerrainContactPoints(lfoot, &l_pts_foot);
                    r.getTerrainContactPoints(rfoot, &r_pts_foot);

                    Eigen::Matrix3Xd l_pts =
                            r.transformPoints(cache, l_pts_foot, lfoot.get_body_index(), 0);
                    Eigen::Matrix3Xd r_pts =
                            r.transformPoints(cache, r_pts_foot, rfoot.get_body_index(), 0);

                    Eigen::Matrix3Xd pts(3, l_pts.cols() + r_pts.cols());
                    pts << l_pts, r_pts;

                    return pts;
                }

                template <typename Scalar>
                std::unique_ptr<RigidBodyTree<Scalar>> getKCGTree() {
                    auto tree = std::make_unique<RigidBodyTree<Scalar>>();
                    std::string urdf_path;
                    urdf_path =
                            "drake/examples/KneedCompassGait/KneedCompassGait.urdf";

                    drake::parsers::urdf::AddModelInstanceFromUrdfFileToWorld(
                            FindResourceOrThrow(urdf_path),
                            multibody::joints::kRollPitchYaw, tree.get());

                    return tree;
                }

            private:
                int neps; // number of slack variables for contact
                bool initialized;
                mutable bool left_is_stance;
                Eigen::Matrix<double, NU, NU> weight_t_;
                Eigen::Matrix<double, 1, 1> weight_yaw_;
                Eigen::Matrix<double, 2, 2> weight_slack_;
                Eigen::Matrix<double, 3, 3> weight_slack2_;
                Eigen::Matrix<double, 3, 3> weight_slack3_;
                Eigen::Matrix<double, 2, 4> Aeq_slack;
                std::shared_ptr<solvers::LinearEqualityConstraint> con_dynf_;
                std::shared_ptr<solvers::LinearEqualityConstraint> con_dyna_;
                std::shared_ptr<solvers::LinearEqualityConstraint> con_slack_;
                std::shared_ptr<solvers::LinearEqualityConstraint> con_slack2_;
                std::shared_ptr<solvers::LinearEqualityConstraint> con_slack3_;
                std::shared_ptr<solvers::LinearEqualityConstraint> con_yaw_;
                std::shared_ptr<solvers::BoundingBoxConstraint> con_fric_lim_;
                std::shared_ptr<solvers::QuadraticCost> cost_xddot_;
                std::shared_ptr<solvers::QuadraticCost> cost_t_;
                std::shared_ptr<solvers::QuadraticCost> cost_yaw_;
                std::shared_ptr<solvers::QuadraticCost> cost_slack_;
                std::shared_ptr<solvers::QuadraticCost> cost_slack2_;
                std::shared_ptr<solvers::QuadraticCost> cost_slack3_;
                Eigen::Matrix<double, NQ, 1> w_qdd_;
                solvers::GurobiSolver solver;
                solvers::MathematicalProgram prog;
                Eigen::Matrix<double, NQ, NQ> weight_;
                VectorX<double> q_des_;
                Eigen::Matrix<double, NQ, 1> Kp_qdd_;
                Eigen::Matrix<double, NV, 1> Kd_qdd_;
                Eigen::Matrix<double, COM/2, 1> Kp_xdd_;
                Eigen::Matrix<double, COM/2, 1> Kd_xdd_;
                std::unique_ptr<RigidBodyTree<double>> rtree_;
                RigidBody<double>* lfoot_;
                RigidBody<double>* rfoot_;
            };
        }
    }
}

