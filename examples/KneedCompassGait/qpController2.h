//
// Created by Junda Huang on 8/20/19.
//
#pragma once

#include <memory>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <vector>

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

// walk FOR eight step
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
#define ALPHA 0
#define gama -1
#define ZH 0.08 // the desired z height of foot toe
#define COMZH 0.7 // the heighest point of com z trajectory
#define STEP_LEN 0.4 // one foot step length
#define contact_phi 1e-3
#define W1 1e5
#define W1y 1e4
#define W2 1e3
#define W3 0
#define W4 0
#define W5 1e8
#define W6 1e5
#define W5i 1e5 // prevent front leg from slipping after touching ground
#define W6i 1e8

#define KPXDDX 10
#define KDXDDX 45
#define KPXDDY 1
#define KDXDDY 0
#define KPXDDZ 772 // increase to decrease height vs x
#define KDXDDZ 50 //50
#define KDZWEIGHT 10 // the weight of the differential parameter

#define Kpx_tra 12 // the Kp of foot coordinate x 1.9
#define Kdx_tra 4 // the Kd of foot coordinate x 0.5
#define Kpz_tra 9
#define Kdz_tra 5

#define Kpx_loc 30
#define Kdx_loc 5
#define KpzUP_loc 50
#define KdzUP_loc 0
#define KpzDOWN_loc 100
#define KdzDOWN_loc 0
#define KpzDOWN_loc_H 100
#define FastDownHeight 0.03
#define ADJUST 0.1

#define Kpy_tra 0// the Kp of foot coordinate y
#define Kdy_tra 0 // the Kd of foot coordinate y
#define Kpy_loc 100
#define Kdy_loc 0

#define Kpxi 6// the Kp of foot coordinate x
#define Kdxi 0 // the Kd of foot coordinate x
#define Kpyi 0 // the Kp of foot coordinate y
#define Kdyi 0 // the Kd of foot coordinate y
#define KpzUPi 3000// the Kp of foot coordinate z, consistent
#define KdzUPi 0// the Kd of foot coordinate z

// trick dumpe
#define KpzUPd 3000

// trick ground impulse parameters
#define Kpxip 0// the Kp of foot coordinate x
#define Kdxip 0 // the Kd of foot coordinate x
#define Kpyip 0 // the Kp of foot coordinate y
#define Kdyip 0 // the Kd of foot coordinate y
#define KpzUPip 8000// the Kp of foot coordinate z, consistent
#define KdzUPip 100// the Kd of foot coordinate z
#define THRESH 0.1 // the up and down tag
#define IMPULSE_TIME 3 // the impulse push time

#define Lowest_V 0.57
#define BandWidth_L 0.05
#define TOP 1.0
#define BOTTOM 0.5

#define Maxlateralbias 0.001
#define MaxLateralV 0.05

// WALKING FOR 8 STEPS
//#define NQ 11
//#define NV 11
//#define NU 5
//#define DIM 3
//#define ND 4 // friction cone approx
//#define NC 2 // 2 on the left 2 on the right
//#define NF 8 // number of contact force variables
//#define m_surface_tangents 4
//#define mu 1.0
//#define COM 6 // state of COM
//#define ALPHA 0
//#define gama -1
//#define ZH 0.05 // the desired z height of foot toe
//#define COMZH 0.7 // the heighest point of com z trajectory
//#define STEP_LEN 0.4 // one foot step length
//#define contact_phi 1e-3
//#define W1 1e5
//#define W1y 1e7
//#define W2 1e3
//#define W3 0
//#define W4 0
//#define W5 1e8
//#define W6 1e5
//#define W5i 1e5 // prevent front leg from slipping after touching ground
//#define W6i 1e8
//
//#define KPXDDX 1
//#define KDXDDX 5
//#define KPXDDY 0
//#define KDXDDY 0
//#define KPXDDZ 400 // increase to decrease height vs x
//#define KDXDDZ 30 //50
//#define KDZWEIGHT 10 // the weight of the differential parameter
//
//#define Kpx_tra 200 // the Kp of foot coordinate x 1.9
//#define Kdx_tra 20 // the Kd of foot coordinate x 0.5
//#define Kpz_tra 200
//#define Kdz_tra 20
//
//#define Kpx_loc 5
//#define Kdx_loc 0
//#define KpzUP_loc 0
//#define KdzUP_loc 0
//#define location_Kpzdown 10
//#define KdzDOWN_loc 0
//
//#define Kpy_tra 1000// the Kp of foot coordinate y
//#define Kdy_tra 100 // the Kd of foot coordinate y
//#define Kpy_loc 10000
//#define Kdy_loc 1000
//
//#define Kpxi 6// the Kp of foot coordinate x
//#define Kdxi 0 // the Kd of foot coordinate x
//#define Kpyi 1 // the Kp of foot coordinate y
//#define Kdyi 0 // the Kd of foot coordinate y
//#define Impact_KpzUP 3000// the Kp of foot coordinate z, consistent
//#define KdzUPi 0// the Kd of foot coordinate z
//
//// trick ground impulse parameters
//#define Kpxip 0// the Kp of foot coordinate x
//#define Kdxip 0 // the Kd of foot coordinate x
//#define Kpyip 1 // the Kp of foot coordinate y
//#define Kdyip 0 // the Kd of foot coordinate y
//#define KpzUPip 3000// the Kp of foot coordinate z, consistent
//#define KdzUPip 100// the Kd of foot coordinate z
//#define THRESH 0.3 // the up and down tag
//#define IMPULSE_TIME 3 // the impulse push time
//#define SLOPE -0.1 // redesign the down slope of the com z
// trick parameters
//#define RECOVER_PARA 4
//#define VXRECOVER 1 // 0.01 when the vcomx is lower than the desired vcomx
//#define COMZ_RECOVER 3
//#define COMZ_DIFF 1 // 0.01 WHEN THE comz is higher than desired comz
//#define COMZM_RECOVER 30
//#define COMZM_DIFF 2 // when the comz is lower than the desired comz
//#define FOOTZ_RECOVER 10 // when the foot is higher than the offset
//#define ZH_OFFSET 1

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
            this->DeclareVectorOutputPort(systems::BasicVector<double>(37),
                                          &qpController::COM_ToeOutPut);

            // to judge the runing period;
            // period_state_
            // 2 means regular running
            // 1 means reaches a new piece of trajectory
            // 0 means phase space x corrupted
            this->DeclareAbstractState(AbstractValue::Make(period_state_));
            this->DeclareAbstractState(AbstractValue::Make(sim_time));
            this->DeclareAbstractState(AbstractValue::Make(t));
            this->DeclareAbstractState(AbstractValue::Make(t_pre_));
            this->DeclareAbstractState(AbstractValue::Make(m_));
            this->DeclareAbstractState(AbstractValue::Make(count_num_));
            this->DeclareAbstractState(AbstractValue::Make(count_num_change_));
            this->DeclareAbstractState(AbstractValue::Make(step_num_));
            this->DeclareAbstractState(AbstractValue::Make(replanning_));
            this->DeclareAbstractState(AbstractValue::Make(isNew_));
            this->DeclareAbstractState(AbstractValue::Make(stance_now_));
            this->DeclareAbstractState(AbstractValue::Make(location_bias_));
            this->DeclareAbstractState(AbstractValue::Make(modified_));
            this->DeclareAbstractState(AbstractValue::Make(incline_));
            this->DeclareAbstractState(AbstractValue::Make(refreshed_));
            this->DeclareAbstractState(AbstractValue::Make(lateral_incline_));
            this->DeclareAbstractState(AbstractValue::Make(lateral_bias_));

            this->DeclareDiscreteState(NU);

            this->DeclarePeriodicDiscreteUpdate(0.0005);
            this->DeclarePeriodicUnrestrictedUpdateEvent(0.0005, 0, &qpController::AbstractUpdate);

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
            std::ifstream ifstr_data("examples/KneedCompassGait/data.txt");
            double temp_data;
            std::vector<std::vector<double>> mat;
            std::vector<double> one_row(6,0);
            for (int j = 0; !ifstr_data.eof() ; ++j) {
                for (int i = 0; i < 6; ++i) {
                    ifstr_data >> temp_data;
                    one_row[i] = temp_data;
                }
                mat.push_back(one_row);
            }
            ifstr_data.close();

            std::ifstream ifstr_data2("examples/KneedCompassGait/data2.txt");
            std::vector<std::vector<double>> mat2;
            std::vector<double> one_row2(8,0);

            for (int j = 0; !ifstr_data2.eof() ; ++j) {
                for (int i = 0; i < 8; ++i) {
                    ifstr_data2 >> temp_data;
                    one_row2[i] = temp_data;
                }
                mat2.push_back(one_row2);
            }
            ifstr_data2.close();

            std::ifstream ifstr_data3("examples/KneedCompassGait/data3.txt");
            std::vector<std::vector<double>> mat3;
            std::vector<double> one_row3(6,0);

            for (int j = 0; !ifstr_data3.eof() ; ++j) {
                for (int i = 0; i < 6; ++i) {
                    ifstr_data3 >> temp_data;
                    one_row3[i] = temp_data;
                }
                mat3.push_back(one_row3);
            }
            ifstr_data3.close();

            this->trajectory_ = mat;
            this->foot_step_ = mat2;
            this->foot_trajectory_ = mat3;

            // add decision variables--------------------------------------------------------------------
            auto vdot = prog.NewContinuousVariables(NV, "joint acceleration"); // qddot var
            auto u = prog.NewContinuousVariables(NU, "input");  // torque var
            auto beta = prog.NewContinuousVariables(NF, "friction forces"); // contact force var
            auto eps = prog.NewContinuousVariables(2, "slack variables"); // 2 knee slack variables
            auto yita_st = prog.NewContinuousVariables(3, "stance leg slack"); // 3 stance leg slack variables
            auto yita_sw = prog.NewContinuousVariables(3, "swing leg slack"); // 3 swing leg slack variables

            // Problem Constrains -----------------------------------------------------------------------
            auto con_u_lim = prog.AddBoundingBoxConstraint(umin,umax, u).evaluator();
            auto con_fric_lim = prog.AddBoundingBoxConstraint(5, 1000000, beta).evaluator();

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
            this->con_slack2_ = con_slack2;
            this->con_slack3_ = con_slack3;
            this->con_dynf_ = con_dynf;
            this->con_dyna_ = con_dyna;
            this->con_fric_lim_ = con_fric_lim;
            this->cost_xddot_ = cost_xddot;
            this->cost_t_ = cost_t;
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
            t = context.get_time();

            cout << "stance Now:" << stance_now_   <<
                 "\tsim_tim:" << sim_time << "\tt" << t << "\tstep_num:" <<
                 step_num_   << "\tperiod_state:" << period_state_ <<
                 "\tm:" << m_ << "\tlocation bias:" << location_bias_  << "\tlateral incline" << lateral_bias_ << endl;

            if (sim_time <= t-h) {
            VectorX<double> x = this->EvalVectorInput(context, 0)->CopyToVector();
            static Eigen::Matrix<double, NU, 1> t_pre = t_pre_;
            t_pre.setZero();
            static int m=m_;
            static int count_num = count_num_;
            static bool count_num_change = count_num_change_;
            static int step_num = step_num_;
            if (semophore_) {
                m = m_;
                count_num = count_num_;count_num_change = count_num_change_;step_num = step_num_;t_pre = t_pre_;
                incline_ = 0; lateral_incline_ = 0;
            }

          // get the current state and current disired trajectory
            VectorX<double> xcom_des(COM), foot_traj_des(COM);
            VectorX<double> q(NQ), qd(NV);
            VectorX<double> this_stance(3),next_stance(3);
            for (int i = 0; i < NQ; ++i) {
                q[i] = x[i];
                qd[i] = x[i+NQ];
            }

            this->left_is_stance = foot_step_[m][6];

            static bool replanning = replanning_;
            static bool isNew = isNew_;
            static double stance_now = stance_now_;
            if (semophore_) {
                replanning = replanning_;isNew = isNew_;stance_now = stance_now_;
                semophore_ = false;
            }
            static int impulse_push = IMPULSE_TIME;

            stance_now = foot_step_[m][0]; //-foot_step_error;
            if (replanning) {
                while(!isNew) {// firstly skip to the switching point
                    this_stance[0] = foot_step_[m][0]; //-foot_step_error;
                    if (stance_now == this_stance[0])
                        isNew = false;
                    else{
                        isNew = true;
                        step_num += 1;
                    }
                    stance_now = this_stance[0];
                    ++m;
                }
            }
            replanning = false;

            for (int l = 0; l < COM; ++l) {
                if (l < COM){
                    xcom_des[l] = trajectory_[m][l];
                    foot_traj_des[l] = foot_trajectory_[m][l];
                }
                if (l < 3){
                    this_stance[l] = foot_step_[m][l];
                    next_stance[l] = foot_step_[m][l+3];
                }
            }
            this->left_is_stance = foot_step_[m++][6];

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

            double rdis_x = next_stance[0] - right_toe_pos[0];
            double rdis_y = next_stance[1] - right_toe_pos[1];
            double ldis_x = next_stance[0] - left_toe_pos[0];
            double ldis_y = next_stance[1] - left_toe_pos[1];

            rdis_x = rdis_x - location_bias_;
            ldis_x = ldis_x - location_bias_;
            rdis_y = rdis_y + lateral_bias_ - 0.1;
            ldis_y = ldis_y + lateral_bias_ - 0.1;

            // Problem Constraints ----------------------------------------------------------------------
            Eigen::Matrix<double, NF, 1> l_contact;
            Eigen::Matrix<double, NF, 1> u_contact;
            l_contact.setOnes();u_contact.setOnes();
            l_contact *= 5;
            u_contact *= 1e6;
            int count=2; // if count == 1; then is swing and stance phase; if count == 2, then is impact phase
            for (int i = 0; i < NC; ++i) {
                if(phi[i] > contact_phi){
                    count--;
                    for (int j = 0; j < ND; ++j) {
                        l_contact(4*i+j,0) = 0;
                        u_contact(4*i+j,0) = 0;
                    }
                }
            }
            this->con_fric_lim_->set_bounds(l_contact, u_contact);
            Eigen::Vector2d beq_slack;
            beq_slack.setZero();

            Eigen::Matrix<double, NQ-NU, NQ+NF> dyn_conf;
            dyn_conf.setZero();
            dyn_conf << Hf, -JBf;
            this->con_dynf_->UpdateCoefficients(dyn_conf, -Cf);

            Eigen::Matrix<double, NU, NQ+NU+NF> dyn_cona;
            dyn_cona.setZero();
            dyn_cona << Ha, -Ba, -JBa;
            this->con_dyna_->UpdateCoefficients(dyn_cona, -Ca);

            // Problem Costs ---------------------------------------------------------------------------
            Eigen::Matrix<double, COM/2, 1> kp = this->Kp_xdd_;
            Eigen::Matrix<double, COM/2, 1> kd = this->Kd_xdd_;

            auto xddot_des = kp.cwiseProduct(xcom_des.segment(0, 3)-com) +
                    kd.cwiseProduct(xcom_des.segment(3,3)-vcom);
            Eigen::Matrix<double, 3, 3> wxddot;
            wxddot << W1, 0, 0,
                        0, W1y, 0,
                        0, 0, W1;

            auto Q_xdd = 2*Jcom.transpose()*wxddot*Jcom;
            auto b_xdd = 2*Jcom.transpose()*wxddot*(Jcomdot_times_v-xddot_des);
            this->cost_xddot_->UpdateCoefficients(Q_xdd, b_xdd);


            Eigen::Matrix<double, NU, NU> Q_t = 2*this->weight_t_;
            Eigen::Matrix<double, NU, 1> b_t = 2*this->weight_t_.transpose()*(-t_pre);
            if (count == 2){ Q_t *= 0; b_t *= 0;}
            this->cost_t_->UpdateCoefficients(Q_t, b_t);

            Eigen::Matrix<double, 2, 2> Q_slack = 2*this->weight_slack_;
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
            double xlddot, ylddot, zlddot;

            // a judgement, when the toe is too high, give it a higher desired acceleartion towards ground
            double KpzTraj;
            KpzTraj = Kpz_tra;
            double KdzTraj;
            KdzTraj = Kdz_tra;

            // a judgement, when it is the first time in a piece of trajectory to hit the ground, give it a
            // time impulse push towards the ground
            double kpxi, kdxi, kpyi, kdyi, kpzi, kdzi;
            kpxi = Kpxi;
            kdxi = Kdxi;
            kpyi = Kpyi;
            kdyi = Kdyi;
            kpzi = KpzUPi;
            kdzi = KdzUPi;
            if (count == 2) {
                if (step_num%6 == 0) {
                    kpzi = KpzUPd;
                }
                if (impulse_push) {
                    kpxi = Kpxip;
                    kdxi = Kdxip;
                    kpyi = Kpyip;
                    kdyi = Kdyip;
                    kpzi = KpzUPip;
                    kdzi = KdzUPip;
                    --impulse_push;
                }
            }

            double location_Kpzdown = KpzDOWN_loc;

            period_state_ = update_automata(isNew, step_num, -vcom(0,0), com(1,0));

            if (isNew){
                if (left_is_stance){    // if this trajectory is new and left is the stance leg then is swing phase
                    count_num = count;
                    // stance leg constraint
                    std::cout << "=======left is stance===1=====" << std::endl;
                    JStance << left_toe_jaco;
                    JdotqdotStance << left_toe_jacodotv;
                    JqdotStance << ALPHA*left_toe_Jqdot;
                    beq_slack2 << -JdotqdotStance-JqdotStance;
                    Aeq_slack2 << JStance, -I;
                    this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                    // stance leg cost
                    Q_slack2.setIdentity();
                    Q_slack2 *= W5;
                    this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                    // swing leg constraint

                    // the trajectory part
                    xddot = Kpx_tra*(foot_traj_des[0] - right_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - right_toe_Jqdot[0]);
                    yddot = Kpy_tra*(foot_traj_des[1] - right_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - right_toe_Jqdot[1]);
                    zddot = KpzTraj*(foot_traj_des[2] - right_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - right_toe_Jqdot[2]);

                    // the foot location part
                    xlddot = Kpx_loc*rdis_x-Kdx_loc*right_toe_Jqdot[0];
                    ylddot = Kpy_loc*rdis_y-Kdy_loc*right_toe_Jqdot[1];
                    if (fabs(rdis_x) + fabs(rdis_y) >= THRESH){
                        zlddot = KpzUP_loc * (ZH - right_toe_pos[2]) - KdzUP_loc * right_toe_Jqdot[2];
                        Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                    }
                    else{
                        zlddot = -location_Kpzdown*right_toe_pos[2] - KdzDOWN_loc*right_toe_Jqdot[2];
                        if (phi[1] < contact_phi)
                            Xddot << 0, 0, zddot;
                        else
                            Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
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

                } else {    // if the trajectory is new and right is the stance leg then to swing phase
                    count_num = count;
                    // stance leg constraint
                    cout << "======right is stance===2=====" << endl;
                    JStance << right_toe_jaco;
                    JdotqdotStance << right_toe_jacodotv;
                    JqdotStance << ALPHA*right_toe_Jqdot;
                    beq_slack2 << -JdotqdotStance-JqdotStance;
                    Aeq_slack2 << JStance, -I;
                    this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                    // stance leg cost
                    Q_slack2.setIdentity();
                    Q_slack2 *= W5;
                    this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                    // swing leg constraint
                    xddot = Kpx_tra*(foot_traj_des[0] - left_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - left_toe_Jqdot[0]);
                    yddot = Kpy_tra*(foot_traj_des[1] - left_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - left_toe_Jqdot[1]);
                    zddot = KpzTraj*(foot_traj_des[2] - left_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - left_toe_Jqdot[2]);

                    xlddot = Kpx_loc*ldis_x-Kdx_loc*left_toe_Jqdot[0];
                    ylddot = Kpy_loc*ldis_y-Kdy_loc*left_toe_Jqdot[1];
                    if (fabs(ldis_x)+fabs(ldis_y) >= THRESH) {
                        zlddot = KpzUP_loc * (ZH - left_toe_pos[2]) - KdzUP_loc * left_toe_Jqdot[2];
                        Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                    }
                    else{
                        zlddot = -location_Kpzdown*left_toe_pos[2] - KdzDOWN_loc*left_toe_Jqdot[2];
                        if (phi[0] < contact_phi)
                            Xddot << 0, 0, zddot;
                        else
                            Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
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
                impulse_push = IMPULSE_TIME;
                count_num_change = false;
                isNew = false;
       //         comz_is_greater = false;
            } else {
                if (count_num == count) {
                    if (count_num_change) {
                        if (left_is_stance && count==1) {
                            // stance leg constraint
                            replanning = true;
//                            foot_step_error += next_stance[0] - right_toe_pos[0];
                            std::cout << "=======left is stance====3====" << std::endl;
                            JStance << left_toe_jaco;
                            JdotqdotStance << left_toe_jacodotv;
                            JqdotStance << ALPHA*left_toe_Jqdot;
                            beq_slack2 << -JdotqdotStance-JqdotStance;
                            Aeq_slack2 << JStance, -I;
                            this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                            // stance leg cost
                            Q_slack2.setIdentity();
                            Q_slack2 *= W5;
                            this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                            // swing leg constraint

                            xddot = Kpx_tra*(foot_traj_des[0] - right_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - right_toe_Jqdot[0]);
                            yddot = Kpy_tra*(foot_traj_des[1] - right_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - right_toe_Jqdot[1]);
                            zddot = KpzTraj*(foot_traj_des[2] - right_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - right_toe_Jqdot[2]);

                            // the foot location part
                            xlddot = Kpx_loc*rdis_x-Kdx_loc*right_toe_Jqdot[0];
                            ylddot = Kpy_loc*rdis_y-Kdy_loc*right_toe_Jqdot[1];
                            if (fabs(rdis_x) + fabs(rdis_y) >= THRESH){
                                zlddot = KpzUP_loc * (ZH - right_toe_pos[2]) - KdzUP_loc * right_toe_Jqdot[2];
                                Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
                            else{
                                zlddot = -location_Kpzdown*right_toe_pos[2] - KdzDOWN_loc*right_toe_Jqdot[2];
                                if (phi[1] < contact_phi)
                                    Xddot << 0, 0, zddot;
                                else
                                    Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
//                            xddot = Kpx_tra*rdis_x-Kdx_tra*right_toe_Jqdot[0];
//                            yddot = Kpy_tra*rdis_y-Kdy_tra*right_toe_Jqdot[1];
//                            if (fabs(rdis_x) + fabs(rdis_y) >= THRESH){
//                                zddot = KpzUP_loc * (ZH - right_toe_pos[2]) - KdzUP_loc * right_toe_Jqdot[2];
//                                Xddot << xddot, yddot, zddot;
//                            }
//                            else{
//                                zddot = -KpzTraj*right_toe_pos[2] - KdzDOWN_loc*right_toe_Jqdot[2];
//                                if (phi[1] < contact_phi)
//                                    Xddot << 0, 0, zddot;
//                                else
//                                    Xddot << xddot, yddot, zddot;
//                            }
                            JSwing << right_toe_jaco;
                            JdotqdotSwing << right_toe_jacodotv;
                            Aeq_slack3 << JSwing, -I;
                            beq_slack3 << Xddot-JdotqdotSwing;
                            this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                            // swing leg cost
                            Q_slack3.setIdentity();
                            Q_slack3 *= W6;
                            this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                            impulse_push = IMPULSE_TIME;
                        } else if (left_is_stance && count==2) { // left stance impact phase
                            // stance leg constraint
                            cout << "======left is stance==impact pahse===4===" << endl;
                            JStance << right_toe_jaco;
                            JdotqdotStance << right_toe_jacodotv;
                            JqdotStance << ALPHA*right_toe_Jqdot;
                            beq_slack2 << -JdotqdotStance-JqdotStance;
                            Aeq_slack2 << JStance, -I;
                            this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                            // stance leg cost
                            Q_slack2.setIdentity();
                            Q_slack2 *= W5i;
                            this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                            // swing leg constraint


                            xddot = kpxi*ldis_x-kdxi*left_toe_Jqdot[0];
                            yddot = kpyi*ldis_y-kdyi*left_toe_Jqdot[1];
                            zddot = kpzi * (ZH - left_toe_pos[2]) - kdzi * left_toe_Jqdot[2];
                            Xddot << xddot, yddot, zddot;
                            JSwing << left_toe_jaco;
                            JdotqdotSwing << left_toe_jacodotv;
                            Aeq_slack3 << JSwing, -I;
                            beq_slack3 << Xddot-JdotqdotSwing;
                            this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                            // swing leg cost
                            Q_slack3.setIdentity();
                            Q_slack3 *= W6i;
                            this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                        } else if (!left_is_stance && count==1) {
                            // stance leg constraint
                            replanning = true;
//                            foot_step_error += next_stance[0] - left_toe_pos[0];
                            cout << "======right is stance===5=====" << endl;
                            JStance << right_toe_jaco;
                            JdotqdotStance << right_toe_jacodotv;
                            JqdotStance << ALPHA*right_toe_Jqdot;
                            beq_slack2 << -JdotqdotStance-JqdotStance;
                            Aeq_slack2 << JStance, -I;
                            this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                            // stance leg cost/**/
                            Q_slack2.setIdentity();
                            Q_slack2 *= W5;
                            this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                            // swing leg constraint

                            xddot = Kpx_tra*(foot_traj_des[0] - left_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - left_toe_Jqdot[0]);
                            yddot = Kpy_tra*(foot_traj_des[1] - left_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - left_toe_Jqdot[1]);
                            zddot = KpzTraj*(foot_traj_des[2] - left_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - left_toe_Jqdot[2]);

                            xlddot = Kpx_loc*ldis_x-Kdx_loc*left_toe_Jqdot[0];
                            ylddot = Kpy_loc*ldis_y-Kdy_loc*left_toe_Jqdot[1];
                            if (fabs(ldis_x)+fabs(ldis_y) >= THRESH) {
                                zlddot = KpzUP_loc * (ZH - left_toe_pos[2]) - KdzUP_loc * left_toe_Jqdot[2];
                                Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
                            else{
                                zlddot = -location_Kpzdown*left_toe_pos[2] - KdzDOWN_loc*left_toe_Jqdot[2];
                                if (phi[0] < contact_phi)
                                    Xddot << 0, 0, zddot;
                                else
                                    Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
//                            xddot = Kpx_tra*ldis_x-Kdx_tra*left_toe_Jqdot[0];
//                            yddot = Kpy_tra*ldis_y-Kdy_tra*left_toe_Jqdot[1];
//                            if (fabs(ldis_x)+fabs(ldis_y) >= THRESH) {
//                                zddot = KpzUP_loc * (ZH - left_toe_pos[2]) - KdzUP_loc * left_toe_Jqdot[2];
//                                Xddot << xddot, yddot, zddot;
//                            }
//                            else{
//                                zddot = -KpzTraj*left_toe_pos[2] - KdzDOWN_loc*left_toe_Jqdot[2];
//                                if (phi[0] < contact_phi)
//                                    Xddot << 0, 0, zddot;
//                                else
//                                    Xddot << xddot, yddot, zddot;
//                            }
                            JSwing << left_toe_jaco;
                            JdotqdotSwing << left_toe_jacodotv;
                            Aeq_slack3 << JSwing, -I;
                            beq_slack3 << Xddot-JdotqdotSwing;
                            this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                            // swing leg cost
                            Q_slack3.setIdentity();
                            Q_slack3 *= W6;
                            this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                            impulse_push = IMPULSE_TIME;
                        } else { // right stance impact phase
                            // stance leg constraint/**/
                            std::cout << "=======right is stance==impact phase==6====" << std::endl;
                            JStance << left_toe_jaco;
                            JdotqdotStance << left_toe_jacodotv;
                            JqdotStance << ALPHA*left_toe_Jqdot;
                            beq_slack2 << -JdotqdotStance-JqdotStance;
                            Aeq_slack2 << JStance, -I;
                            this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                            // stance leg cost
                            Q_slack2.setIdentity();
                            Q_slack2 *= W5i;
                            this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                            // swing leg constraint


                            xddot = kpxi*rdis_x-kdxi*right_toe_Jqdot[0];
                            yddot = kpyi*rdis_y-kdyi*right_toe_Jqdot[1];
                            zddot = kpzi * (ZH - right_toe_pos[2]) - kdzi * right_toe_Jqdot[2];
                            Xddot << xddot, yddot, zddot;
                            JSwing << right_toe_jaco;
                            JdotqdotSwing << right_toe_jacodotv;
                            Aeq_slack3 << JSwing, -I;
                            beq_slack3 << Xddot-JdotqdotSwing;
                            this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                            // swing leg cost
                            Q_slack3.setIdentity();
                            Q_slack3 *= W6i;
                            this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                        }
                    } else {
                        if (left_is_stance) {  // if left is the stance leg, then swing
                            // stance leg constraint
                            std::cout << "=======left is stance===7=====" << std::endl;
                            JStance << left_toe_jaco;
                            JdotqdotStance << left_toe_jacodotv;
                            JqdotStance << ALPHA*left_toe_Jqdot;
                            beq_slack2 << -JdotqdotStance-JqdotStance;
                            Aeq_slack2 << JStance, -I;
                            this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                            // stance leg cost
                            Q_slack2.setIdentity();
                            Q_slack2 *= W5;
                            this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                            // swing leg constraint

                            xddot = Kpx_tra*(foot_traj_des[0] - right_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - right_toe_Jqdot[0]);
                            yddot = Kpy_tra*(foot_traj_des[1] - right_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - right_toe_Jqdot[1]);
                            zddot = KpzTraj*(foot_traj_des[2] - right_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - right_toe_Jqdot[2]);

                            xlddot = Kpx_loc*rdis_x-Kdx_loc*right_toe_Jqdot[0];
                            ylddot = Kpy_loc*rdis_y-Kdy_loc*right_toe_Jqdot[1];
                            if (fabs(rdis_x) + fabs(rdis_y) >= THRESH){
                                zlddot = KpzUP_loc * (ZH - right_toe_pos[2]) - KdzUP_loc * right_toe_Jqdot[2];
                                Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
                            else{
                                zlddot = -location_Kpzdown*right_toe_pos[2] - KdzDOWN_loc*right_toe_Jqdot[2];
                                if (phi[1] < contact_phi)
                                    Xddot << 0, 0, zddot;
                                else
                                    Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
//                            xddot = Kpx_tra*rdis_x-Kdx_tra*right_toe_Jqdot[0];
//                            yddot = Kpy_tra*rdis_y-Kdy_tra*right_toe_Jqdot[1];
//                            if (fabs(rdis_x) + fabs(rdis_y) >= THRESH){
//                                zddot = KpzUP_loc * (ZH - right_toe_pos[2]) - KdzUP_loc * right_toe_Jqdot[2];
//                                Xddot << xddot, yddot, zddot;
//                            }
//                            else{
//                                zddot = -KpzTraj*right_toe_pos[2] - KdzDOWN_loc*right_toe_Jqdot[2];
//                                if (phi[1] < contact_phi)
//                                    Xddot << 0, 0, zddot;
//                                else
//                                    Xddot << xddot, yddot, zddot;
//                            }
                            JSwing << right_toe_jaco;
                            JdotqdotSwing << right_toe_jacodotv;
                            Aeq_slack3 << JSwing, -I;
                            beq_slack3 << Xddot-JdotqdotSwing;
                            this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                            // swing leg cost
                            Q_slack3.setIdentity();
                            Q_slack3 *= W6;
                            this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);


                        } else {    // if right is the stance leg, then swing
                            // stance leg constraint
                            cout << "======right is stance====8====" << endl;
                            JStance << right_toe_jaco;
                            JdotqdotStance << right_toe_jacodotv;
                            JqdotStance << ALPHA*right_toe_Jqdot;
                            beq_slack2 << -JdotqdotStance-JqdotStance;
                            Aeq_slack2 << JStance, -I;
                            this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                            // stance leg cost
                            Q_slack2.setIdentity();
                            Q_slack2 *= W5;
                            this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                            // swing leg constraint

                            xddot = Kpx_tra*(foot_traj_des[0] - left_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - left_toe_Jqdot[0]);
                            yddot = Kpy_tra*(foot_traj_des[1] - left_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - left_toe_Jqdot[1]);
                            zddot = KpzTraj*(foot_traj_des[2] - left_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - left_toe_Jqdot[2]);

                            xlddot = Kpx_loc*ldis_x-Kdx_loc*left_toe_Jqdot[0];
                            ylddot = Kpy_loc*ldis_y-Kdy_loc*left_toe_Jqdot[1];
                            if (fabs(ldis_x)+fabs(ldis_y) >= THRESH) {
                                zlddot = KpzUP_loc * (ZH - left_toe_pos[2]) - KdzUP_loc * left_toe_Jqdot[2];
                                Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
                            else{
                                zlddot = -KpzTraj*left_toe_pos[2] - KdzDOWN_loc*left_toe_Jqdot[2];
                                if (phi[0] < contact_phi)
                                    Xddot << 0, 0, zddot;
                                else
                                    Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                            }
//                            xddot = Kpx_tra*ldis_x-Kdx_tra*left_toe_Jqdot[0];
//                            yddot = Kpy_tra*ldis_y-Kdy_tra*left_toe_Jqdot[1];
//                            if (fabs(ldis_x)+fabs(ldis_y) >= THRESH) {
//                                zddot = KpzUP_loc * (ZH - left_toe_pos[2]) - KdzUP_loc * left_toe_Jqdot[2];
//                                Xddot << xddot, yddot, zddot;
//                            }
//                            else{
//                                zddot = -KpzTraj*left_toe_pos[2] - KdzDOWN_loc*left_toe_Jqdot[2];
//                                if (phi[0] < contact_phi)
//                                    Xddot << 0, 0, zddot;
//                                else
//                                    Xddot << xddot, yddot, zddot;
//                            }
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
                        impulse_push = IMPULSE_TIME;
                    }
                } else {
                    count_num_change = true;
                    if (left_is_stance && count==1) {
                        // stance leg constraint
                        std::cout << "=======left is stance===9=====" << std::endl;
                        JStance << left_toe_jaco;
                        JdotqdotStance << left_toe_jacodotv;
                        JqdotStance << ALPHA*left_toe_Jqdot;
                        beq_slack2 << -JdotqdotStance-JqdotStance;
                        Aeq_slack2 << JStance, -I;
                        this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                        // stance leg cost
                        Q_slack2.setIdentity();
                        Q_slack2 *= W5;
                        this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                        // swing leg constraint

                        xddot = Kpx_tra*(foot_traj_des[0] - right_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - right_toe_Jqdot[0]);
                        yddot = Kpy_tra*(foot_traj_des[1] - right_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - right_toe_Jqdot[1]);
                        zddot = KpzTraj*(foot_traj_des[2] - right_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - right_toe_Jqdot[2]);

                        xlddot = Kpx_loc*rdis_x-Kdx_loc*right_toe_Jqdot[0];
                        ylddot = Kpy_loc*rdis_y-Kdy_loc*right_toe_Jqdot[1];
                        if (fabs(rdis_x) + fabs(rdis_y) >= THRESH){
                            zlddot = KpzUP_loc * (ZH - right_toe_pos[2]) - KdzUP_loc * right_toe_Jqdot[2];
                            Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                        }
                        else{
                            zlddot = -location_Kpzdown*right_toe_pos[2] - KdzDOWN_loc*right_toe_Jqdot[2];
                            if (phi[1] < contact_phi)
                                Xddot << 0, 0, zddot;
                            else
                                Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                        }
//                        xddot = Kpx_tra*rdis_x-Kdx_tra*right_toe_Jqdot[0];
//                        yddot = Kpy_tra*rdis_y-Kdy_tra*right_toe_Jqdot[1];
//                        if (fabs(rdis_x) + fabs(rdis_y) >= THRESH){
//                            zddot = KpzUP_loc * (ZH - right_toe_pos[2]) - KdzUP_loc * right_toe_Jqdot[2];
//                            Xddot << xddot, yddot, zddot;
//                        }
//                        else{
//                            zddot = -KpzTraj*right_toe_pos[2] - KdzDOWN_loc*right_toe_Jqdot[2];
//                            if (phi[1] < contact_phi)
//                                Xddot << 0, 0, zddot;
//                            else
//                                Xddot << xddot, yddot, zddot;
//                        }
                        JSwing << right_toe_jaco;
                        JdotqdotSwing << right_toe_jacodotv;
                        Aeq_slack3 << JSwing, -I;
                        beq_slack3 << Xddot-JdotqdotSwing;
                        this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                        // swing leg cost
                        Q_slack3.setIdentity();
                        Q_slack3 *= W6;
                        this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                        impulse_push = IMPULSE_TIME;
                    } else if (left_is_stance && count==2) { // left stance impact phase
                        // stance leg constraint
                        cout << "======left is stance==impact pahse====10==" << endl;
                        JStance << right_toe_jaco;
                        JdotqdotStance << right_toe_jacodotv;
                        JqdotStance << ALPHA*right_toe_Jqdot;
                        beq_slack2 << -JdotqdotStance-JqdotStance;
                        Aeq_slack2 << JStance, -I;
                        this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                        // stance leg cost
                        Q_slack2.setIdentity();
                        Q_slack2 *= W5i;
                        this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                        // swing leg constraint
                        xddot = kpxi*ldis_x-kdxi*left_toe_Jqdot[0];
                        yddot = kpyi*ldis_y-kdyi*left_toe_Jqdot[1];
                        zddot = kpzi * (ZH - left_toe_pos[2]) - kdzi * left_toe_Jqdot[2];
                        Xddot << xddot, yddot, zddot;
                        JSwing << left_toe_jaco;
                        JdotqdotSwing << left_toe_jacodotv;
                        Aeq_slack3 << JSwing, -I;
                        beq_slack3 << Xddot-JdotqdotSwing;
                        this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                        // swing leg cost
                        Q_slack3.setIdentity();
                        Q_slack3 *= W6i;
                        this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                    } else if (!left_is_stance && count==1) {
                        // stance leg constraint
                        cout << "======right is stance====11====" << endl;
                        JStance << right_toe_jaco;
                        JdotqdotStance << right_toe_jacodotv;
                        JqdotStance << ALPHA*right_toe_Jqdot;
                        beq_slack2 << -JdotqdotStance-JqdotStance;
                        Aeq_slack2 << JStance, -I;
                        this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                        // stance leg cost
                        Q_slack2.setIdentity();
                        Q_slack2 *= W5;
                        this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                        // swing leg constraint

                        xddot = Kpx_tra*(foot_traj_des[0] - left_toe_pos[0]) + Kdx_tra*(foot_traj_des[3] - left_toe_Jqdot[0]);
                        yddot = Kpy_tra*(foot_traj_des[1] - left_toe_pos[1]) + Kdy_tra*(foot_traj_des[4] - left_toe_Jqdot[1]);
                        zddot = KpzTraj*(foot_traj_des[2] - left_toe_pos[2]) + KdzTraj*(foot_traj_des[5] - left_toe_Jqdot[2]);

                        xlddot = Kpx_loc*ldis_x-Kdx_loc*left_toe_Jqdot[0];
                        ylddot = Kpy_loc*ldis_y-Kdy_loc*left_toe_Jqdot[1];
                        if (fabs(ldis_x)+fabs(ldis_y) >= THRESH) {
                            zlddot = KpzUP_loc * (ZH - left_toe_pos[2]) - KdzUP_loc * left_toe_Jqdot[2];
                            Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                        }
                        else{
                            zlddot = -location_Kpzdown*left_toe_pos[2] - KdzDOWN_loc*left_toe_Jqdot[2];
                            if (phi[0] < contact_phi)
                                Xddot << 0, 0, zddot;
                            else
                                Xddot << xddot+xlddot, yddot+ylddot, zddot+zlddot;
                        }
//                        xddot = Kpx_tra*ldis_x-Kdx_tra*left_toe_Jqdot[0];
//                        yddot = Kpy_tra*ldis_y-Kdy_tra*left_toe_Jqdot[1];
//                        if (fabs(ldis_x)+fabs(ldis_y) >= THRESH) {
//                            zddot = KpzUP_loc * (ZH - left_toe_pos[2]) - KdzUP_loc * left_toe_Jqdot[2];
//                            Xddot << xddot, yddot, zddot;
//                        }
//                        else{
//                            zddot = -KpzTraj*left_toe_pos[2] - KdzDOWN_loc*left_toe_Jqdot[2];
//                            if (phi[0] < contact_phi)
//                                Xddot << 0, 0, zddot;
//                            else
//                                Xddot << xddot, yddot, zddot;
//                        }
                        JSwing << left_toe_jaco;
                        JdotqdotSwing << left_toe_jacodotv;
                        Aeq_slack3 << JSwing, -I;
                        beq_slack3 << Xddot-JdotqdotSwing;
                        this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                        // swing leg cost
                        Q_slack3.setIdentity();
                        Q_slack3 *= W6;
                        this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                        impulse_push = IMPULSE_TIME;
                    } else { // right stance impact phase
                        cout << "left is stance :" << left_is_stance << "\tcount:" << count << endl;
                        // stance leg constraint
                        std::cout << "=======right is stance==impact phase==12====" << std::endl;
                        JStance << left_toe_jaco;
                        JdotqdotStance << left_toe_jacodotv;
                        JqdotStance << ALPHA * left_toe_Jqdot;
                        beq_slack2 << -JdotqdotStance-JqdotStance;
                        Aeq_slack2 << JStance, -I;
                        this->con_slack2_->UpdateCoefficients(Aeq_slack2, beq_slack2);

                        // stance leg cost
                        Q_slack2.setIdentity();
                        Q_slack2 *= W5i;
                        this->cost_slack2_->UpdateCoefficients(Q_slack2, b_slack2);

                        // swing leg constraint


                        xddot = kpxi*rdis_x-kdxi*right_toe_Jqdot[0];
                        yddot = kpyi*rdis_y-kdyi*right_toe_Jqdot[1];
                        zddot = kpzi * (ZH - right_toe_pos[2]) - kdzi * right_toe_Jqdot[2];
                        Xddot << xddot, yddot, zddot;
                        JSwing << right_toe_jaco;
                        JdotqdotSwing << right_toe_jacodotv;
                        Aeq_slack3 << JSwing, -I;
                        beq_slack3 << Xddot-JdotqdotSwing;
                        this->con_slack3_->UpdateCoefficients(Aeq_slack3, beq_slack3);

                        // swing leg cost
                        Q_slack3.setIdentity();
                        Q_slack3 *= W6i;
                        this->cost_slack3_->UpdateCoefficients(Q_slack3, b_slack3);

                    }
                }
            }

            // solve the problem --------------------------------------------------------------------------
            drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(this->prog);
            auto result_vec = result.GetSolution();
            Eigen::Matrix<double, NU, 1> u(result_vec.middleRows(NQ, NU));

            Eigen::Matrix<double, 3, 1> bias_mass(0,0,0);
            auto left_lowleg_mass = rtree_->transformPoints(kinsol, bias_mass, 8, 0);
            auto left_upleg_mass = rtree_->transformPoints(kinsol, bias_mass, 6, 0);
            auto right_lowleg_mass = rtree_->transformPoints(kinsol, bias_mass, 12, 0);
            auto right_upleg_mass = rtree_->transformPoints(kinsol, bias_mass, 10, 0);
            auto hip_mass = rtree_->transformPoints(kinsol, bias_mass, 2, 0);
            auto llowleg_v = rtree_->transformPointsJacobian(kinsol, bias_mass, 8, 0, true)*qd;
            auto luppleg_v = rtree_->transformPointsJacobian(kinsol, bias_mass, 6, 0, true)*qd;
            auto rlowleg_v = rtree_->transformPointsJacobian(kinsol, bias_mass, 12, 0, true)*qd;
            auto ruppleg_v = rtree_->transformPointsJacobian(kinsol, bias_mass, 10, 0, true)*qd;
            auto hip_v = rtree_->transformPointsJacobian(kinsol, bias_mass, 2, 0, true)*qd;

            auto K = qd.transpose()*H*qd/2;
            auto V = 2.5*9.81*(left_lowleg_mass(2,0)+left_upleg_mass(2,0)+right_lowleg_mass(2,0)+right_upleg_mass(2,0)) +
                    10*9.81*hip_mass(2,0);
            auto W =K(0,0)+V;

            updates->get_mutable_vector().SetFromVector(u);
            COM_FOOT << xcom_des, com, vcom, right_toe_pos, left_toe_pos, foot_traj_des,
                        left_lowleg_mass(1,0), left_upleg_mass(1,0), right_lowleg_mass(1,0), right_upleg_mass(1,0), hip_mass(1,0),
                        llowleg_v(1,0), luppleg_v(1,0), rlowleg_v(1,0), ruppleg_v(1,0), hip_v(1,0), W, K, V;
            t_pre = u;

            // recored all the states
            sim_time += h;
            t_pre_ = t_pre;
            m_ = m;
            count_num_ = count_num;
            count_num_change_ = count_num_change;
            step_num_ = step_num;
            replanning_ = replanning;
            isNew_ = isNew;
            stance_now_ = stance_now;
            }
        }

        void AbstractUpdate(const systems::Context<double> & context,
                            systems::State<double> *state) const {
            auto& a_state = context.get_abstract_state();
            const bool& modified = context.get_abstract_state<bool>(12);
            if (modified) {
                period_state_ = a_state.get_value(0).get_value<int>();
                sim_time = a_state.get_value(1).get_value<double>();
                t_pre_ = a_state.get_value(3).get_value<Eigen::Matrix<double, NU, 1>>();
                m_ = a_state.get_value(4).get_value<int>();
                count_num_ = a_state.get_value(5).get_value<int>();
                count_num_change_ = a_state.get_value(6).get_value<bool>();
                step_num_ = a_state.get_value(7).get_value<int>();
                replanning_ = a_state.get_value(8).get_value<bool>();
                isNew_ = a_state.get_value(9).get_value<bool>();
                stance_now_ = a_state.get_value(10).get_value<double>();
                location_bias_ = a_state.get_value(11).get_value<double>();
                modified_ = modified;
                incline_ = a_state.get_value(13).get_value<int>();
                refreshed_ = a_state.get_value(14).get_value<bool>();
                lateral_incline_ = a_state.get_value(15).get_value<int>();
                lateral_bias_ = a_state.get_value(16).get_value<double>();
                context_changed_ = true;
                semophore_ = true;
            } // stack pull

            if (modified_ && context_changed_) {
                modified_ = false;
                context_changed_ = false;
            }

            state->template get_mutable_abstract_state<int>(0) = period_state_;
            state->template get_mutable_abstract_state<double>(1) = sim_time;
            state->template get_mutable_abstract_state<double>(2) = t;
            state->template get_mutable_abstract_state<Eigen::Matrix<double, NU, 1>>(3) = t_pre_;
            state->template get_mutable_abstract_state<int>(4) = m_;
            state->template get_mutable_abstract_state<int>(5) = count_num_;
            state->template get_mutable_abstract_state<bool>(6) = count_num_change_;
            state->template get_mutable_abstract_state<int>(7) = step_num_;
            state->template get_mutable_abstract_state<bool>(8) = replanning_;
            state->template get_mutable_abstract_state<bool>(9) = isNew_;
            state->template get_mutable_abstract_state<double>(10) = stance_now_;
            state->template get_mutable_abstract_state<double>(11) = location_bias_;
            state->template get_mutable_abstract_state<bool>(12) = modified_;
            state->template get_mutable_abstract_state<int >(13) = incline_;
            state->template get_mutable_abstract_state<bool>(14) = refreshed_;
            state->template get_mutable_abstract_state<int>(15) = lateral_incline_;
            state->template get_mutable_abstract_state<double>(16) = lateral_bias_;

            std::cout << "abstract update" << std::endl;
        }

        int update_automata(bool isNew, int step_num, double vcom_saggital, double com_lateral) const {
            static double lowest_velocity = 1;
            static int keyframe_num = 0;
            static bool ticket = false;
            static int m = 300, n = 300;
            static bool first = true;
            static bool protection = false; // to make it proceed for one piece
            static int drift_num = 0;
            if (step_num > 10) {
                if (first) {
                    std::cout << "1" << std::endl;
                    first = false;
                    return 4; // recorde mode
                }
                if (refreshed_) {
                    lowest_velocity = 1;
                    keyframe_num = 0;
                    m = 300;
                    n = 300;
                    ticket = false;
                    refreshed_ = false;
                }

                if (vcom_saggital < lowest_velocity)
                    lowest_velocity = vcom_saggital;
                else if (m) {
                    if (m == 1) {
                        keyframe_num += 1;
                    }
                    m-=1;
                } else if (n && ticket) {
                    if (n == 1) {
                        keyframe_num += 1;
                    }
                    n-=1;
                }

                std::cout << "actual_com_saggittal velocity:" << vcom_saggital
                            << "\tlowest velocity:" << lowest_velocity <<
                            "\t keyframe_num:" << keyframe_num << "\tm:" << m << "\tn:" << n << std::endl;

                // state judge
                if (keyframe_num == 1) {
                    if (isNew) {
                        if (protection) {
                            protection = false;
                            std::cout << "2" << std::endl;
                            return 4; // refresh mode
                        }
                        if (com_lateral > Maxlateralbias) {
                            drift_num += 1;
                            lateral_incline_ = 1;
                        } else if (com_lateral < -Maxlateralbias) {
                            drift_num += 1;
                            lateral_incline_ = -1;
                        } else {
                            lateral_incline_ = 0; // no need to give bias
                        }

                        if (drift_num == 2) lateral_incline_ = 0;
//                        if (fabs(vcom_lateral) < MaxLateralV) {
//                            lateral_incline_ = 0;
//                        }
                        lowest_velocity = 1; ticket = true;
                    }
                }

                {
                    if (vcom_saggital > TOP) {
                        incline_ = 1;
                        std::cout << "3" << std::endl;
                        return 0;
                    } else if (vcom_saggital < BOTTOM) {
                        incline_ = -1;
                        std::cout << "4" << std::endl;
                        return 0;
                    }
                } // if the saggital velocity exceeds the bound

                if (keyframe_num == 2) {
                    if (isNew) {
                        std::cout << "5" << std::endl;
                        return 0;
                    }
                    if (lowest_velocity > Lowest_V + BandWidth_L) {
                        incline_ = 1;
                        std::cout << "6" << std::endl;
                        return 0;
                    } else if (lowest_velocity < Lowest_V) {
                        incline_ = -1;
                        std::cout << "7" << std::endl;
                        return 0; // refresh mode
                    } else if (lateral_incline_ == 0) {
                        protection = true;
                        std::cout << "8" << std::endl;
                        return 1; // preceeding one piece mode
                    } else { // lateral_incline_ != 0 incline_ == 0
                        incline_ = 0;
                        return 0; // refresh mode
                    }
                }
                return 3; // predict mode
            }
            return 2; // smoothing mode
        }

        void CopyCommandOutSim(const systems::Context<double>& context,
                             drake::examples::KneedCompassGait::KcgInput<double>* output) const {
            t = context.get_time();
//            std::cout << "copy state out:" << context.get_discrete_state()[2] << "\t output time:" << t << std::endl;
            output->set_hrtau(context.get_discrete_state()[0]);
            output->set_htau(context.get_discrete_state()[1]);
            output->set_ltau(context.get_discrete_state()[2]);
            output->set_hytau(context.get_discrete_state()[3]);
            output->set_rtau(context.get_discrete_state()[4]);
        }

        void COM_ToeOutPut(const systems::Context<double>& context,
                           systems::BasicVector<double>* output) const {
            t = context.get_time();
            output->SetFromVector(COM_FOOT);
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
            Eigen::MatrixXd J = contactJacobian(r, cache, lfoot, rfoot, in_terms_of_qdot);
            // assume flat terrain at z=0 for now
            Eigen::Vector3d normal;
            normal << 0, 0, 1;
            Eigen::Matrix<double, 3, m_surface_tangents> d = surfaceTangents(normal);

            int inc = J.rows() / 3;
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
        std::vector<std::vector<double>> trajectory_;
        std::vector<std::vector<double>> foot_step_;
        std::vector<std::vector<double>> foot_trajectory_;
        Eigen::Matrix<double, NU, NU> weight_t_;
        Eigen::Matrix<double, 1, 1> weight_yaw_;
        Eigen::Matrix<double, 2, 2> weight_slack_;
        Eigen::Matrix<double, 3, 3> weight_slack2_;
        Eigen::Matrix<double, 3, 3> weight_slack3_;
        mutable Eigen::Matrix<double, 2, 4> Aeq_slack;
        mutable Eigen::Matrix<double, 37, 1> COM_FOOT;
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
        mutable bool context_changed_ = false;
        mutable double t, sim_time = 0;
        double h = 0.0005;
        mutable int period_state_=2;
        mutable Eigen::Matrix<double, NU, 1> t_pre_;
        mutable int m_=0;
        mutable int count_num_ = 1;
        mutable bool count_num_change_ = false;
        mutable int step_num_ = 1;
        mutable bool replanning_ = false;
        mutable bool isNew_ = false;
        mutable double stance_now_ = -1.2;
        mutable double location_bias_ = 0;
        mutable bool modified_ = false; // modified means went through one simulator.AdvanceTo
        mutable bool semophore_ = false;
        mutable int incline_ = 0; // 0 means no incline, -1 means too low, 1 means too large
        mutable bool refreshed_ = false;
        mutable int lateral_incline_ = 0;
        mutable double lateral_bias_ = 0;
    };
}
}
}
