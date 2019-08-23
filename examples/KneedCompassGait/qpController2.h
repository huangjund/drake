//
// Created by Junda Huang on 8/20/19.
//
#pragma once

#include <memory>
#include <vector>
#include <string>

#include "drake/examples/KneedCompassGait/KCG_common.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_ContinuousState.h"
#include "drake/examples/KneedCompassGait/gen/KneedCompassGait_Params.h"
#include "drake/examples/KneedCompassGait/gen/KCG_Input.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
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
#define W1 1
#define W2 1

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
            this->DeclareVectorInputPort("trajectory", systems::BasicVector<double>(NQ+1));
            this->DeclareVectorOutputPort(KneedCompassGait::KcgInput<double>(),
                    &qpController::CopyCommandOutSim);
            this->DeclareDiscreteState(NU);

           // constexpr double print_period = 0.05;
           // double last_print = -print_period;
           // auto last_v = Eigen::VectorXd::Zero(3); // initialization

            this->DeclarePeriodicDiscreteUpdate(0.0005);
            VectorX<double> q_d(NQ);
            q_d.setZero();

            this->com_des = this->rtree_->centerOfMass(this->rtree_->doKinematics(q_d));
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
            std::cout << "ok" << std::endl;
            Eigen::Matrix<double, NU, 1> t_pre;
            t_pre.setZero();

            // add decision variables---------------------------------
            auto vdot = prog.NewContinuousVariables(NV, "joint acceleration"); // qddot var
            auto u = prog.NewContinuousVariables(NU, "input");  // torque var
            auto beta = prog.NewContinuousVariables(NF, "friction forces"); // contact force var
            auto x_yaw = prog.NewContinuousVariables(1, "yaw");  // robot yaw
         //    auto eps = prog.NewContinuousVariables(neps, "slack variables");
         //   int nparams = prog.num_vars();

            // Problem Constrains --------------------------------------
            auto con_u_lim = prog.AddBoundingBoxConstraint(umin,umax, u).evaluator();
         //   auto con_fric_lim = prog.AddBoundingBoxConstraint(5, 1000000, beta).evaluator();
        //    auto con_slack_lim = prog.AddBoundingBoxConstraint(
        //            -slack_limit, slack_limit, eps).evaluator();

            // natural dynamic constraint
            Eigen::Matrix<double, NQ-NU, NQ+NF> idyn_conf;
            idyn_conf.setZero();
            auto con_dynf = prog.AddLinearEqualityConstraint(
                    idyn_conf, Eigen::VectorXd::Zero(NQ-NU), {vdot, beta}).evaluator();

            Eigen::Matrix<double, NU, NQ+NU+NF> idyn_cona;
            idyn_cona.setZero();
            auto con_dyna = prog.AddLinearEqualityConstraint(
                    idyn_cona, Eigen::VectorXd::Zero(NU), {vdot, u, beta}).evaluator();

            // Problem Cost --------------------------------------------
            this->Kp_qdd_.setOnes();
            this->Kp_qdd_ = this->Kp_qdd_*5;
            this->Kd_qdd_ = 0;

            Eigen::VectorXd qddot_des(NQ);
            qddot_des.setZero();
            // lack of PD controller to compute qddot

            this->w_qdd_ << 100, 10, 10, 10, 100, 100, 10, 10, 5, 10, 1;
            this->weight_ = Eigen::Matrix<double, NQ, NQ>(w_qdd_.asDiagonal());
            auto cost_qdd = prog.AddQuadraticErrorCost(weight_, qddot_des, vdot).evaluator();

            this->w_yaw_ = W1;
            Eigen::Matrix<double, 1, 1> w_yaw(2*this->w_yaw_);
            Vector1<double> b_yaw(0);
            auto cost_yaw = prog.AddQuadraticCost(w_yaw, b_yaw, x_yaw).evaluator();

            this->w_t_ = W2;
            this->weight_t_.setIdentity();
            this->weight_t_ = this->w_t_*this->weight_t_;
            auto cost_t = prog.AddQuadraticErrorCost(this->weight_t_, t_pre, u).evaluator();

            // solver setting -------------------------------------------
            this->prog.SetSolverOption(this->solver.solver_id(), "Method", 2);

            // save variables -------------------------------------------
            this->q_des_ = q_d;
            this->con_dynf_ = con_dynf;
            this->con_dyna_ = con_dyna;
            this->cost_qdd_ = cost_qdd;
            this->cost_t_ = cost_t;
            this->cost_yaw_ = cost_yaw;
        }

        void DoCalcDiscreteVariableUpdates(
                const drake::systems::Context<double>& context,
                const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
                drake::systems::DiscreteValues<double>* updates) const override {
            static Eigen::Matrix<double, NU, 1> t_pre;
            t_pre.setZero();

          //  auto utime = context.get_time()*1000000;

          // get the current state and current disired trajectory
            VectorX<double> traj = this->EvalVectorInput(context, 1)->CopyToVector();
            VectorX<double> q_des(NQ),x_yaw_des(1); // qd_des(NV),
            VectorX<double> x = this->EvalVectorInput(context, 0)->CopyToVector();
            VectorX<double> q(NQ), qd(NV);
            for (int i = 0; i < NQ; ++i) {
                q_des[i] = traj[i];
                q[i] = x[i];
           //     qd_des[i] = traj[i+NQ];
                qd[i] = x[i+NQ];
            }
            x_yaw_des[1] = traj[NQ];

            std::cout << "traj\n" << traj << std::endl;
            // create the current kinematic cache
            auto kinsol = (this->rtree_)->doKinematics(q, qd);
            Vector3<double> toe_collision_bias(0, 0, -0.5);

            // setup some dynamic terms
            auto H = rtree_->massMatrix(kinsol);
            auto C = rtree_->dynamicsBiasTerm(kinsol, {}, true);
            auto B = rtree_->B;
            Eigen::MatrixXd JB;
            contactJacobianBV(*(this->rtree_), kinsol, *(this->lfoot_), *(this->rfoot_), false, JB);

            Eigen::Matrix<double, NQ-NU, NQ> Hf(H.topRows(NQ-NU));
            Eigen::Matrix<double, NU, NQ> Ha(H.bottomRows(NU));
            Eigen::Matrix<double, NQ-NU, 1> Cf(C.topRows(NQ-NU));
            Eigen::Matrix<double, NU, 1> Ca(C.bottomRows(NU));
            Eigen::Matrix<double, NQ-NU, NF> JBf(JB.topRows(NQ-NU));
            Eigen::Matrix<double, NU, NF> JBa(JB.bottomRows(NU));
            Eigen::Matrix<double, NQ-NU, NU> Bf;
            Eigen::Matrix<double, NU, NU> Ba;
            Bf.setZero();Ba.setIdentity();

           // auto com = rtree_->centerOfMass(kinsol);
            auto Jcom = rtree_->centerOfMassJacobian(kinsol);
           // auto Jcomdot_times_v = rtree_->centerOfMassJacobianDotTimesV(kinsol);

            // Problem Constraints -------------------------------------------------
            Eigen::Matrix<double, NQ-NU, NQ+NF> dyn_conf;
            dyn_conf.setZero();
            dyn_conf << Hf, -JBf;
            this->con_dynf_->UpdateCoefficients(dyn_conf, -Cf);

            Eigen::Matrix<double, NU, NQ+NU+NF> dyn_cona;
            dyn_cona.setZero();
            dyn_cona << Ha, -Ba, -JBa;
            this->con_dyna_->UpdateCoefficients(dyn_cona, -Ca);

            // Problem Costs ------------------------------------------------------
            // PD controller
            auto kp = this->Kp_qdd_;
            float kd = this->Kd_qdd_;
            auto qddot_des = kp.cwiseProduct(q_des-q) - kd*(qd);

            this->cost_qdd_->UpdateCoefficients(2*(this->weight_), -2*(this->weight_)*qddot_des);

            this->cost_t_->UpdateCoefficients(2*(this->weight_t_), -2*(this->weight_t_)*t_pre);

            VectorX<double> Aeq(1);
            this->cost_yaw_->UpdateCoefficients(2*Aeq, 2*(-2*x_yaw_des));

            drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(this->prog);
            auto result_vec = result.GetSolution();
            Eigen::Matrix<double, NU, 1> u(result_vec.middleRows(NQ, NU));
            std::cout << "result vector:\n" << result_vec << std::endl;
            updates->get_mutable_vector().SetFromVector(u);
            t_pre = u;
          //  std::cout << "ok2" << std::endl;
        }

        void CopyCommandOutSim(const systems::Context<double>& context,
                             drake::examples::KneedCompassGait::KcgInput<double>* output) const {
            output->set_hrtau(context.get_discrete_state()[0]);
            output->set_htau(context.get_discrete_state()[1]);
            output->set_ltau(context.get_discrete_state()[2]);
            output->set_hytau(context.get_discrete_state()[3]);
            output->set_rtau(context.get_discrete_state()[4]);
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
            int ncols = inc * m_surface_tangents;
            int nrows;
            if (in_terms_of_qdot) {
                nrows = r.get_num_positions();
            } else {
                nrows = r.get_num_velocities();
            }

            Eigen::Matrix3Xd B(3, ncols);  // Friction polyhedron basis vectors
          //  B.resize(3, ncols);
            JB.resize(nrows,
                      ncols);  // Jacobian mapping forces along friction basis vectors

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
        float Kd_qdd_;
        bool initialized;
        float w_yaw_;
        float w_t_;
        Eigen::Matrix<double, NU, NU> weight_t_;
        std::shared_ptr<solvers::LinearEqualityConstraint> con_dynf_;
        std::shared_ptr<solvers::LinearEqualityConstraint> con_dyna_;
        std::shared_ptr<solvers::QuadraticCost> cost_qdd_;
        std::shared_ptr<solvers::QuadraticCost> cost_t_;
        std::shared_ptr<solvers::QuadraticCost> cost_yaw_;
        Eigen::Matrix<double, NQ, 1> w_qdd_;
        solvers::GurobiSolver solver;
        solvers::MathematicalProgram prog;
        Eigen::Matrix<double, NQ, NQ> weight_;
        VectorX<double> q_des_;
        VectorX<double> com_des;
        Eigen::Matrix<double, NQ, 1> Kp_qdd_;
        std::unique_ptr<RigidBodyTree<double>> rtree_;
        RigidBody<double>* lfoot_;
        RigidBody<double>* rfoot_;
    };

}
}
}
