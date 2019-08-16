//
// Created by Junda Huang on 8/10/19.
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

#define NQ 9
#define NV 9
#define NU 3
#define DIM 3
#define ND 4 // friction cone approx
#define NC 4 // 2 on the left 2 on the right
#define NF 16 // number of contact force variables
#define m_surface_tangents 4
#define mu 1.0

namespace drake {
namespace examples {
namespace qpControl {
    class qpController : public systems::LeafSystem<double> {
    public:
        qpController(VectorX<double> q_des) {
            // get rigidbodytree of kneed compass gait
            this->rtree_=getKCGTree<double>();

            this->neps = NC*DIM; // number of slack variables for contact

            this->DeclareVectorInputPort("KCG_states", systems::BasicVector<double>(NQ+NV));
            this->DeclareVectorOutputPort(KneedCompassGait::KcgInput<double>(),
                    &qpController::CopyStateOutSim);
            this->DeclareDiscreteState(NU);

           // constexpr double print_period = 0.05;
           // double last_print = -print_period;
           // auto last_v = Eigen::VectorXd::Zero(3); // initialization

            this->DeclarePeriodicDiscreteUpdate(0.0005);
            VectorX<double> q_d(NQ);
            q_d = q_des;
            this->com_des = this->rtree_->centerOfMass(this->rtree_->doKinematics(q_d));
            auto u_des = examples::kkk::KCGFixedPointTorque();

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

            // add decision variables---------------------------------
            auto vdot = prog.NewContinuousVariables(NV, "joint acceleration"); // qddot var
            auto u = prog.NewContinuousVariables(NU, "input");  // torque var
            auto beta = prog.NewContinuousVariables(NF, "friction forces"); // contact force var
        //    auto eps = prog.NewContinuousVariables(neps, "slack variables");
         //   int nparams = prog.num_vars();

            // Problem Constrains --------------------------------------
            auto con_u_lim = prog.AddBoundingBoxConstraint(umin,umax, u).evaluator();
            auto con_fric_lim = prog.AddBoundingBoxConstraint(5, 1000000, beta).evaluator();
        //    auto con_slack_lim = prog.AddBoundingBoxConstraint(
        //            -slack_limit, slack_limit, eps).evaluator();

            // natural dynamic constraint
            Eigen::Matrix<double, NQ, NQ+NU+NF> idyn_con;
            idyn_con.setZero();

            auto con_dyn = prog.AddLinearEqualityConstraint(
                    idyn_con, Eigen::VectorXd::Zero(NQ), {vdot,u,beta}).evaluator();
            // Problem Cost --------------------------------------------
            this->Kp_qdd = 10;
            this->Kd_qdd = 0;

            Eigen::VectorXd qddot_des(NQ);
            qddot_des.setZero();

            Eigen::Matrix3d I1;
            I1.setIdentity();
            Eigen::Matrix<double, NQ-NU, NU> I2;
            Eigen::Matrix<double, NQ, NU> I;
            I2.setZero();
            I << I2, I1;
            this->w_qdd << 10, 10, 10, 1000, 1000, 1000, 1, 1, 1;
            this->weight = Eigen::MatrixXd(w_qdd.asDiagonal());
            auto cost_qdd = prog.AddQuadraticErrorCost(weight, qddot_des, vdot).evaluator();

            // solver setting -------------------------------------------
            this->prog.SetSolverOption(this->solver.solver_id(), "Method", 2);
            // save variables -------------------------------------------
            this->q_des_ = q_d;
            this->con_dyn_ = con_dyn;
            this->cost_qdd_ = cost_qdd;
        }

        void CopyStateOutSim(const systems::Context<double>& context,
                drake::examples::KneedCompassGait::KcgInput<double>* output) const {
            output->set_ltau(context.get_discrete_state()[0]);
            output->set_htau(context.get_discrete_state()[1]);
            output->set_rtau(context.get_discrete_state()[2]);
        }

        void DoCalcDiscreteVariableUpdates(
                const drake::systems::Context<double>& context,
                const std::vector<const drake::systems::DiscreteUpdateEvent<double>*>& ,
                drake::systems::DiscreteValues<double>* updates) const override {
            VectorX<double> x = this->EvalVectorInput(context, 0)->CopyToVector();

            VectorX<double> q(NQ);
            VectorX<double> qd(NQ);
            for (int i = 0; i < NQ; ++i) {
                q[i] = x[i];
                qd[i] = x[i+NQ];
            }
           // auto utime = context.get_time()*10000000;
            auto kinsol = (this->rtree_)->doKinematics(q, qd);
            Vector3<double> toe_collision_bias(0, 0, -0.5);
//            if(!this->initialized){
//                auto toe_l = rtree_->transformPoints(kinsol, toe_collision_bias, lfoot->get_body_index(), 0);
//                auto toe_r = rtree_->transformPoints(kinsol, toe_collision_bias, rfoot->get_body_index(), 0);
//                double yaw = q[5];
//                Matrix2<double> yaw_rot;
//                yaw_rot << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
//                this->initialized = true;
//            }
            auto H = rtree_->massMatrix(kinsol);
            auto C = rtree_->dynamicsBiasTerm(kinsol, {}, true);
            auto B = rtree_->B;

           // auto com = rtree_->centerOfMass(kinsol);
            auto Jcom = rtree_->centerOfMassJacobian(kinsol);
           // auto Jcomdot_times_v = rtree_->centerOfMassJacobianDotTimesV(kinsol);

            // Problem Constraints -------------------------------------------------
            Eigen::Matrix<double, NQ, NQ+NU+NF> dyn_con;
            dyn_con.setZero();
            Eigen::MatrixXd JB;

            contactJacobianBV(*(this->rtree_), kinsol, *(this->lfoot_), *(this->rfoot_), false, JB);
            dyn_con << H, -B, -JB;

            this->con_dyn_->UpdateCoefficients(dyn_con, -C);
            // Problem Costs ------------------------------------------------------
            // PD controller
            float kp = this->Kp_qdd;
            float kd = this->Kd_qdd;
            auto q_des = this->q_des_;
            auto qddot_des = kp*(q_des-q) - kd*(qd);
            this->cost_qdd_->UpdateCoefficients(2*(this->weight), -2*(this->w_qdd)*qddot_des);

            drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(this->prog);
            auto result_vec = result.GetSolution();
            Eigen::Vector3d u;
            u << result_vec[9], result_vec[10], result_vec[11];
            std::cout << result_vec[9] << " " << result_vec[10] << " " << result_vec[11] << std::endl;
            updates->get_mutable_vector().SetFromVector(u);
            std::cout << "ok2" << std::endl;
        }

        Eigen::MatrixXd contactJacobian(const RigidBodyTree<double> &r,
                                 KinematicsCache<double> &cache,
                                 RigidBody<double> &lfoot, RigidBody<double> &rfoot,
                                 bool in_terms_of_qdot) const {
            std::cout << "ok3 " << std::endl;
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
            std::cout << " ok4 " << std::endl;
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
            std::cout << " ok5 " << std::endl;
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
        float Kp_qdd;
        float Kd_qdd;
        bool initialized;
        std::shared_ptr<solvers::LinearEqualityConstraint> con_dyn_;
        std::shared_ptr<solvers::QuadraticCost> cost_qdd_;
        Eigen::Matrix<double, NQ, 1> w_qdd;
        solvers::GurobiSolver solver;
        solvers::MathematicalProgram prog;
        Eigen::MatrixXd weight;
        VectorX<double> q_des_;
        VectorX<double> com_des;
        std::unique_ptr<RigidBodyTree<double>> rtree_;
        RigidBody<double>* lfoot_;
        RigidBody<double>* rfoot_;
    };

}
}
}
