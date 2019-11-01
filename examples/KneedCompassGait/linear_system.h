#pragma once

#include <cmath>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <vector>
#include <Eigen/Core>
#include <unsupported/Eigen/MatrixFunctions>
#include "drake/systems/analysis/simulator.h"
#include "drake/examples/KneedCompassGait/util.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

#define ZAPEX 0.68
#define AQ 0.3
#define RATIO 3
#define ACC 6
#define ApexHeight 0.08
#define HighVel 1.975
#define LateralBias 0.1

namespace linear_system {
    class LinearSystem : public drake::systems::LeafSystem<double> {
    public:
        double x_out;
        LinearSystem(Eigen::Matrix<double, Eigen::Dynamic, 1> _U) {
            DeclareVectorOutputPort("y", drake::systems::BasicVector<double>(sz + 6),
                                    &LinearSystem::CopyStateOut);
            DeclareVectorOutputPort("foot_step", drake::systems::BasicVector<double>(9),
                                    &LinearSystem::CopyDiscreteStateOut);
            DeclarePerStepUnrestrictedUpdateEvent(&LinearSystem::Update);
            DeclareContinuousState(sz + 6);  // One state variable.
            // (stance_x, stance_y, stance_z, next_stance_x, next_stance_y,
            // next_stance_z, current_stance_leg, next stance_leg, yaw)
            DeclareDiscreteState(9);
            U = _U;
        }

        void CalculateAB(double aq, double bq) const {
            Eigen::Matrix<double, 3, 3> Q, R;
            Eigen::Matrix<double, 6, 6> _A, _B;
            double w_sq = g / action_list[step].z_apex;
            Q << 1, 0, 0, 0, 1, 0, aq, bq, 0;
            R << 0, 1 / (m * g), bq / (m * g), 1 / (m * g), 0, aq / (m * g),
                    bq / (m * g), aq / (m * g), 2 * aq * bq / (m * g);
            _A << zero, identity, w_sq * Q, zero;
            _B << zero, zero, -w_sq * Q, w_sq * -R;
            A = _A;
            B = _B;
        }

        void SetInitState(Eigen::Matrix<double, 6, 1>& X0,
                          drake::systems::ContinuousState<double>& state,
                          std::vector<Action>& _action_list) {
            for (unsigned i = 0; i < X0.size(); i++) {
                state[i] = X0[i];
            }
            X = X0;
            action_list = _action_list;
            yaw = 0;
            height = 0;
            prev_step = {X(0, 0) - 0.5 * action_list[0].step_length * std::cos(yaw),
                         X(1, 0) - 0.5 * action_list[0].step_length * std::sin(yaw),
                         X(2, 0)};
            zero << 0, 0, 0, 0, 0, 0, 0, 0, 0;
            identity << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            X(5, 0) = X(3, 0) * 0.15;
            CalculateAB(0.15, 0.15);
        }

        void SetInput(Eigen::Matrix<double, Eigen::Dynamic, 1> _U) {
            U = _U;
            foot(0, 0) = U(0, 0) - action_list[0].step_length;
            foot(3, 0) = 0;
        }

        std::vector<std::vector<double>> GetComTra() { return com_tra; }

        double CalculateSwitchX(double x1, double x1_dot) const {
            double eps = 0.001;
            double w1_sq = g / action_list[step].z_apex;
            double w2_sq = g / action_list[step + 1].z_apex;
            double x1_foot = U(0, 0);
            double x2_foot = U(0, 0) + action_list[step].step_length;
            double x2 = x2_foot;
            double x1_ddot, x2_ddot;
            double x2_dot = action_list[step + 1].x_dot;
            //    std::cout << "   x1" << x1 << "   x2" << x2 << "   x1_dot" << x1_dot
            //              << "  x2_dot" << x2_dot << std::endl;
            while (x2 > x1) {
                if (x1_dot < x2_dot) {
                    x1_ddot = w1_sq * (x1 - x1_foot);
                    x1 = x1 + eps * x1_dot + 0.5 * eps * eps * x1_ddot;
                    x1_dot = x1_dot + eps * x1_ddot;
                } else {
                    x2_ddot = w2_sq * (x2 - x2_foot);
                    x2 = x2 - eps * x2_dot + 0.5 * eps * eps * x2_ddot;
                    x2_dot = x2_dot - eps * x2_ddot;
                }
            }
            if (action_list[step].dheight == 0)
                x_slopes = {-0.15, 0.15};
            else {
                x_slopes = {0, action_list[step].dheight / (x2_foot - (x1 + x2) / 2)};
            }
            // need to tune this
            //    std::cout << "the swith x is" << (x1 + x2) / 2 << "  x1_dot" << x1_dot
            //              << "  x2_dot" << x2_dot << std::endl;
            return (x1 + x2) / 2;
        }

    private:
        double m = 20;
        double g = 9.81;
        mutable int step = 0;
        mutable double yaw;
        mutable double height;
        mutable std::vector<double> x_slopes;
        mutable bool left = false;
        std::vector<Action> action_list;
        mutable std::vector<double> prev_step;
        static const int sz = 6;
        mutable Eigen::Matrix<double, Eigen::Dynamic, 1> U;
        mutable Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A, B;
        Eigen::Matrix<double, 3, 3> zero, identity;
        mutable Eigen::Matrix<double, 6, 1> X;
        mutable Eigen::Matrix<double, 6, 1> foot;
        mutable Eigen::Matrix<double, 6, 1> xdot;
        double h = 0.0005;
        mutable double time = 0, switch_x = std::numeric_limits<double>::max(), t;
        mutable int surf = 0;
        mutable std::vector<std::vector<double>> com_tra;

        // xdot = -x + x³
        void DoCalcTimeDerivatives(
                const drake::systems::Context<double>& context,
                drake::systems::ContinuousState<double>* derivatives) const override {
            t = context.get_time();
            while (time < t - h) {
                xdot = A * X + B * U;
                X = X + xdot * h;
                if (X(0, 0) > switch_x && surf % 2 == 1) {
                    prev_step[0] += action_list[step].step_length * std::cos(yaw);
                    prev_step[1] += action_list[step].step_length * std::sin(yaw);
                    prev_step[2] = X(2, 0);
                    com_tra.push_back(prev_step);
                    left = !left;
                    yaw = yaw + action_list[step].dphi;
                    height = height + action_list[step].dheight;
                    foot(0, 0) = U(0, 0);
                    foot(3, 0) = 0;
                    foot(5, 0) = 0;
//                    std::cout << "height************************" << time << std::endl;
                    U(0, 0) = U(0, 0) + action_list[step].step_length;
                    U(1, 0) = -U(1, 0);
                    U(2, 0) = U(2, 0) + action_list[step].dheight;
                    CalculateAB(x_slopes[surf], 0.2);
                    X(3, 0) =
                            X(3, 0) + 2 * (switch_x - X(0, 0)) / (xdot(0, 0)) * xdot(0, 0);
                    X(5, 0) = x_slopes[surf] * X(3, 0) + 0.2 * X(4, 0);
                    // std::cout << "switch:......." << std::endl;
                    // std::cout << X(0, 0) << "\t" << X(1, 0) << "\t" << X(2, 0) << "\t"
                    //          << X(3, 0) << "\t" << X(4, 0) << "\t" << xdot(5, 0)
                    //          << std::endl;
                    surf = 0;
                    step++;
                }
                if (X(0, 0) > U(0, 0) && surf % 2 == 0) {
                    com_tra.push_back(
                            {prev_step[0] + action_list[step].step_length * std::cos(yaw) / 2,
                             prev_step[1] + action_list[step].step_length * std::sin(yaw) / 2,
                             X(2, 0)});
                    switch_x = CalculateSwitchX(X(0, 0), X(3, 0));
                    CalculateAB(x_slopes[surf], -0.2);
                    X(2, 0) = U(2, 0) + action_list[step].z_apex;
                    X(5, 0) = x_slopes[surf] * X(3, 0) - 0.2 * X(4, 0);
                   // std::cout << "top:........" << std::endl;
//                    std::cout << X(0, 0) << "\t" << X(1, 0) << "\t" << X(2, 0) << "\t"
//                              << X(3, 0) << "\t" << X(4, 0) << "\t" << xdot(5, 0)
//                              << std::endl;
                    surf++;
                }
                time = time + h;
                float bf = ApexHeight + U(2, 0);
                float af = action_list[step].step_length - bf;
                float kf = U(2, 0);
                foot(0, 0) = foot(0, 0) + foot(3, 0) * h;
                double tau = HighVel/6;
                double x_val = 0.5*tau*HighVel;
//                double ttot = 4*action_list[step].step_length/HighVel;
//                std::cout<<tau<<"   " <<ttot<<std::endl;
                double acc;
                if (foot(0, 0) <= U(0,0)-(action_list[step].step_length-x_val)) {
                    acc = ACC;
                } else{
                    acc = -4.106;
                }
                foot(3, 0) = foot(3, 0) + acc * h;
                foot(1, 0) = 0;
                foot(4, 0) = 2 * X(4, 0);
                float hf = U(0, 0);
                if (foot(0, 0) > U(0, 0) + af) {
                    hf = U(0, 0) + af;
                    foot(2, 0) = std::sqrt(bf * bf - (foot(0, 0) - hf) * (foot(0, 0) - hf));
                    foot(5, 0) =
                            (-foot(0, 0) + hf) * foot(3, 0) / (foot(2, 0) - kf);
                } else if (foot(0, 0) < U(0, 0) - af) {
                    hf = U(0, 0) - af;
                    foot(2, 0) = std::sqrt(bf * bf - (foot(0, 0) - hf) * (foot(0, 0) - hf));
                    foot(5, 0) =
                            (-foot(0, 0) + hf) * foot(3, 0) / (foot(2, 0) - kf);
                } else {
                    foot(2, 0) = bf;
                    foot(5, 0) = 0;
                }
                if (std::isnan(foot(2, 0))) {
                    foot(2, 0) = 0;
                    foot(5, 0) = 0;
                }
                if (foot(0, 0) >= U(0, 0) + action_list[step].step_length) {
                    foot(0, 0) = U(0, 0) + action_list[step].step_length;
                    foot(2, 0) = 0;
                    foot(3, 0) = 0;
                    foot(5, 0) = 0;
                }

                double w = M_PI/action_list[step].step_length;
                double a = LateralBias/2.0;
                double y = a*std::sin(w*(X[0]-1)), ydot = a*w*std::cos(w*(X[0]-1))*X[3];
                X(1,0) = y; X(4,0) = ydot;
                for (int i = 0; i < 6; ++i) {
                    if (i == 0 || i == 3)
                        std::cout << std::fixed << std::setprecision(6) << -X[i] << " ";
                    else
                        std::cout << std::fixed << std::setprecision(6) << X[i] << " ";
                }
                X(1,0) = 0; X(4,0) = 0;
                std::cout << "\n";

//                        auto d_state =
//                        context.get_discrete_state().get_vector().CopyToVector(); for
//                        (int i = 0; i < 8; ++i) {
//                            std::cout << std::fixed<<std::setprecision(6)<< d_state[i]
//                            <<" ";
//                        }
//
//                        std::cout << "\n";

//                        for (int i = 0; i < 6; ++i) {
//                            if (i == 0 || i == 3)
//                                std::cout << std::fixed << std::setprecision(6) <<
//                                -foot[i] << " ";
//                            else
//                                std::cout << std::fixed << std::setprecision(6) <<
//                                foot[i] << " ";
//                        }
//                        std::cout << "\n";
                // std::cout << "x\n" << X << std::endl;
            }

            // context.SetContinuousState(X);
            for (int i = 0; i < sz; i++) {
                (*derivatives)[i] = xdot(i, 0);
            }
            for (int i = sz; i < sz + 6; i++) {
                (*derivatives)[i] = 0;
            }
        }

        // y = x
        void CopyStateOut(const drake::systems::Context<double>& context,
                          drake::systems::BasicVector<double>* output) const {
            t = context.get_time();
            for (int i = 0; i < sz; i++) {
                (*output)[i] = X[i];
            }
            for (int i = sz; i < sz + 6; i++) {
                (*output)[i] = foot[i - sz];
            }
        }

        void CopyDiscreteStateOut(const drake::systems::Context<double>& context,
                                  drake::systems::BasicVector<double>* output) const {
            auto d_state = context.get_discrete_state().get_vector().CopyToVector();
            output->SetFromVector(d_state);
        }

        drake::systems::EventStatus Update(
                const drake::systems::Context<double>& context,
                drake::systems::State<double>* state) const {
            t = context.get_time();
            drake::systems::ContinuousState<double>& c_state =
                    state->get_mutable_continuous_state();
            drake::systems::DiscreteValues<double>& d_state =
                    state->get_mutable_discrete_state();
            d_state[0] = -U(0, 0);
            d_state[1] = -U(1, 0);
            d_state[2] = -U(2, 0);
            d_state[3] = -(U(0, 0) + action_list[step].step_length);
            d_state[4] = -U(1, 0);
            d_state[5] = -U(2, 0);
            d_state[6] = left;
            d_state[7] = !left;
            d_state[8] = yaw;
            for (int i = 0; i < sz; i++) {
                c_state[i] = X(i, 0);
            }
            for (int i = sz; i < sz + 6; i++) {
                c_state[i] = foot(i - sz, 0);
            }
            // c_state[sz-1] = x_slopes[surf]*X(3,0)+0.2*X(4,0);
            return drake::systems::EventStatus::Succeeded();
        }
    };

    class System {
    public:
        Eigen::Matrix<double, 6, 1> U;
        std::vector<std::vector<double>> com_tra;
        Primitives<double> prim;
        System() {
            // Create the simple system.
            // prim = prmtvs;
            U << 0, 0, 0, 0, 0, 0;
            LinearSystem system(U);
            drake::systems::Simulator<double> simulator(system);
            drake::systems::ContinuousState<double>& state =
                    simulator.get_mutable_context().get_mutable_continuous_state();
            U(0, 0) = 1.2;

            Eigen::Matrix<double, 6, 1> X0;
            X0 << 1, 0, 0.65, 0.83, 0, 0.125;
            std::vector<double> step_lengths = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
                                                0.4, 0.4, 0.4, 0.4, 0.4, 0.4,
                                                0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
            std::vector<double> key_xdot = {0.35, 0.35, 0.35, 0.35, 0.35,
                                            0.35, 0.35, 0.35, 0.35, 0.35,
                                            0.35, 0.35, 0.35, 0.35, 0.35,
                                            0.35, 0.35, 0.35, 0.35, 0.35};
            std::vector<double> change_in_yaw = {0, 0, 0, 0, -0, -0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            std::vector<double> change_in_height = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
            std::vector<double> key_zap = {0.68, 0.68, 0.68, 0.68, 0.68, 0.68,
                                           0.68, 0.68, 0.68, 0.68, 0.68, 0.68,
                                           0.68, 0.68, 0.68, 0.68, 0.68, 0.68};
            std::vector<Action> action_list;
            for (int i = 0; i < 18; i++) {
                Action acn(change_in_yaw[i], change_in_height[i], step_lengths[i],
                           key_xdot[i], key_zap[i]);
                action_list.push_back(acn);
            }
            system.SetInitState(X0, state, action_list);
            system.SetInput(U);
            simulator.AdvanceTo(12);
        }
    };
}  // namespace linear_system