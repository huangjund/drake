#pragma once

#include <cmath>
#include <iostream>
#include <map>
#include <memory>
#include <vector>
#include <iomanip>

#include "drake/systems/analysis/simulator.h"
#include "drake/systems/framework/leaf_system.h"
#include "drake/systems/primitives/linear_system.h"

#define ZAPEX 0.68
#define AQ 0.3
#define RATIO 3

namespace linear_system
{
    class LinearSystem : public drake::systems::LeafSystem<double>
    {
    public:
        double x_out;
        LinearSystem(Eigen::Matrix<double, Eigen::Dynamic, 1> _U)
        {
            this->DeclareVectorOutputPort("y", drake::systems::BasicVector<double>(sz),
                                          &LinearSystem::CopyStateOut);
            this->DeclareVectorOutputPort("foot_step", drake::systems::BasicVector<double>(8),
                                          &LinearSystem::CopyDiscreteStateOut);
            this->DeclarePerStepUnrestrictedUpdateEvent(&LinearSystem::Update);
            this->DeclareContinuousState(sz); // One state variable.
            this->DeclareDiscreteState(8);
            U = _U;
            zero << 0, 0, 0, 0, 0, 0, 0, 0, 0;
            identity << 1, 0, 0, 0, 1, 0, 0, 0, 1;
            CalculateAB(AQ / RATIO, 0.2, ZAPEX);
        }

        void CalculateAB(double aq, double bq, double z_apex) const
        {
            Eigen::Matrix<double, 3, 3> Q, R;
            Eigen::Matrix<double, 6, 6> _A, _B;
            double w_sq = g / z_apex;
            Q << 1, 0, 0, 0, 1, 0, aq, bq, 0;
            R << 0, 1 / (m * g), bq / (m * g), 1 / (m * g), 0, aq / (m * g),
                    bq / (m * g), aq / (m * g), 2 * aq * bq / (m * g);
            _A << zero, Q, w_sq * Q, zero;
            _B << zero, zero, -w_sq * Q, w_sq * -R;
            A = _A;
            B = _B;
        }

        void SetInitState(Eigen::Matrix<double, 6, 1> &X0,
                          drake::systems::ContinuousState<double> &state,
                          std::vector<double> &_step_lengths,
                          std::vector<double> &_change_in_yaw)
        {
            for (unsigned i = 0; i < X0.size(); i++)
            {
                state[i] = X0[i];
            }
            X = X0;
            step_lengths = _step_lengths;
            change_in_yaw = _change_in_yaw;
        }

        void SetInput(Eigen::Matrix<double, Eigen::Dynamic, 1> _U) { U = _U;
            foot(0, 0) = U(0, 0)-step_lengths[0];}

    private:
        double m = 20;
        double g = 9.81;
        mutable int step = 0;
        mutable double yaw = 0;
        mutable bool left = false;
        std::vector<double> step_lengths;
        std::vector<double> change_in_yaw;
        static const int sz = 6;
        mutable Eigen::Matrix<double, Eigen::Dynamic, 1> U;
        mutable Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> A, B;
        Eigen::Matrix<double, 3, 3> zero, identity;
        mutable Eigen::Matrix<double, 6, 1> X;
        mutable Eigen::Matrix<double, 6, 1> xdot;
        mutable Eigen::Matrix<double, 6, 1> foot;
        double h = 0.0005;
        mutable double time = 0, switch_x, t;
        mutable int surf = 1;
        // xdot = -x + x³
        void DoCalcTimeDerivatives(
                const drake::systems::Context<double> &context,
                drake::systems::ContinuousState<double> *derivatives) const override {
            auto d_state = context.get_discrete_state().get_vector().CopyToVector();
            switch_x = U(0, 0) + step_lengths[step] / 2;
            t = context.get_time();
            while (time < t - h) {
                xdot = A * X + B * U;
                X = X + xdot * h;
                X(5, 0) = A(2, 3) * X(3, 0) + A(2, 4) * X(4, 0);
                if (X(0, 0) > switch_x) {
                    yaw = 0;
                    X(3, 0) =
                            X(3, 0) + 2 * (switch_x - X(0, 0)) / (xdot(0, 0)) * xdot(0, 0);
                    left = !left;
                    foot(0, 0) = U(0, 0);
                    foot(3, 0) = 0;
                    foot(5, 0) = 0;
                    U(0, 0) = U(0, 0) + step_lengths[step];
                    switch_x = U(0, 0) + step_lengths[step];
                    CalculateAB(AQ / RATIO, 0.2, ZAPEX);
                    surf += 1;
                }
                if (X(2, 0) > ZAPEX && surf % 2 == 1) {
                    //                   std::cout << "=====================switch====================" << std::endl;
                    CalculateAB(-AQ, -0.2, ZAPEX);
                    surf += 1;
                }
                time = time + h;
//                                for (int i = 0; i < 6; ++i) {
//                                    if (i == 0 || i == 3)
//                                        std::cout << std::fixed<<std::setprecision(6)<< -X[i] <<" ";
//                                    else
//                                        std::cout << std::fixed<<std::setprecision(6)<< X[i] <<" ";
//                                }
//
//                                std::cout << "\n";
                //                for (int i = 0; i < 8; ++i) {
                //                    std::cout << std::fixed<<std::setprecision(6)<< d_state[i] <<" ";
                //                }
                //
                //                std::cout << "\n";
                //            std::cout << "==================================TIME:" << time << "==========================" << "\n";
                float bf = 0.1 + U(2, 0);
                float af = step_lengths[step] - bf;
                float kf = U(2, 0);
                foot(0, 0) = foot(0, 0) + foot(3, 0) * h;
                double acc = 7;
                if (foot(0, 0) > U(0, 0) && foot(3, 0) > 0) {
                    foot(3, 0) = foot(3, 0) - acc * h;
                } else if (foot(0, 0) < X(0, 0)) {
                    foot(3, 0) = foot(3, 0) + 1.01 * acc * h;
                } else {
                    foot(3, 0) = foot(3, 0) + acc * h;
                }
                foot(1, 0) = 0;
                foot(4, 0) = 2 * X(4, 0);
                float hf = U(0, 0);
                if (foot(0, 0) > U(0, 0) + af) {
                    hf = U(0, 0) + af;
                    foot(2, 0) = std::sqrt(bf * bf - (foot(0, 0) - hf) * (foot(0, 0) - hf));
                    foot(5, 0) =
                            (-foot(0, 0) + hf) * foot(3, 0) / (foot(2, 0) - kf + 0.0000001);
                } else if (foot(0, 0) < U(0, 0) - af) {
                    hf = U(0, 0) - af;
                    foot(2, 0) = std::sqrt(bf * bf - (foot(0, 0) - hf) * (foot(0, 0) - hf));
                    foot(5, 0) =
                            (-foot(0, 0) + hf) * foot(3, 0) / (foot(2, 0) - kf + 0.00000001);
                } else {
                    foot(2, 0) = bf;
                    foot(5, 0) = 0;
                }
                if (std::isnan(foot(2, 0))) {
                    foot(2, 0) = 0;
                    foot(5, 0) = 0;
                }
                if (foot(0, 0) >= U(0, 0) + step_lengths[step]) {
                    foot(0, 0) = U(0, 0) + step_lengths[step];
                    foot(2, 0) = 0;
                    foot(3, 0) = 0;
                    foot(5, 0) = 0;
                }
//                for (int i = 0; i < 6; ++i) {
//                    if (i == 0 || i == 3)
//                        std::cout << std::fixed << std::setprecision(6) << -foot[i] << " ";
//                    else
//                        std::cout << std::fixed << std::setprecision(6) << foot[i] << " ";
//                }
//                std::cout << "\n";

                // context.SetContinuousState(X);
                for (int i = 0; i < sz; i++) {
                    (*derivatives)[i] = xdot(i, 0);
                }
//                for (int i = sz; i < sz + 6; i++) {
//                    (*derivatives)[i] = 0;
//                }
            }
        }
            // y = x
            void CopyStateOut(const drake::systems::Context<double> &context,
                              drake::systems::BasicVector<double> *output) const
            {
                t = context.get_time();
                Eigen::Matrix<double, 12, 1> temp;
                temp << X,foot;
                temp(0, 0) = -temp(0, 0);
                temp(3, 0) = -temp(3, 0);
                output->SetFromVector(temp);
            }

            void CopyDiscreteStateOut(const drake::systems::Context<double> &context,
                                      drake::systems::BasicVector<double> *output) const
            {
                auto d_state = context.get_discrete_state().get_vector().CopyToVector();
                output->SetFromVector(d_state);
            }

            drake::systems::EventStatus Update(
                    const drake::systems::Context<double> &context,
                    drake::systems::State<double> *state) const
            {
                t = context.get_time();
                drake::systems::ContinuousState<double> &c_state =
                        state->get_mutable_continuous_state();
                drake::systems::DiscreteValues<double> &d_state =
                        state->get_mutable_discrete_state();
                d_state[0] = -U(0, 0);
                d_state[1] = -U(1, 0);
                d_state[2] = -U(2, 0);
                d_state[3] = -(U(0, 0) + step_lengths[step]);
                d_state[4] = -U(1, 0);
                d_state[5] = -U(2, 0);
                d_state[6] = left;
                d_state[7] = !left;
                for (int i = 0; i < sz; i++)
                {
                    c_state[i] = X(i, 0);
                }
//                for (int i = sz; i < sz + 6; i++)
//                {
//                    c_state[i] = foot(i - sz, 0);
//                }
                return drake::systems::EventStatus::Succeeded();
            }
        };

        class System
        {
        public:
            Eigen::Matrix<double, 6, 1> U;
            System()
            {
                // Create the simple system.
                U << 0, 0, 0, 0, 0, 0;
                LinearSystem system(U);
                drake::systems::Simulator<double> simulator(system);
                drake::systems::ContinuousState<double> &state =
                        simulator.get_mutable_context().get_mutable_continuous_state();
                U(0, 0) = 1.2;

                Eigen::Matrix<double, 6, 1> X0;
                X0 << 1, 0, 0.65, 1, 0, 0.2;
                std::vector<double> step_lengths = {0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4, 0.4};
                std::vector<double> change_in_yaw = {0, 0, 0, 0, 0, 0, 0, 0};
                system.SetInitState(X0, state, step_lengths, change_in_yaw);
                system.SetInput(U);
                simulator.AdvanceTo(5);
            }
        };
    } // namespace linear_system
