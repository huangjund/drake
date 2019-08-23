//
// Created by Junda Huang on 8/12/19.
//
#include <memory>
#include <string>

#include "drake/solvers/osqp_solver.h"
#include "drake/solvers/gurobi_solver.h"
#include "drake/solvers/mathematical_program.h"
#include "drake/multibody/rigid_body_tree.h"
#include "drake/solvers/solve.h"
#include "drake/systems/primitives/linear_system.h"

int main(){
    using std::cout;
    using drake::systems::LinearSystem;
    using std::endl;


//    Eigen::Matrix<double, 2, 2> A;
//    Eigen::Matrix<double, 2, 1> b;
//    Eigen::Matrix<double, 1, 2> c;
//    Eigen::Matrix<double, 1, 1> d;
//    LinearSystem<double> system(A, b, c, d);


    drake::solvers::MathematicalProgram prog;
    // add decision variables ------------------
    auto x = prog.NewContinuousVariables(2, "x_variables");
    auto y = prog.NewContinuousVariables(3, "y_variables");
    auto z = prog.NewContinuousVariables(4, "z_variables");
    // add problem constrains ------------------
    Eigen::Vector2d xulim(1, 1), xllim(-1, -1);
    auto con_x_lim = prog.AddBoundingBoxConstraint(xllim, xulim, x).evaluator();
    // Problem cost ----------------------------
    auto Q = Eigen::MatrixXd::Identity(2,2);
    auto b = Eigen::MatrixXd::Zero(2,1);
    auto cost = prog.AddQuadraticCost(Q, b, x).evaluator();

    drake::solvers::GurobiSolver solver;
    drake::solvers::SolverOptions options;
    options.SetOption(solver.id(), "Method", 2);
    Eigen::Vector2d initial_guess(0.5,0.5);
    prog.SetSolverOption(solver.solver_id(), "Method", 2);
    prog.SetInitialGuess(x, initial_guess);
    const drake::solvers::MathematicalProgramResult result = drake::solvers::Solve(prog);
    cout << result.GetSolution() << endl;
    return 0;
}
