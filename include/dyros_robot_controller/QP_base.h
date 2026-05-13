// Copyright 2026 Electronics and Telecommunications Research Institute (ETRI)
//
// Developed by Yoon Junheon at the Dynamic Robotic Systems Laboratory (DYROS),
// Seoul National University, under a research agreement with ETRI.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once
#include <Eigen/Dense>
#include <string>
#include <iostream>
#include <fstream>

#include "OsqpEigen/OsqpEigen.h"
#include "suhan_benchmark.h"

using namespace Eigen;

namespace drc
{
    namespace QP
    {
        /**
         * @brief Struct to hold time durations for various stages of the QP solving process.
         */
        struct TimeDuration
        {
            double set_qp;          // Time taken to set up the QP problem.(set_cost + set_bound + set_ineq + set_eq + set_constraint)
            double set_cost;        // Time taken to set the cost function.
            double set_bound;       // Time taken to set the bound constraints.
            double set_ineq;        // Time taken to set the inequality constraints.
            double set_eq;          // Time taken to set equality constraints.
            double set_constraint;  // Time taken to set constraints. (bound, inequality, equality)
            double set_solver;      // Time taken to set the solver.
            double solve_qp;        // Time taken to solve the QP problem.
            /**
             * @brief Set all time durations to zero.
             */
            void setZero()
            {
                set_qp = 0;
                set_cost = 0;
                set_bound = 0;
                set_ineq = 0;
                set_eq = 0;
                set_constraint = 0;
                set_solver = 0;
                solve_qp = 0;
            }
        };
        /**
         * @brief Base class for QP solver.
         * 
         * This class provides a framework for setting up and solving QP problems.
         * It includes methods for setting the size of the QP, defining cost functions,
         * constraints, and solving the QP problem.
         */
        class QPBase
        {
            public:
                /**
                 * @brief Constructor.
                 */
                QPBase() : solver_initialized_(false), warm_start_ready_(false) {}
                /**
                 * @brief Set the size and initialize each variables of the QP problem.
                 * @param nx      (int) Number of decision variables.
                 * @param nbc     (int) Number of bound constraints.
                 * @param nineqc  (int) Number of inequality constraints.
                 * @param neqc    (int) Number of equality constraints.
                 */
                void setQPsize(const int &nx, const int&nbc, const int &nineqc, const int &neqc)
                {
                    assert(nbc == nx || nbc == 0);

                    if (solver_initialized_)
                    {
                        solver_.clearSolverVariables();
                        solver_.clearSolver();
                        solver_initialized_ = false;
                    }

                    warm_start_ready_ = false;
            
                    nx_ = nx;
                    nbc_ = nbc;
                    nineqc_ = nineqc;
                    neqc_ = neqc;
                    nc_ = nineqc_ + neqc_+ nbc_ ;
                    
                    P_ds_.setZero(nx_, nx_);
                    q_ds_.setZero(nx_);
                    
                    A_ineq_ds_.setZero(nineqc_, nx_);
                    l_ineq_ds_.setConstant(nineqc_,-OSQP_INFTY);
                    u_ineq_ds_.setConstant(nineqc_,OSQP_INFTY);
                    
                    l_bound_ds_.setConstant(nbc_,-OSQP_INFTY);
                    u_bound_ds_.setConstant(nbc_,OSQP_INFTY);
                    
                    A_eq_ds_.setZero(neqc_, nx_);
                    b_eq_ds_.setZero(neqc_);
                    
                    A_ds_.setZero(nc_, nx_);
                    l_ds_.setConstant(nc_,-OSQP_INFTY);
                    u_ds_.setConstant(nc_,OSQP_INFTY);
            
                    time_status_.setZero();
                }
                /**
                 * @brief Solve the QP problem.
                 * @param sol          (Eigen::MatrixXd) Output solution matrix.
                 * @param time_status  (TimeDuration) Output time durations structure for the QP solving process.
                 * @return (bool) True if the QP was solved successfully.
                 */
                bool solveQP(MatrixXd &sol, TimeDuration &time_status)
                {
                    timer_.reset();
                    setCost();
                    time_status.set_cost = timer_.elapsedAndReset();
                    if(nbc_ != 0) setBoundConstraint();
                    time_status.set_bound = timer_.elapsedAndReset();
                    if(nineqc_ != 0) setIneqConstraint();
                    time_status.set_ineq = timer_.elapsedAndReset();
                    if(neqc_ != 0) setEqConstraint();
                    time_status.set_eq = timer_.elapsedAndReset();
                    setConstraint();
                    time_status.set_constraint = timer_.elapsedAndReset();
                    time_status.set_qp = time_status.set_cost + time_status.set_bound + time_status.set_ineq + time_status.set_eq + time_status.set_constraint; 
                    // std::cout << "P:\n" << P_ds_<<std::endl;
                    // std::cout << "q: " << q_ds_.transpose()<<std::endl;
                    // std::cout << "l: " << l_ds_.transpose()<<std::endl;
                    // std::cout << "u: " << u_ds_.transpose()<<std::endl;
                    // std::cout << "A:\n " << A_ds_ << std::endl;
                    /* 
                    min   1/2 x' P x + q' x
                    x
            
                    subject to
                    l <= A x <= u
            
                    with :
                    P sparse (nx x nx) positive definite
                    q dense  (nx x 1)
                    A sparse (nc x n)
                    l dense (nc x 1)
                    u dense (nc x 1)
                    */
                    P_ds_ = 0.5 * (P_ds_ + P_ds_.transpose()).eval();
                    if (!P_ds_.allFinite() || !q_ds_.allFinite() ||
                        !A_ds_.allFinite() || !l_ds_.allFinite() || !u_ds_.allFinite())
                    {
                        std::cerr << "[QP] FAILED: NaN/Inf in QP matrices"
                                  << " P=" << !P_ds_.allFinite()
                                  << " q=" << !q_ds_.allFinite()
                                  << " A=" << !A_ds_.allFinite()
                                  << " l=" << !l_ds_.allFinite()
                                  << " u=" << !u_ds_.allFinite() << std::endl;
                        last_primal_.setZero(nx_);
                        last_dual_.setZero(nc_);
                        warm_start_ready_ = false;
                        return false;
                    }

                    SparseMatrix<double> P = P_ds_.sparseView();
                    SparseMatrix<double> A = A_ds_.sparseView();

                    if (!solver_initialized_)
                    {
                        // settings
                        solver_.settings()->setWarmStart(true);
                        // solver_.settings()->getSettings()->eps_abs = 1e-4;
                        // solver_.settings()->getSettings()->eps_rel = 1e-5;
                        solver_.settings()->getSettings()->verbose = false;
                        solver_.settings()->getSettings()->time_limit = 3e-4;

                        // set the initial data of the QP solver
                        solver_.data()->setNumberOfVariables(nx_);
                        solver_.data()->setNumberOfConstraints(nc_);
                        if (!solver_.data()->setHessianMatrix(P))           { std::cerr << "[QP] FAILED: setHessianMatrix" << std::endl; return false; }
                        if (!solver_.data()->setGradient(q_ds_))            { std::cerr << "[QP] FAILED: setGradient" << std::endl; return false; }
                        if (!solver_.data()->setLinearConstraintsMatrix(A)) { std::cerr << "[QP] FAILED: setLinearConstraintsMatrix" << std::endl; return false; }
                        if (!solver_.data()->setLowerBound(l_ds_))          { std::cerr << "[QP] FAILED: setLowerBound" << std::endl; return false; }
                        if (!solver_.data()->setUpperBound(u_ds_))          { std::cerr << "[QP] FAILED: setUpperBound" << std::endl; return false; }

                        // instantiate the solver
                        if (!solver_.initSolver()) { std::cerr << "[QP] FAILED: initSolver" << std::endl; return false; }
                        solver_initialized_ = true;
                    }
                    else
                    {
                        if (!solver_.updateHessianMatrix(P))           { std::cerr << "[QP] FAILED: updateHessianMatrix" << std::endl; return false; }
                        if (!solver_.updateGradient(q_ds_))            { std::cerr << "[QP] FAILED: updateGradient" << std::endl; return false; }
                        if (!solver_.updateLinearConstraintsMatrix(A)) { std::cerr << "[QP] FAILED: updateLinearConstraintsMatrix" << std::endl; return false; }
                        if (!solver_.updateBounds(l_ds_, u_ds_))       { std::cerr << "[QP] FAILED: updateBounds" << std::endl; return false; }
                    }

                    if (warm_start_ready_ &&
                        last_primal_.size() == nx_ &&
                        last_dual_.size() == nc_ &&
                        last_primal_.allFinite() &&
                        last_dual_.allFinite())
                    {
                        solver_.setWarmStart(last_primal_, last_dual_);
                    }
                    else
                    {
                        warm_start_ready_ = false;
                    }

                    time_status.set_solver = timer_.elapsedAndReset();

                    // solve the QP problem
                    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
                    {
                        std::cerr << "[QP] FAILED: solveProblem returned error" << std::endl;
                        last_primal_.setZero(nx_);
                        last_dual_.setZero(nc_);
                        return false;
                    }
                    qp_status_ = solver_.getStatus();
                    if (qp_status_ != OsqpEigen::Status::Solved &&
                        qp_status_ != OsqpEigen::Status::SolvedInaccurate &&
                        qp_status_ != OsqpEigen::Status::TimeLimitReached)
                    {
                        std::cerr << "[QP] FAILED: status=" << statusToString(qp_status_) << std::endl;
                        last_primal_.setZero(nx_);
                        last_dual_.setZero(nc_);
                        warm_start_ready_ = false;
                        return false;
                    }
                    if (qp_status_ == OsqpEigen::Status::TimeLimitReached)
                        std::cerr << "[QP] WARNING: TimeLimitReached, using best-effort solution" << std::endl;

                    time_status.solve_qp = timer_.elapsedAndReset();

                    sol = solver_.getSolution();
                    const VectorXd dual_sol = solver_.getDualSolution();

                    if (sol.rows() != nx_ || sol.cols() != 1 || !sol.allFinite() ||
                        dual_sol.size() != nc_ || !dual_sol.allFinite())
                    {
                        std::cerr << "[QP] FAILED: solution is non-finite or wrong size"
                                  << " sol_finite=" << sol.allFinite()
                                  << " dual_finite=" << dual_sol.allFinite() << std::endl;
                        last_primal_.setZero(nx_);
                        last_dual_.setZero(nc_);
                        warm_start_ready_ = false;
                        return false;
                    }

                    last_primal_ = sol;
                    last_dual_ = dual_sol;
                    warm_start_ready_ = true;
            
                    return true;
                }
            
            private:
                static const char* statusToString(OsqpEigen::Status s)
                {
                    switch(s)
                    {
                        case OsqpEigen::Status::Solved:                      return "Solved";
                        case OsqpEigen::Status::SolvedInaccurate:            return "SolvedInaccurate";
                        case OsqpEigen::Status::PrimalInfeasible:            return "PrimalInfeasible";
                        case OsqpEigen::Status::PrimalInfeasibleInaccurate:  return "PrimalInfeasibleInaccurate";
                        case OsqpEigen::Status::DualInfeasible:              return "DualInfeasible";
                        case OsqpEigen::Status::DualInfeasibleInaccurate:    return "DualInfeasibleInaccurate";
                        case OsqpEigen::Status::MaxIterReached:              return "MaxIterReached";
                        case OsqpEigen::Status::TimeLimitReached:            return "TimeLimitReached";
                        case OsqpEigen::Status::NonCvx:                      return "NonCvx";
                        default:                                              return "Unknown";
                    }
                }
                /**
                 * @brief Set the cost function for the QP problem.
                 */
                virtual void setCost() = 0;
                /**
                 * @brief Set the bound constraints such as control input limits for the QP problem.
                 */
                virtual void setBoundConstraint() = 0;
                /**
                 * @brief Set the inequality constraints such as joint angle limits for the QP problem.
                 */
                virtual void setIneqConstraint() = 0;
                /**
                 * @brief Set the equality constraints such as equations of motion for the QP problem.
                 */
                virtual void setEqConstraint() = 0;
                /**
                 * @brief Cumulate constraints to solve the QP problem.
                 */
                void setConstraint()
                {
                    // Bound Constraint
                    if(nbc_ != 0)
                    {
                        A_ds_.block(0,0,nbc_,nx_).setIdentity();
                        l_ds_.segment(0,nbc_) = l_bound_ds_;
                        u_ds_.segment(0,nbc_) = u_bound_ds_;
                    }
            
                    // Ineqaulity Constraint
                    if(nineqc_ != 0)
                    {
                        A_ds_.block(nbc_,0,nineqc_,nx_) = A_ineq_ds_;
                        l_ds_.segment(nbc_,nineqc_) = l_ineq_ds_;
                        u_ds_.segment(nbc_,nineqc_) = u_ineq_ds_;
                    }
            
                    // Eqaulity Constraint
                    if(neqc_ != 0)
                    {
                        A_ds_.block(nbc_+nineqc_,0,neqc_,nx_) = A_eq_ds_;
                        l_ds_.segment(nbc_+nineqc_,neqc_) = b_eq_ds_;
                        u_ds_.segment(nbc_+nineqc_,neqc_) = b_eq_ds_;
                    }
                }
            
            protected:
                int nx_;     // number of decision variables
                int nc_;     // number of constraints (ineq + bound + eq)
                int nbc_;    // number of bound constraints
                int nineqc_; // number of inequality constraints
                int neqc_;   // number of equality constraints

                MatrixXd P_ds_;                 // Hessian matrix
                VectorXd q_ds_;                 // Gradient vector

                MatrixXd A_ds_;                 // Constraint matrix
                VectorXd l_ds_;                 // Lower bounds for constraints
                VectorXd u_ds_;                 // Upper bounds for constraints

                MatrixXd A_ineq_ds_;            // Inequality constraint matrix
                VectorXd l_ineq_ds_;            // Lower bounds for inequality constraints
                VectorXd u_ineq_ds_;            // Upper bounds for inequality constraints

                VectorXd l_bound_ds_;           // Lower bounds for bound constraints
                VectorXd u_bound_ds_;           // Upper bounds for bound constraints

                MatrixXd A_eq_ds_;              // Equality constraint matrix
                VectorXd b_eq_ds_;              // Bounds for equality constraints


                OsqpEigen::Status qp_status_;   // Status of the QP solver
                SuhanBenchmark timer_;          // Timer for benchmarking
                TimeDuration time_status_;      // Time durations structure for the QP solving process

                OsqpEigen::Solver solver_;      // Persistent solver instance to avoid re-init
                bool solver_initialized_;       // True if solver has been initialized at least once

                VectorXd last_primal_;          // Primal solution from the previous solve (used for warm start)
                VectorXd last_dual_;            // Dual solution from the previous solve (used for warm start)
                bool warm_start_ready_;         // True if warm start data is available from a prior solve
        };
    } // namespace QP
} // namespace drc
