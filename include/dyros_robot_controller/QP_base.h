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
                 * @brief Cosntructor.
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
                        if (!solver_.data()->setHessianMatrix(P))           return false;
                        if (!solver_.data()->setGradient(q_ds_))            return false;
                        if (!solver_.data()->setLinearConstraintsMatrix(A)) return false;
                        if (!solver_.data()->setLowerBound(l_ds_))          return false;
                        if (!solver_.data()->setUpperBound(u_ds_))          return false;

                        // instantiate the solver
                        if (!solver_.initSolver()) return false;
                        solver_initialized_ = true;
                    }
                    else
                    {
                        if (!solver_.updateHessianMatrix(P))           return false;
                        if (!solver_.updateGradient(q_ds_))            return false;
                        if (!solver_.updateLinearConstraintsMatrix(A)) return false;
                        if (!solver_.updateBounds(l_ds_, u_ds_))        return false;
                    }

                    if (warm_start_ready_)
                    {
                        solver_.setWarmStart(last_primal_, last_dual_);
                    }

                    time_status.set_solver = timer_.elapsedAndReset();

                    // solve the QP problem
                    if (solver_.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return false;
                    qp_status_ = solver_.getStatus();
                    // if (solver_.getStatus() != OsqpEigen::Status::Solved) return false;
                    // if (solver.getStatus() != OsqpStatus::Solved && solver.getStatus() != OsqpStatus::SolvedInaccurate) return false;
                    // TODO: near constraints, QP failed witihin timelimit
                     if (solver_.getStatus() != OsqpEigen::Status::Solved && solver_.getStatus() != OsqpEigen::Status::TimeLimitReached) return false;

                    time_status.solve_qp = timer_.elapsedAndReset();

                    sol = solver_.getSolution();

                    last_primal_ = sol;
                    last_dual_ = solver_.getDualSolution();
                    warm_start_ready_ = true;
            
                    return true;
                }
            
            private:
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
                bool solver_initialized_;

                VectorXd last_primal_;
                VectorXd last_dual_;
                bool warm_start_ready_;
        };
    } // namespace QP
} // namespace drc
