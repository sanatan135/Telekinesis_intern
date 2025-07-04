#pragma once
#include <Eigen/Dense>

// QP Solver Interface
class QPSolver {
public:
    virtual ~QPSolver() = default;
    virtual bool solve(const Eigen::MatrixXd& Q, const Eigen::VectorXd& c,
                      const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq,
                      const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max,
                      Eigen::VectorXd& x_opt) = 0;
};

// Simple QP Solver using pseudoinverse
class SimpleQPSolver : public QPSolver {
public:
    bool solve(const Eigen::MatrixXd& Q, const Eigen::VectorXd& c,
              const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq,
              const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max,
              Eigen::VectorXd& x_opt) override;
};
