#include "../include/qp_solver.h"
#include <iostream>

bool SimpleQPSolver::solve(const Eigen::MatrixXd& Q, const Eigen::VectorXd& c,
                          const Eigen::MatrixXd& A_eq, const Eigen::VectorXd& b_eq,
                          const Eigen::VectorXd& x_min, const Eigen::VectorXd& x_max,
                          Eigen::VectorXd& x_opt) {
    
    int n_vars = Q.rows();
    int n_eq = A_eq.rows();
    
    try {
        // Solve using KKT system: [Q A_eq^T; A_eq 0] [x; lambda] = [-c; b_eq]
        Eigen::MatrixXd KKT(n_vars + n_eq, n_vars + n_eq);
        Eigen::VectorXd rhs(n_vars + n_eq);
        
        KKT.topLeftCorner(n_vars, n_vars) = Q;
        KKT.topRightCorner(n_vars, n_eq) = A_eq.transpose();
        KKT.bottomLeftCorner(n_eq, n_vars) = A_eq;
        KKT.bottomRightCorner(n_eq, n_eq).setZero();
        
        rhs.head(n_vars) = -c;
        rhs.tail(n_eq) = b_eq;
        
        // Solve KKT system
        Eigen::VectorXd solution = KKT.ldlt().solve(rhs);
        x_opt = solution.head(n_vars);
        
        // Apply box constraints (simple projection)
        for (int i = 0; i < n_vars; ++i) {
            x_opt(i) = std::max(x_min(i), std::min(x_max(i), x_opt(i)));
        }
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "QP Solver failed: " << e.what() << std::endl;
        x_opt = Eigen::VectorXd::Zero(n_vars);
        return false;
    }
}
