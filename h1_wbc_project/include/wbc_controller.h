#pragma once
#include "robot_state.h"
#include "wbc_tasks.h"
#include "qp_solver.h"
#include "robot_model.h"
#include <memory>
#include <vector>

class WBCController {
private:
    std::vector<std::unique_ptr<WBCTask>> tasks_;
    std::unique_ptr<QPSolver> qp_solver_;
    std::unique_ptr<RobotModel> robot_model_;
    
    // QP matrices
    int n_dof_;
    int n_contacts_;
    int total_vars_;
    
    Eigen::MatrixXd Q_;
    Eigen::VectorXd c_;
    Eigen::MatrixXd A_eq_;
    Eigen::VectorXd b_eq_;
    Eigen::VectorXd x_min_;
    Eigen::VectorXd x_max_;
    
    double mu_reg_ = 1e-4;
    
public:
    WBCController(std::unique_ptr<RobotModel> robot_model);
    void addTask(std::unique_ptr<WBCTask> task);
    void updateTasks(const RobotState& state);
    bool solve(const RobotState& state, Eigen::VectorXd& q_ddot, 
               Eigen::VectorXd& contact_forces);
    
private:
    void setupQP(const RobotState& state);
    void setupEqualityConstraints(const RobotState& state);
};
