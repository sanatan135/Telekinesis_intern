#include "../include/wbc_controller.h"
#include <iostream>

WBCController::WBCController(std::unique_ptr<RobotModel> robot_model) 
    : robot_model_(std::move(robot_model)) {
    
    n_dof_ = robot_model_->getNumDOF();
    n_contacts_ = robot_model_->getNumContacts();
    total_vars_ = n_dof_ + n_contacts_;
    
    qp_solver_ = std::make_unique<SimpleQPSolver>();
    
    // Initialize QP matrices
    Q_.resize(total_vars_, total_vars_);
    c_.resize(total_vars_);
    x_min_.resize(total_vars_);
    x_max_.resize(total_vars_);
    
    // Set bounds
    x_min_.head(n_dof_).setConstant(-100.0);  // Joint acceleration limits
    x_max_.head(n_dof_).setConstant(100.0);
    x_min_.tail(n_contacts_).setConstant(-1000.0);  // Contact force limits
    x_max_.tail(n_contacts_).setConstant(1000.0);
}

void WBCController::addTask(std::unique_ptr<WBCTask> task) {
    tasks_.push_back(std::move(task));
}

void WBCController::updateTasks(const RobotState& state) {
    for (auto& task : tasks_) {
        task->update(state);
    }
}

bool WBCController::solve(const RobotState& state, Eigen::VectorXd& q_ddot, 
                         Eigen::VectorXd& contact_forces) {
    // Update all tasks
    updateTasks(state);
    
    // Setup QP problem
    setupQP(state);
    
    // Solve QP
    Eigen::VectorXd x_opt;
    bool success = qp_solver_->solve(Q_, c_, A_eq_, b_eq_, x_min_, x_max_, x_opt);
    
    if (success) {
        q_ddot = x_opt.head(n_dof_);
        contact_forces = x_opt.tail(n_contacts_);
    } else {
        std::cerr << "WBC QP solve failed!" << std::endl;
        q_ddot = Eigen::VectorXd::Zero(n_dof_);
        contact_forces = Eigen::VectorXd::Zero(n_contacts_);
    }
    
    return success;
}

void WBCController::setupQP(const RobotState& state) {
    // Reset matrices
    Q_.setZero();
    c_.setZero();
    
    // Count total task dimensions
    int total_task_dim = 0;
    for (const auto& task : tasks_) {
        total_task_dim += task->getDimension();
    }
    
    // Build task stack matrices
    Eigen::MatrixXd A_task_stack(total_task_dim, total_vars_);
    Eigen::VectorXd b_task_stack(total_task_dim);
    
    int row_offset = 0;
    for (const auto& task : tasks_) {
        const auto& J = task->getJacobian();
        const auto& J_dot = task->getJacobianDot();
        const auto& x_ddot_des = task->getDesiredAcceleration();
        double weight = task->getWeight();
        int dim = task->getDimension();
        
        // Task constraint: sqrt(w) * [J, 0] * [q_ddot; f_c] = sqrt(w) * (x_ddot_des - J_dot * q_dot)
        A_task_stack.block(row_offset, 0, dim, n_dof_) = std::sqrt(weight) * J;
        A_task_stack.block(row_offset, n_dof_, dim, n_contacts_).setZero();
        
        b_task_stack.segment(row_offset, dim) = std::sqrt(weight) * (x_ddot_des - J_dot * state.q_dot);
        
        row_offset += dim;
    }
    
    // Build QP cost: minimize ||A_task_stack * x - b_task_stack||^2 + mu * ||f_c||^2
    Q_ = A_task_stack.transpose() * A_task_stack;
    c_ = -A_task_stack.transpose() * b_task_stack;
    
    // Add regularization for contact forces
    Q_.bottomRightCorner(n_contacts_, n_contacts_) += 
        mu_reg_ * Eigen::MatrixXd::Identity(n_contacts_, n_contacts_);
    
    // Setup equality constraints
    setupEqualityConstraints(state);
}

void WBCController::setupEqualityConstraints(const RobotState& state) {
    // Get robot dynamics matrices
    const Eigen::MatrixXd& M = robot_model_->getMassMatrix(state);
    const Eigen::VectorXd& h = robot_model_->getNonlinearTerms(state);
    const Eigen::MatrixXd& J_c = robot_model_->getContactJacobian(state);
    const Eigen::MatrixXd& J_c_dot = robot_model_->getContactJacobianDot(state);
    
    // Dynamics constraint: M * q_ddot - J_c^T * f_c = -h
    Eigen::MatrixXd A_dynamics(n_dof_, total_vars_);
    A_dynamics.leftCols(n_dof_) = M;
    A_dynamics.rightCols(n_contacts_) = -J_c.transpose();
    Eigen::VectorXd b_dynamics = -h;
    
    // Contact constraint: J_c * q_ddot + J_c_dot * q_dot = 0
    Eigen::MatrixXd A_contact(n_contacts_, total_vars_);
    A_contact.leftCols(n_dof_) = J_c;
    A_contact.rightCols(n_contacts_).setZero();
    Eigen::VectorXd b_contact = -J_c_dot * state.q_dot;
    
    // Stack equality constraints
    int n_eq = n_dof_ + n_contacts_;
    A_eq_.resize(n_eq, total_vars_);
    b_eq_.resize(n_eq);
    
    A_eq_.topRows(n_dof_) = A_dynamics;
    A_eq_.bottomRows(n_contacts_) = A_contact;
    
    b_eq_.head(n_dof_) = b_dynamics;
    b_eq_.tail(n_contacts_) = b_contact;
}
