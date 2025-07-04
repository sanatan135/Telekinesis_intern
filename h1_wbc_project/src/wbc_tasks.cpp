#include "../include/wbc_tasks.h"
#include <cmath>
#include <iostream>

// COMTask Implementation
COMTask::COMTask(double weight) : weight_(weight) {
    jacobian_.resize(3, 19);
    jacobian_dot_.resize(3, 19);
    desired_pos_.setZero();
    desired_vel_.setZero();
    desired_acc_.setZero();
}

void COMTask::setDesiredTrajectory(const Eigen::Vector3d& pos, 
                                  const Eigen::Vector3d& vel, 
                                  const Eigen::Vector3d& acc) {
    desired_pos_ = pos;
    desired_vel_ = vel;
    desired_acc_ = acc;
}

void COMTask::update(const RobotState& state) {
    current_pos_ = state.com_pos;
    current_vel_ = state.com_vel;
    
    // Compute desired acceleration with PD control
    Eigen::Vector3d pos_error = desired_pos_ - current_pos_;
    Eigen::Vector3d vel_error = desired_vel_ - current_vel_;
    
    desired_acc_ = desired_acc_ + kp_ * pos_error + kd_ * vel_error;
    
    // Update Jacobian
    updateJacobian(state);
}

void COMTask::updateJacobian(const RobotState& state) {
    // Simplified CoM Jacobian (in practice, compute from robot model)
    jacobian_.setZero();
    jacobian_dot_.setZero();
    
    // Simple approximation: CoM is affected by all joints
    for (int i = 0; i < 19; ++i) {
        jacobian_(0, i) = 0.1 * std::cos(state.q(i));  // X direction
        jacobian_(1, i) = 0.1 * std::sin(state.q(i));  // Y direction  
        jacobian_(2, i) = 0.05;                         // Z direction
    }
}

// PostureTask Implementation
PostureTask::PostureTask(const Eigen::VectorXd& nominal_pose, double weight) 
    : desired_q_(nominal_pose), weight_(weight) {
    int n_dof = desired_q_.size();
    jacobian_ = Eigen::MatrixXd::Identity(n_dof, n_dof);
    jacobian_dot_ = Eigen::MatrixXd::Zero(n_dof, n_dof);
    desired_q_dot_ = Eigen::VectorXd::Zero(n_dof);
    desired_q_ddot_ = Eigen::VectorXd::Zero(n_dof);
}

void PostureTask::update(const RobotState& state) {
    Eigen::VectorXd pos_error = desired_q_ - state.q;
    Eigen::VectorXd vel_error = desired_q_dot_ - state.q_dot;
    desired_q_ddot_ = kp_ * pos_error + kd_ * vel_error;
}
