#include "../include/robot_model.h"
#include <cmath>

Eigen::MatrixXd H1RobotModel::getMassMatrix(const RobotState& state) {
    Eigen::MatrixXd M = Eigen::MatrixXd::Identity(N_DOF, N_DOF);
    
    // Simplified mass matrix with realistic values for H1
    std::vector<double> joint_inertias = {
        2.0, 2.0, 2.0,  // Torso joints
        1.5, 1.5, 0.8,  // Left leg (hip, knee, ankle)
        1.5, 1.5, 0.8,  // Right leg
        0.5, 0.5, 0.5,  // Left arm
        0.5, 0.5, 0.5,  // Right arm
        0.3, 0.3, 0.3, 0.2  // Remaining joints
    };
    
    for (int i = 0; i < N_DOF && i < (int)joint_inertias.size(); ++i) {
        M(i, i) = joint_inertias[i] + 0.1 * std::sin(state.q(i));
    }
    
    return M;
}

Eigen::VectorXd H1RobotModel::getNonlinearTerms(const RobotState& state) {
    Eigen::VectorXd h = Eigen::VectorXd::Zero(N_DOF);
    
    // Simplified gravity compensation
    std::vector<double> gravity_factors = {
        0.0, 0.0, 0.0,      // Torso (vertical joints)
        0.8, 1.2, 0.3,      // Left leg
        0.8, 1.2, 0.3,      // Right leg  
        0.2, 0.3, 0.1,      // Left arm
        0.2, 0.3, 0.1,      // Right arm
        0.1, 0.1, 0.1, 0.05 // Remaining
    };
    
    for (int i = 0; i < N_DOF && i < (int)gravity_factors.size(); ++i) {
        h(i) = gravity_factors[i] * 9.81 * std::cos(state.q(i));
    }
    
    // Add velocity-dependent terms (simplified Coriolis)
    for (int i = 0; i < N_DOF; ++i) {
        h(i) += 0.1 * state.q_dot(i) * state.q_dot(i);
    }
    
    return h;
}

Eigen::MatrixXd H1RobotModel::getContactJacobian(const RobotState& state) {
    Eigen::MatrixXd J_c = Eigen::MatrixXd::Zero(N_CONTACTS, N_DOF);
    
    // Simplified contact Jacobian for both feet
    for (int i = 0; i < N_CONTACTS; ++i) {
        for (int j = 0; j < N_DOF; ++j) {
            J_c(i, j) = 0.1 * std::sin(i + j + state.q(j));
        }
    }
    
    return J_c;
}

Eigen::MatrixXd H1RobotModel::getContactJacobianDot(const RobotState& state) {
    return Eigen::MatrixXd::Zero(N_CONTACTS, N_DOF);
}
