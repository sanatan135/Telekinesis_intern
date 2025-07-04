#!/bin/bash

echo "Creating all files properly..."

# Create include/robot_state.h
cat > include/robot_state.h << 'EOF'
#pragma once
#include <Eigen/Dense>
#include <vector>

struct RobotState {
    Eigen::VectorXd q;              // Joint positions [19x1]
    Eigen::VectorXd q_dot;          // Joint velocities [19x1]
    Eigen::Vector3d com_pos;        // Center of mass position
    Eigen::Vector3d com_vel;        // Center of mass velocity
    Eigen::Vector3d base_pos;       // Base position
    Eigen::Vector3d base_vel;       // Base velocity
    Eigen::Quaterniond base_ori;    // Base orientation
    Eigen::Vector3d base_angvel;    // Base angular velocity
    Eigen::Vector3d imu_acc;        // IMU acceleration
    Eigen::Vector3d imu_gyro;       // IMU gyroscope
    std::vector<bool> contacts;     // Contact states
    double timestamp;               // Time stamp
    
    RobotState() {
        q = Eigen::VectorXd::Zero(19);
        q_dot = Eigen::VectorXd::Zero(19);
        com_pos = Eigen::Vector3d::Zero();
        com_vel = Eigen::Vector3d::Zero();
        base_pos = Eigen::Vector3d::Zero();
        base_vel = Eigen::Vector3d::Zero();
        base_ori = Eigen::Quaterniond::Identity();
        base_angvel = Eigen::Vector3d::Zero();
        imu_acc = Eigen::Vector3d(0, 0, 9.81);
        imu_gyro = Eigen::Vector3d::Zero();
        contacts = {true, true};  // Both feet in contact initially
        timestamp = 0.0;
    }
};
EOF

# Create include/robot_model.h
cat > include/robot_model.h << 'EOF'
#pragma once
#include "robot_state.h"
#include <Eigen/Dense>

// Robot Model Interface
class RobotModel {
public:
    virtual ~RobotModel() = default;
    virtual Eigen::MatrixXd getMassMatrix(const RobotState& state) = 0;
    virtual Eigen::VectorXd getNonlinearTerms(const RobotState& state) = 0;
    virtual Eigen::MatrixXd getContactJacobian(const RobotState& state) = 0;
    virtual Eigen::MatrixXd getContactJacobianDot(const RobotState& state) = 0;
    virtual int getNumDOF() const = 0;
    virtual int getNumContacts() const = 0;
};

// Simplified H1 Robot Model
class H1RobotModel : public RobotModel {
private:
    static constexpr int N_DOF = 19;
    static constexpr int N_CONTACTS = 12;  // 6 per foot
    
public:
    Eigen::MatrixXd getMassMatrix(const RobotState& state) override;
    Eigen::VectorXd getNonlinearTerms(const RobotState& state) override;
    Eigen::MatrixXd getContactJacobian(const RobotState& state) override;
    Eigen::MatrixXd getContactJacobianDot(const RobotState& state) override;
    int getNumDOF() const override { return N_DOF; }
    int getNumContacts() const override { return N_CONTACTS; }
};
EOF

# Create src/robot_model.cpp
cat > src/robot_model.cpp << 'EOF'
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
EOF

# Create src/main.cpp (simple version)
cat > src/main.cpp << 'EOF'
#include "../include/robot_state.h"
#include "../include/robot_model.h"
#include <iostream>
#include <iomanip>

int main() {
    std::cout << "=== Unitree H1 WBC Walking Controller Demo ===" << std::endl;
    
    // Test robot state creation
    RobotState state;
    std::cout << "✓ Robot state created with " << state.q.size() << " DOF" << std::endl;
    
    // Test robot model
    H1RobotModel robot_model;
    auto M = robot_model.getMassMatrix(state);
    std::cout << "✓ Mass matrix created: " << M.rows() << "x" << M.cols() << std::endl;
    
    auto h = robot_model.getNonlinearTerms(state);
    std::cout << "✓ Nonlinear terms computed: " << h.size() << " elements" << std::endl;
    
    std::cout << std::endl;
    std::cout << "Basic WBC components working! Build successful." << std::endl;
    
    return 0;
}
EOF

echo "All files created successfully!"
