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
