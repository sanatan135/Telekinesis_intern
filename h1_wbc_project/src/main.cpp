#include "../include/robot_state.h"
#include "../include/robot_model.h"
#include "../include/wbc_tasks.h"
#include "../include/wbc_controller.h"
#include <iostream>
#include <iomanip>
#include <memory>

int main() {
    std::cout << "=== Unitree H1 WBC Walking Controller Demo ===" << std::endl;
    
    // Create robot state
    RobotState state;
    state.com_pos = Eigen::Vector3d(0.0, 0.0, 0.8);  // 0.8m height
    std::cout << "✓ Robot state initialized" << std::endl;
    
    // Create robot model
    auto robot_model = std::make_unique<H1RobotModel>();
    std::cout << "✓ Robot model created" << std::endl;
    
    // Create WBC controller
    WBCController wbc_controller(std::move(robot_model));
    std::cout << "✓ WBC controller created" << std::endl;
    
    // Create and add tasks
    auto com_task = std::make_unique<COMTask>(1000.0);
    com_task->setDesiredTrajectory(
        Eigen::Vector3d(0.1, 0.0, 0.8),  // desired position
        Eigen::Vector3d(0.1, 0.0, 0.0),  // desired velocity
        Eigen::Vector3d(0.0, 0.0, 0.0)   // desired acceleration
    );
    
    Eigen::VectorXd nominal_pose = Eigen::VectorXd::Zero(19);
    auto posture_task = std::make_unique<PostureTask>(nominal_pose, 1.0);
    
    wbc_controller.addTask(std::move(com_task));
    wbc_controller.addTask(std::move(posture_task));
    std::cout << "✓ Tasks added to WBC controller" << std::endl;
    
    // Test WBC solve
    Eigen::VectorXd q_ddot, contact_forces;
    bool success = wbc_controller.solve(state, q_ddot, contact_forces);
    
    if (success) {
        std::cout << "✓ WBC solve successful!" << std::endl;
        std::cout << "  Joint accelerations norm: " << q_ddot.norm() << std::endl;
        std::cout << "  Contact forces norm: " << contact_forces.norm() << std::endl;
    } else {
        std::cout << "✗ WBC solve failed!" << std::endl;
    }
    
    // Simple simulation loop
    std::cout << std::endl << "Running short simulation..." << std::endl;
    double dt = 0.01;  // 10ms
    
    for (int i = 0; i < 100; ++i) {  // 1 second
        // Update state (simple integration)
        state.com_pos(0) += 0.001;  // Move forward slowly
        state.timestamp += dt;
        
        // Solve WBC
        if (wbc_controller.solve(state, q_ddot, contact_forces)) {
            if (i % 20 == 0) {  // Print every 200ms
                std::cout << "t=" << std::fixed << std::setprecision(2) << state.timestamp 
                         << "s, CoM_x=" << std::setprecision(3) << state.com_pos(0) 
                         << "m, |q_ddot|=" << std::setprecision(2) << q_ddot.norm() << std::endl;
            }
        }
    }
    
    std::cout << std::endl << "=== WBC Demo Completed Successfully! ===" << std::endl;
    return 0;
}
