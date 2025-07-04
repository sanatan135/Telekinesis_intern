#pragma once
#include "robot_state.h"
#include <Eigen/Dense>
#include <memory>
#include <string>

// Abstract Task Class
class WBCTask {
public:
    virtual ~WBCTask() = default;
    virtual void update(const RobotState& state) = 0;
    virtual const Eigen::MatrixXd& getJacobian() const = 0;
    virtual const Eigen::MatrixXd& getJacobianDot() const = 0;
    virtual const Eigen::VectorXd& getDesiredAcceleration() const = 0;
    virtual double getWeight() const = 0;
    virtual int getDimension() const = 0;
    virtual std::string getName() const = 0;
};

// Center of Mass Task
class COMTask : public WBCTask {
private:
    Eigen::Vector3d desired_pos_;
    Eigen::Vector3d desired_vel_;
    Eigen::Vector3d desired_acc_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_dot_;
    double weight_;
    Eigen::Vector3d current_pos_;
    Eigen::Vector3d current_vel_;
    
    double kp_ = 400.0;
    double kd_ = 40.0;
    
public:
    COMTask(double weight = 1000.0);
    void setDesiredTrajectory(const Eigen::Vector3d& pos, 
                             const Eigen::Vector3d& vel, 
                             const Eigen::Vector3d& acc);
    void update(const RobotState& state) override;
    const Eigen::MatrixXd& getJacobian() const override { return jacobian_; }
    const Eigen::MatrixXd& getJacobianDot() const override { return jacobian_dot_; }
    const Eigen::VectorXd& getDesiredAcceleration() const override { return desired_acc_; }
    double getWeight() const override { return weight_; }
    int getDimension() const override { return 3; }
    std::string getName() const override { return "COM"; }
    
private:
    void updateJacobian(const RobotState& state);
};

// Posture Task for joint regularization
class PostureTask : public WBCTask {
private:
    Eigen::VectorXd desired_q_;
    Eigen::VectorXd desired_q_dot_;
    Eigen::VectorXd desired_q_ddot_;
    Eigen::MatrixXd jacobian_;
    Eigen::MatrixXd jacobian_dot_;
    double weight_;
    
    double kp_ = 50.0;
    double kd_ = 10.0;
    
public:
    PostureTask(const Eigen::VectorXd& nominal_pose, double weight = 1.0);
    void update(const RobotState& state) override;
    const Eigen::MatrixXd& getJacobian() const override { return jacobian_; }
    const Eigen::MatrixXd& getJacobianDot() const override { return jacobian_dot_; }
    const Eigen::VectorXd& getDesiredAcceleration() const override { return desired_q_ddot_; }
    double getWeight() const override { return weight_; }
    int getDimension() const override { return desired_q_.size(); }
    std::string getName() const override { return "posture"; }
};
