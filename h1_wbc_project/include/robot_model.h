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
