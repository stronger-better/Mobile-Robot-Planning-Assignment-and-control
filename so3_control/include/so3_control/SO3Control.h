#ifndef __SO3_CONTROL_H__
#define __SO3_CONTROL_H__

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <fstream>
#include <ros/ros.h>

class SO3Control
{
public:
  SO3Control();

  void setMass(const double mass);
  void setGravity(const double g);
  void setPosition(const Eigen::Vector3d& position);
  void setVelocity(const Eigen::Vector3d& velocity);
  void setAcc(const Eigen::Vector3d& acc);
  
  // 姿态控制相关函数
  void setAttitude(const Eigen::Quaterniond& attitude);
  void setAngularVelocity(const Eigen::Vector3d& angular_vel);
  void setAttitudeGains(const double kR[3], const double kOm[3]);
  
  void calculateControl(const Eigen::Vector3d& des_pos,
                       const Eigen::Vector3d& des_vel,
                       const Eigen::Vector3d& des_acc,
                       const double des_yaw,
                       const double des_yaw_dot,
                       const Eigen::Vector3d& kx,
                       const Eigen::Vector3d& kv,
                       const Eigen::Vector3d& kOmega);

  const Eigen::Vector3d&    getComputedForce(void);
  const Eigen::Quaterniond& getComputedOrientation(void);
  const Eigen::Vector3d&    getComputedTorque(void);
  const Eigen::Vector3d&    getComputedAngularVelocity(void);

  void setMaxAngularRate(double max_rate) { max_angular_rate_ = max_rate; }

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  // 基本参数
  double mass_;
  double g_;
  
  // 状态变量
  Eigen::Vector3d pos_;
  Eigen::Vector3d vel_;
  Eigen::Vector3d acc_;
  
  // 控制输出
  Eigen::Vector3d force_;
  Eigen::Quaterniond orientation_;
  Eigen::Vector3d torque_;
  
  // 姿态控制相关变量
  Eigen::Quaterniond current_attitude_;    // 当前姿态
  Eigen::Matrix3d current_rotation_;       // 当前旋转矩阵
  Eigen::Vector3d current_angular_vel_;    // 当前角速度
  Eigen::Vector3d desired_angular_vel_;    // 期望角速度
  bool attitude_initialized_;              // 姿态初始化标志
  
  // 控制增益
  Eigen::Vector3d kR_;     // 姿态误差增益
  Eigen::Vector3d kOmega_; // 角速度误差增益
  
  // 辅助函数
  Eigen::Vector3d calculateAttitudeError(const Eigen::Matrix3d& R_des);
  Eigen::Matrix3d quaternionToRotation(const Eigen::Quaterniond& q);
  void calculateDesiredRotation(const Eigen::Vector3d& force,
                              const double des_yaw,
                              Eigen::Matrix3d& R_des);

  double max_angular_rate_;
};

#endif
