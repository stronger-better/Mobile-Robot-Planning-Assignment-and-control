#include <iostream>
#include <so3_control/SO3Control.h>

#include <ros/ros.h>

SO3Control::SO3Control()
  : mass_(0.5)
  , g_(9.81)
  , attitude_initialized_(false)
{
  acc_.setZero();
  current_angular_vel_.setZero();
  
  // 根据四旋翼参数设置默认增益
  kR_ = Eigen::Vector3d(5.0, 5.0, 3.0);     // 基于转动惯量调整
  kOmega_ = Eigen::Vector3d(0.5, 0.5, 0.2); // 基于电机响应特性调整
  torque_.setZero();
}

void
SO3Control::setMass(const double mass)
{
  mass_ = mass;
}

void
SO3Control::setGravity(const double g)
{
  g_ = g;
}

void
SO3Control::setPosition(const Eigen::Vector3d& position)
{
  pos_ = position;
}

void
SO3Control::setVelocity(const Eigen::Vector3d& velocity)
{
  vel_ = velocity;
}

void
SO3Control::setAttitude(const Eigen::Quaterniond& attitude) {
    current_attitude_ = attitude;
    current_rotation_ = quaternionToRotation(attitude);
}

void SO3Control::setAngularVelocity(const Eigen::Vector3d& angular_vel) {
    current_angular_vel_ = angular_vel;
}

void SO3Control::setAttitudeGains(const double kR[3], const double kOm[3]) {
    kR_ = Eigen::Vector3d(kR[0], kR[1], kR[2]);
    kOmega_ = Eigen::Vector3d(kOm[0], kOm[1], kOm[2]);
}

Eigen::Vector3d SO3Control::calculateAttitudeError(const Eigen::Matrix3d& R_des)
{
    if (!attitude_initialized_) {
        return Eigen::Vector3d::Zero();
    }
    
    // 计算姿态误差
    Eigen::Matrix3d eR_mat = 0.5 * (R_des.transpose() * current_rotation_ - 
                                   current_rotation_.transpose() * R_des);
    return Eigen::Vector3d(eR_mat(2,1), eR_mat(0,2), eR_mat(1,0));
}

Eigen::Matrix3d SO3Control::quaternionToRotation(const Eigen::Quaterniond& q) {
    return q.toRotationMatrix();
}

void SO3Control::calculateDesiredRotation(const Eigen::Vector3d& force,
                                        const double des_yaw,
                                        Eigen::Matrix3d& R_des)
{
    const double MAX_TILT_ANGLE = 35.0 * M_PI / 180.0; // 最大倾角35度
    
    Eigen::Vector3d b3_des = force.normalized();
    
    // 限制倾角
    double tilt_angle = acos(b3_des.dot(Eigen::Vector3d(0, 0, 1)));
    if (tilt_angle > MAX_TILT_ANGLE) {
        Eigen::Vector3d tilt_axis = Eigen::Vector3d(0, 0, 1).cross(b3_des).normalized();
        Eigen::AngleAxisd rotation(MAX_TILT_ANGLE, tilt_axis);
        b3_des = rotation * Eigen::Vector3d(0, 0, 1);
    }

    Eigen::Vector3d b1_des(cos(des_yaw), sin(des_yaw), 0);
    Eigen::Vector3d b2_des = b3_des.cross(b1_des).normalized();
    b1_des = b2_des.cross(b3_des);
    
    R_des.col(0) = b1_des;
    R_des.col(1) = b2_des;
    R_des.col(2) = b3_des;
}

void
SO3Control::calculateControl(const Eigen::Vector3d& des_pos,
                             const Eigen::Vector3d& des_vel,
                             const Eigen::Vector3d& des_acc,
                             const double des_yaw,
                             const double des_yaw_dot,
                             const Eigen::Vector3d& kx,
                             const Eigen::Vector3d& kv,
                             const Eigen::Vector3d& kOmega)
{
    // 1. 计算位置控制力（考虑最大推力限制）
    const double MAX_THRUST = mass_ * 15.0;  // 假设最大推力是重力的15倍
    
    force_ = kx.array() * (des_pos - pos_).array() +
             kv.array() * (des_vel - vel_).array() +
             mass_ * des_acc.array() +
             mass_ * g_ * Eigen::Vector3d(0, 0, 1).array();
    
    // 推力限制
    double thrust_magnitude = force_.norm();
    if (thrust_magnitude > MAX_THRUST) {
        force_ *= MAX_THRUST / thrust_magnitude;
    }

    // 2. 计算期望姿态（考虑最大倾角）
    Eigen::Matrix3d R_des;
    calculateDesiredRotation(force_, des_yaw, R_des);

    // 3. 计算姿态误差
    Eigen::Vector3d eR = calculateAttitudeError(R_des);

    // 4. 计算期望角速度（考虑最大角速度）
    const double MAX_ANGULAR_VELOCITY = 3.0; // 最大角速度（rad/s）
    desired_angular_vel_ = Eigen::Vector3d(0, 0, des_yaw_dot);
    if (desired_angular_vel_.norm() > MAX_ANGULAR_VELOCITY) {
        desired_angular_vel_ *= MAX_ANGULAR_VELOCITY / desired_angular_vel_.norm();
    }

    // 5. 计算角速度误差
    Eigen::Vector3d eOmega = current_angular_vel_ - desired_angular_vel_;

    // 6. 计算控制力矩（考虑最大力矩）
    const double MAX_TORQUE = 1.0; // 根据电机特性设置最大力矩
    
    if (attitude_initialized_) {
        torque_ = -(kR_.array() * eR.array()) - (kOmega.array() * eOmega.array());
        
        // 力矩限制
        double torque_magnitude = torque_.norm();
        if (torque_magnitude > MAX_TORQUE) {
            torque_ *= MAX_TORQUE / torque_magnitude;
        }
    } else {
        torque_.setZero();
    }

    // 7. 更新姿态四元数
    orientation_ = Eigen::Quaterniond(R_des);

    // 添加调试输出
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Force: " << force_.transpose());
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Torque: " << torque_.transpose());
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Attitude error: " << eR.transpose());
    ROS_DEBUG_STREAM_THROTTLE(1.0, "Angular velocity error: " << eOmega.transpose());
}

const Eigen::Vector3d&
SO3Control::getComputedForce(void)
{
  return force_;
}

const Eigen::Quaterniond&
SO3Control::getComputedOrientation(void)
{
  return orientation_;
}

void
SO3Control::setAcc(const Eigen::Vector3d& acc)
{
  acc_ = acc;
}

const Eigen::Vector3d& SO3Control::getComputedTorque(void) {
    return torque_;
}

const Eigen::Vector3d& SO3Control::getComputedAngularVelocity(void) {
    return desired_angular_vel_;
}
