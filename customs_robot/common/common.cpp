#include "common.h"
namespace Common{

Pose::Pose()
{
    SetZero();
}
Pose::Pose(double x, double y, double z\
     , double qx, double qy, double qz, double qw)
{
    SetPose(x,y,z,qx,qy,qz,qw);
}

Pose::Pose(double x, double y, double z\
     , double pitch, double roll, double yaw)
{
    SetPose(x,y,z,pitch,roll,yaw);
}
Pose::~Pose(){}

void Pose::SetPose(double x, double y, double z\
             , double qx, double qy, double qz, double qw)
{
    x_ = x;
    y_ = y;
    z_ = z;
    qx_ = qx;
    qy_ = qy;
    qz_ = qz;
    qw_ = qw;
    CreateEulerData();
}

void Pose::SetPose(double x, double y, double z\
             , double pitch, double roll, double yaw)
{
    x_ = x;
    y_ = y;
    z_ = z;
    pitch_ = pitch;
    roll_ = roll;
    yaw_ = yaw;
    CreateQuaternionData();
}

void Pose::SetZero()
{
    x_ = 0.0;
    y_ = 0.0;
    z_ = 0.0;
    qx_ = 0.0;
    qy_ = 0.0;
    qz_ = 0.0;
    qw_ = 0.0;
    pitch_ = 0.0;
    roll_ = 0.0;
    yaw_ = 0.0;
}


//通过四元素创建欧拉角数据
void Pose::CreateEulerData()
{
    double q0, q1, q2, q3;
    q0 = this->qw_;
    q1 = this->qx_;
    q2 = this->qy_;
    q3 = this->qz_;
    // 计算滚转角（roll）
    double sinr_cosp = 2 * (q0 * q1 + q2 * q3);
    double cosr_cosp = 1 - 2 * (q1 * q1 + q2 * q2);
    this->roll_ = std::atan2(sinr_cosp, cosr_cosp);

    // 计算俯仰角（pitch）
    double sinp = 2 * (q0 * q2 - q3 * q1);
    if (std::abs(sinp) >= 1)
        this->pitch_ = std::copysign(M_PI / 2, sinp); // 避免除零错误
    else
        this->pitch_ = std::asin(sinp);

    // 计算方位角（yaw）
    double siny_cosp = 2 * (q0 * q3 + q1 * q2);
    double cosy_cosp = 1 - 2 * (q2 * q2 + q3 * q3);
    this->yaw_ = std::atan2(siny_cosp, cosy_cosp);

    return;
}

//通过欧拉角创建四元素数据
void Pose::CreateQuaternionData()
{
    // 计算滚转角（roll）的一半
    double roll_half = this->roll_ * 0.5;
    double cos_roll_half = cos(roll_half);
    double sin_roll_half = sin(roll_half);

    // 计算俯仰角（pitch）的一半
    double pitch_half = this->pitch_ * 0.5;
    double cos_pitch_half = cos(pitch_half);
    double sin_pitch_half = sin(pitch_half);

    // 计算偏航角（yaw）的一半
    double yaw_half = this->yaw_ * 0.5;
    double cos_yaw_half = cos(yaw_half);
    double sin_yaw_half = sin(yaw_half);

    // 计算四元数的分量
    this->qw_ = cos_roll_half * cos_pitch_half * cos_yaw_half + sin_roll_half * sin_pitch_half * sin_yaw_half;
    this->qx_ = sin_roll_half * cos_pitch_half * cos_yaw_half - cos_roll_half * sin_pitch_half * sin_yaw_half;
    this->qy_ = cos_roll_half * sin_pitch_half * cos_yaw_half + sin_roll_half * cos_pitch_half * sin_yaw_half;
    this->qz_ = cos_roll_half * cos_pitch_half * sin_yaw_half - sin_roll_half * sin_pitch_half * cos_yaw_half;
    return;
}

int Pose::PoseTransform(const Pose& tf_pose)
{
    this->x_ += tf_pose.x_;
    this->y_ += tf_pose.y_;
    this->z_ += tf_pose.z_;

    float w1, x1, y1, z1;
    float w2, x2, y2, z2;
    w2 = this->qw_;
    x2 = this->qx_;
    y2 = this->qy_;
    z2 = this->qz_;

    w1 = tf_pose.qw_;
    x1 = tf_pose.qx_;
    y1 = tf_pose.qy_;
    z1 = tf_pose.qz_;

    this->qw_ = w1*w2 - x1*x2 - y1*y2 - z1*z2;
    this->qx_ = w1*x2 + x1*w2 + y1*z2 - z1*y2;
    this->qy_ = w1*y2 - x1*z2 + y1*w2 + z1*x2;
    this->qz_ = w1*z2 + x1*y2 - y1*x2 + z1*w2;

    CreateEulerData();
    return 0;
}

void Pose::TransformPose(Eigen::Matrix4d& transformMatrix)
{
    Eigen::Vector3d position(x_, y_, z_);
    Eigen::Quaterniond orientation(qx_, qy_, qz_, qw_);
    //将位置和姿态转换为齐次变换矩阵
    Eigen::Matrix4d poseMatrix = Eigen::Matrix4d::Identity();
    poseMatrix.block<3,3>(0,0) = orientation.toRotationMatrix();
    poseMatrix.block<3,1>(0,3) = position;

    //将齐次变换矩阵与转换矩阵相乘
    Eigen::Matrix4d resultMatrix = transformMatrix * poseMatrix;

    //从齐次变换矩阵中提取位置和姿态
    position = resultMatrix.block<3,1>(0,3);
    orientation = Eigen::Quaterniond(resultMatrix.block<3,3>(0,0));

    SetPose(position.transpose().x(), position.transpose().y(), position.transpose().z()
            ,orientation.x(), orientation.y()
            , orientation.z(), orientation.w());
}

}
