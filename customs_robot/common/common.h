#ifndef COMMON_H
#define COMMON_H
#include <math.h>
#include <Eigen/Dense>
#include <QDebug>
#include <bitset>

namespace Common{

#define PI       3.14159265358979323846   // pi

class Pose
{
public:
    Pose();

    Pose(double x, double y, double z\
         , double qx, double qy, double qz, double qw);

    Pose(double x, double y, double z\
         , double pitch, double roll, double yaw);
    ~Pose();

    void SetPose(double x, double y, double z\
                 , double qx, double qy, double qz, double qw);

    void SetPose(double x, double y, double z\
                 , double pitch, double roll, double yaw);

    void SetZero();

    //通过四元素创建欧拉角数据
    void CreateEulerData();

    //通过欧拉角创建四元素数据
    void CreateQuaternionData();

    int PoseTransform(const Pose& tf_pose);

    void TransformPose(Eigen::Matrix4d& transformMatrix);

public:
    double x_;
    double y_;
    double z_;
    double qx_;
    double qy_;
    double qz_;
    double qw_;
    double pitch_;
    double roll_;
    double yaw_;
};

//  二进制赋值
template <typename T>
bool BinaryAssign(T &value, int index, bool state)
{
    if (index > (int)(sizeof(value) * 8))
    {
        return false;
    }
    std::bitset<sizeof(value) * 8> temp(value);
    temp[index-1] = state;
    value = temp.to_ullong();
    return true;
}


//  获取二进制有效下标
template <typename T>
bool BinaryValidIndex(T &value, std::list<int>& valid_list)
{
    valid_list.clear();
    std::bitset<sizeof(value) * 8> temp(value);
    for (int i = 0; i < (int)(sizeof(value) * 8); i++)
    {
        if (temp[i] == true)
        {
            valid_list.push_back(i);
        }
    }
    return true;
}


// 货箱尺寸 By JF Gan
enum BoxPoseType
{
    Left  = 0,  //  左侧
    Middle = 1, //  中部
    Right = 2   //  右侧
};

class Box
{
public:
    Box()
    {
        Width = 0.0;
        Length = 0.0;
        Height = 0.0;
        BoxPoseType_ = BoxPoseType::Middle;
        bNeedCorrected_ = false;
    }

    Box(double length, double width, double height)
    {
        Width = width;
        Length = length;
        Height = height;
        BoxPoseType_ = BoxPoseType::Middle;
        bNeedCorrected_ = false;
    }
public:
    double Length;
    double Width;
    double Height;
    Pose LoadPose_;         // 装载位置
    Pose OperationPose_;    // 操作位置
    BoxPoseType BoxPoseType_;
    bool bNeedCorrected_;   // 需要纠正
    int iLabel_;
};

class Container
{
public:
    Container()
    {
        dInsideWidth_ = 0.0;
        dInsideLength_ = 0.0;
        dInsideHeight_ = 0.0;
    }

    Container(double length, double width, double height)
    {
        dInsideWidth_ = width;
        dInsideLength_ = length;
        dInsideHeight_ = height;
    }
public:
    double dInsideWidth_;   // 集装箱 内侧宽  mm
    double dInsideLength_;  // 集装箱 内侧长  mm
    double dInsideHeight_;  // 集装箱 内侧高  mm
};

class ROIRectangle
{
public:
    ROIRectangle()
    {
        dRoi_x_ = 0.0;
        dRoi_y_ = 0.0;
        dRoi_w_ = 0.0;
        dRoi_h_ = 0.0;
    }

    ROIRectangle(double roi_x, double roi_y, double roi_w, double roi_h)
    {
        dRoi_x_ = roi_x;
        dRoi_y_ = roi_y;
        dRoi_w_ = roi_w;
        dRoi_h_ = roi_h;
    }

    ~ROIRectangle(){}

    ROIRectangle& operator=(const ROIRectangle& data)
    {
        dRoi_x_ = data.dRoi_x_;
        dRoi_y_ = data.dRoi_y_;
        dRoi_w_ = data.dRoi_w_;
        dRoi_h_ = data.dRoi_h_;
        return *this;
    }

    bool operator==(const ROIRectangle& data)
    {
        if ((dRoi_x_ == data.dRoi_x_)
            && (dRoi_y_ == data.dRoi_y_)
            && (dRoi_w_ == data.dRoi_w_)
            && (dRoi_h_ == data.dRoi_h_))
        {
            return true;
        }
        return false;
    }

    bool operator!=(const ROIRectangle& data)
    {
        if ((dRoi_x_ == data.dRoi_x_)
            && (dRoi_y_ == data.dRoi_y_)
            && (dRoi_w_ == data.dRoi_w_)
            && (dRoi_h_ == data.dRoi_h_))
        {
            return false;
        }
        return true;
    }

    void SetROI(double roi_x, double roi_y, double roi_w, double roi_h)
    {
        dRoi_x_ = roi_x;
        dRoi_y_ = roi_y;
        dRoi_w_ = roi_w;
        dRoi_h_ = roi_h;
    }

public:
    double dRoi_x_;
    double dRoi_y_;
    double dRoi_w_;
    double dRoi_h_;
};

}
#endif // COMMON_H
