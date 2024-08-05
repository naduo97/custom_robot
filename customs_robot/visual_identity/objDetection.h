#include <iostream>
#include <fstream>
#include <cassert>
#include <float.h>
#include <algorithm>
#include <string>

#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <Eigen/StdVector>
#include <bits/stdc++.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h> 

#include "mmdeploy/detector.h"

class BoxPoseDetection {
  public:
    BoxPoseDetection(char const *model_path_temp, cv::Mat &color_img_temp, cv::Mat &depth_img_temp
                       , Eigen::Matrix3d K_temp, Eigen::VectorXd distCoeffs_temp
                       , float roi_x, float roi_y, float roi_w, float roi_h, float score_value);
    ~BoxPoseDetection();

    // function
    Eigen::Matrix3d eulerAnglesToRotationMatrix(Eigen::Vector3d euler_angle);
    static bool cmp(std::vector<double>&a,std::vector<double>&b);
    bool change_sign(bool sign);
    double get_depth(int u, int v);
    bool get_real_value(int u, int v, double &x, double &y, double &z);

      //预处理
      //预处理
    cv::Point2f roi_top_left_pt_;    //roi左上角像素点
    int roi_img_width_;
    int roi_img_height_;
    cv::Mat roi_rgb_image_;
    void getRoiImage(cv::Mat cur_frame_img);

    // sub_function
    std::vector<std::vector<double>> update_rotation_matrix(std::vector<std::vector<double>> bbox, std::vector<Eigen::Matrix4d> RT_matrix);
    std::vector<std::vector<double>> sort_in_range(int col_begin, int col_end, std::vector<std::vector<double>> bbox_after, bool sign);
    std::vector<std::vector<double>> get_3D_pose(std::vector<std::vector<double>> bbox_temp);

    // main process function
    std::vector<std::vector<double>> vis_pc_bboxuv_process(int* res_count, mmdeploy_detection_t* bboxes, std::vector<pcl::PointCloud<pcl::PointXYZ>> &objsPointCloud);
    std::vector<std::vector<double>> update_front_bbox(std::vector<std::vector<double>> bbox);
    std::vector<Eigen::Matrix4d> calculateGraspPose(std::vector<pcl::PointCloud<pcl::PointXYZ>> &objsPointCloud, std::vector<std::vector<double>> &bbox_temp);
    std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> box_tra_process(std::vector<std::vector<double>> bbox, double up_angle, double down_angle, double indentation, double inde_angle);

    std::queue<std::vector<double>> process(cv::Mat &img);

  private:
    std::string str_model_path_;
    std::string str_device_name_;
    char const *color_img_path_;
    char const *depth_img_path_;

    cv::Mat color_img_;
    cv::Mat depth_img_;

    Eigen::Matrix3d K_;
    cv::Mat K_m_;
    Eigen::VectorXd distCoeffs_ = Eigen::VectorXd::Zero(5);
    cv::Mat distCoeffs_m_;

    int bbox_num_ = 0;
    float fScoreValue_;
    void vertex_reorder(cv::Point2f vertex[]);   
};
