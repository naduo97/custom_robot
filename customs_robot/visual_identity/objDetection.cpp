#include <iostream>
#include "objDetection.h"

// 定义点云类型
typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloud;

EIGEN_DEFINE_STL_VECTOR_SPECIALIZATION(Eigen::Matrix4d);
// 构造函数
BoxPoseDetection::BoxPoseDetection(char const *model_path_temp, cv::Mat &color_img_temp, cv::Mat &depth_img_temp
                                   , Eigen::Matrix3d K_temp, Eigen::VectorXd distCoeffs_temp
                                   , float roi_x, float roi_y, float roi_w, float roi_h, float score_value)
{
  str_model_path_ = model_path_temp;
  str_device_name_ = "cuda";
  color_img_ = color_img_temp.clone();
  depth_img_ = depth_img_temp.clone();

  // obj face to base  5222
  roi_top_left_pt_.x = roi_x;
  roi_top_left_pt_.y = roi_y;
  roi_img_width_ = roi_w;
  roi_img_height_ = roi_h;
  
  fScoreValue_ = score_value;

  K_ = K_temp;
  distCoeffs_ = distCoeffs_temp;
  distCoeffs_m_ = (cv::Mat_<double>(1,5) << distCoeffs_temp(0),distCoeffs_temp(1),distCoeffs_temp(2),distCoeffs_temp(3),distCoeffs_temp(4));
  K_m_ = (cv::Mat_<double>(3,3) << K_temp(0,0),K_temp(0,1),K_temp(0,2),
                                   K_temp(1,0),K_temp(1,1),K_temp(1,2),
                                   K_temp(2,0),K_temp(2,1),K_temp(2,2));
}


BoxPoseDetection::~BoxPoseDetection()
{
}


/***************************************************************
  *  @brief     欧拉角转换为旋转矩阵
  *  @param     euler_angle   Eigen::Vector3d
  *  @note      欧拉角格式为 [rx,ry,rz] 为固定轴旋转方式
 **************************************************************/
Eigen::Matrix3d BoxPoseDetection::eulerAnglesToRotationMatrix(Eigen::Vector3d euler_angle){
  double rz = euler_angle(0), ry = euler_angle(1), rx = euler_angle(2);
  Eigen::Matrix3d R_x;
  R_x << 1,      0,        0,
         0,      cos(rx), -sin(rx),
         0,      sin(rx),  cos(rx);
  Eigen::Matrix3d R_y;
  R_y <<  cos(ry),      0,        sin(ry),
          0,            1,        0,
         -sin(ry),      0,        cos(ry);
  Eigen::Matrix3d R_z;
  R_z << cos(rz),      -sin(rz),       0,
         sin(rz),       cos(rz),       0,
         0,             0,             1;
  return R_z*R_y*R_x;
}

/*for sort */
bool BoxPoseDetection::cmp(std::vector<double>&a,std::vector<double>&b){
  return a[0]>b[0];
}


/*for sort */
bool BoxPoseDetection::change_sign(bool sign) {
  if (sign)
    sign = false;
  else sign = true;
  return sign;
}

/***************************************************************
  *  @brief     从像素坐标获取深度图的深度值
  *  @param     u  int 为深度图的u坐标     
  *  @param     v  int 为深度图的v坐标
  *  @result    depth   指定坐标的深度值
  *  @note      从指定的像素坐标位置上下左右5个像素进行深度值平均值计算，若周围皆无深度值，则报错
 **************************************************************/
double BoxPoseDetection::get_depth(int u, int v) {
  double true_depth=0;
  int count = 0;
  for(int m=(u-5);m<(u+5);m++){
    for(int n=(v-5);n<(v+5);n++){
      double d = static_cast<double>(depth_img_.at<ushort>(n, m));
      if(d-1<0) continue;
      else {
        true_depth = true_depth + d;
        count++;
      }
    }
  }
  double depth = true_depth/count;
  if (depth-1<0) std::cout<<"ERROR: NO DEPTH BACK AT ("<<u<<","<<v<<") POINT!\n";
  return depth;
}

bool BoxPoseDetection::get_real_value(int u, int v, double &x, double &y, double &z)
{
    double depth = get_depth(u,v);

    PointT p; // declare point p
    // conculate the 3D position
    p.z = depth * 0.0002500000118743628;
    p.x = (u - K_(0,2)) * p.z / K_(0,0);
    p.y = (v - K_(1,2)) * p.z / K_(1,1);
    x = p.x;
    y = p.y;
    z = p.z;
    return true;
}

/***************************************************************
  *  @brief     输入detector结果，获取到实际可用箱子数量、作可视化、获取点云，并得到bonding box的抓取点的初步坐标
  *  @param     res_count  int*     
  *  @param     bboxes     mmdeploy_detection_t*
  *  @param     objsPointCloud  std::vector<pcl::PointCloud<pcl::PointXYZ>>  获取到点云数据后并存储到该变量当中
  *  @result    bbox  具有11列的二维vector数组，bbox[index][0]为初步u坐标，bbox[index][1]为初步v坐标，bbox[index][2,3,4]为tcp的y轴在箱体位置相对于相机坐标系的向量(x,y,z)
 **************************************************************/
std::vector<std::vector<double>> BoxPoseDetection::vis_pc_bboxuv_process(int* res_count, mmdeploy_detection_t* bboxes, std::vector<pcl::PointCloud<pcl::PointXYZ>> &objsPointCloud) {
  std::vector<std::vector<double>> bbox_temp;
  for (int i = 0; i < *res_count; ++i) {
    const auto& box = bboxes[i].bbox;
    const auto& mask = bboxes[i].mask;

    std::cout << "CHECKING: " << i << ", score: " << bboxes[i].score << std::endl;
    // skip detections with invalid bbox size (bbox height or width < 1)
    if ((box.right - box.left) < 1 || (box.bottom - box.top) < 1) continue;

    // skip detections less than specified score threshold
    if (bboxes[i].score < fScoreValue_) continue;

    PointCloud::Ptr cloud(new PointCloud); // declare new cloud

    // generate mask overlay if model exports masks
    if (mask != nullptr) {
      cv::Mat imgMask(mask->height, mask->width, CV_8UC1, &mask->data[0]); // get the mask_img

      auto u_corner = std::max(std::floor(box.left) - 1, 0.f) + roi_top_left_pt_.x;
      auto v_corner = std::max(std::floor(box.top) - 1, 0.f) + roi_top_left_pt_.y;   //(u_corner, v_corner)raw_rgb_img图像中的左上角

      // draw mask
      cv::Rect roi((int)u_corner, (int)v_corner, mask->width, mask->height);
      cv::Mat ch[3];
      split(color_img_, ch);
      int col = 0;  // int col = i % 3;
      cv::bitwise_or(imgMask, ch[col](roi), ch[col](roi));
      merge(ch, 3, color_img_);

      // get pc from depth_img 
      for (int m = 0; m < mask->height; m++){
        for (int n = 0; n < mask->width; n++){
          int u = u_corner+n;
          int v = v_corner+m;

          // get depth value from depth_img(u,v)
          double depth = get_depth(u,v);

          PointT p; // declare point p
          // conculate the 3D position
          p.z = depth * 0.0002500000118743628;
          p.x = (u - K_(0,2)) * p.z / K_(0,0);
          p.y = (v - K_(1,2)) * p.z / K_(1,1);

          // add one point into point cloud pool
          cloud->points.push_back(p);
        }
      }
      // 设置并保存点云
      cloud->height = 1;
      cloud->width = cloud->points.size();
      cloud->is_dense = false;
      if(cloud->size()>0){
        objsPointCloud.push_back(*cloud);
        cloud->points.clear();
      }

      // get bbox uv based on mask 
      cv::Mat mask_img = imgMask.clone();

      std::vector<std::vector<cv::Point>> contours0;
      std::vector<cv::Vec4i> hierarchy;
      cv::findContours(mask_img,contours0,hierarchy,cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

      cv::Point2f vtx[4];    //第一个元素是左上角，第2个元素是右上角，第三个右下角，第四个元素是左下角
      cv::RotatedRect mask_box = cv::minAreaRect(contours0[0]);   //带旋转的矩形
      mask_box.points(vtx);
      vertex_reorder(vtx);
      float angle = -atan((vtx[0].y - vtx[1].y)/(vtx[1].x - vtx[0].x))/M_PI*180;

      // ********************************************************************************************************//
      // int u_base = u_corner+(vtx[0].x+vtx[1].x)/2;        int u_diff = (vtx[0].x-vtx[1].x)/8;
      // int v_base = v_corner+(vtx[0].y+vtx[3].y)/2;        int v_diff = (vtx[0].y-vtx[3].y)/8;
      // double depth_base = get_depth(u_base,v_base);
      // double z = depth_base*0.0002500000118743628;
      // double x_distance_per_pixel = z/K_(0,0);
      // double y_distance_per_pixel = z/K_(1,1);
      // int u = u_base - (0.05/y_distance_per_pixel)*sin(angle/180*M_PI);
      // int v = v_base - (0.05/y_distance_per_pixel)*cos(angle/180*M_PI);

      // int u_sample = u_corner + vtx[0].x;    //angle约等于90度
      // int v_sample = v_corner + vtx[0].y - (0.05/y_distance_per_pixel)*cos(angle/180*M_PI);
      // double z_sample_left=0,x_sample_left=0,y_sample_left=0;
      // double z_sample_right=0,x_sample_right=0,y_sample_right=0;

      // // left sample point conculate
      // u_sample = u_sample - u_diff;
      // v_sample = v_sample - v_diff;
      // double depth_sample = get_depth(u_sample,v_sample);
      // z_sample_left = depth_sample*0.0002500000118743628;
      // x_sample_left = ((u_sample - K_(0,2))*z)/K_(0,0);
      // y_sample_left = ((v_sample - K_(1,2))*z)/K_(1,1);

      // // right sample point conculate
      // u_sample = u_sample - 6*u_diff;
      // v_sample = v_sample - 6*v_diff;
      // depth_sample = get_depth(u_sample,v_sample);
      // z_sample_right = depth_sample*0.0002500000118743628;
      // x_sample_right = ((u_sample - K_(0,2))*z)/K_(0,0);
      // y_sample_right = ((v_sample - K_(1,2))*z)/K_(1,1);

      double u_base = u_corner+(vtx[0].x+vtx[1].x)/2;
      double v_base = v_corner+(vtx[0].y+vtx[3].y)/2;
      double depth_base = get_depth(u_base,v_base);
      depth_base *= 0.0002500000118743628;
      double y_distance_per_pixel = depth_base/K_(1,1);


      double u_dash = u_base - 5;
      double v_dash = v_base;

      double depth_dash = get_depth(u_dash, v_dash);
      depth_dash *= 0.0002500000118743628;

      // std::cout << "base: u: " << u_base << ", v: " << v_base << ", depth: " << depth_base << std::endl;
      // std::cout << "dash: u: " << u_dash << ", v: " << v_dash << ", depth: " << depth_dash << std::endl;
      
      pcl::PointXYZ base;
      base.x = (u_base - K_(0,2)) * depth_base / K_(0,0);
      base.y = (v_base - K_(1,2)) * depth_base / K_(1,1);
      base.z = depth_base;

      pcl::PointXYZ dash;
      dash.x = (u_dash - K_(0,2)) * depth_dash / K_(0,0);
      dash.y = (v_dash - K_(1,2)) * depth_dash / K_(1,1);
      dash.z = depth_dash;

      Eigen::Matrix<double,1,3> delta;
      delta << (dash.x - base.x), (dash.y - base.y), (dash.z - base.z);
      delta.normalized(); 
      


      std::cout << "base: x: " << base.x << ", y: " << base.y << ", z: " << base.z << std::endl;
      std::cout << "dash: x: " << dash.x << ", y: " << dash.y << ", z: " << dash.z << std::endl;
      std::cout << "delta: x: " << delta[0]*1000 << ", y: " << delta[1]*1000 << ", z: " << delta[2]*1000 << std::endl << std::endl;



      Eigen::Matrix<double,1,3> vector;  vector << (delta[0]), (delta[1]), (delta[2]);
      // Eigen::Matrix<double,1,3> vector;  vector << (x_sample_left-x_sample_right), (y_sample_left-y_sample_right), (z_sample_left-z_sample_right);
      vector = vector.normalized();

      std::vector<double> temp(11);
      temp[0] = u_corner+(vtx[0].x+vtx[1].x)/2;
      temp[1] = v_corner+(vtx[2].y+vtx[3].y)/2-(0.048/y_distance_per_pixel)*cos(angle/180*M_PI); // 0.045
      // temp[0] = u_corner+(vtx[0].x+vtx[1].x)/2;
      // temp[1] = v_corner+(vtx[0].y+vtx[3].y)/2;
      temp[2] = vector[0];    // x
      temp[3] = vector[1];    // y
      temp[4] = vector[2];    // z
      // temp[2] = -1;    // x
      // temp[3] = 0;    // y
      // temp[4] = 0;    // z
      
      // ********************************************************************************************************//
      bbox_temp.push_back(temp);
    }

    // draw bonding boxes
    cv::rectangle(color_img_, cv::Point{(int)(box.left+roi_top_left_pt_.x), (int)(box.top+roi_top_left_pt_.y)},
                              cv::Point{(int)(box.right+roi_top_left_pt_.x), (int)(box.bottom+roi_top_left_pt_.y)}, cv::Scalar{0, 255, 0},2);
  }
  cv::rectangle(color_img_, cv::Point{(int)(roi_top_left_pt_.x), (int)(roi_top_left_pt_.y)},
                        cv::Point{(int)(roi_top_left_pt_.x + roi_img_width_), (int)(roi_top_left_pt_.y + roi_img_height_)}, cv::Scalar{255, 0, 0},2);
  bbox_num_ = bbox_temp.size();
  return bbox_temp;
}

void BoxPoseDetection::vertex_reorder(cv::Point2f* vertex) {
    cv::Point2f temp_vertex[4];
    double mid_x = (vertex[0].x + vertex[1].x + vertex[2].x + vertex[3].x)/4.0;
    double mid_y = (vertex[0].y + vertex[1].y + vertex[2].y + vertex[3].y)/4.0;
    for (int i = 0; i < 4; i ++){
        auto temp = vertex[i];
        if (temp.x < mid_x && temp.y < mid_y) {
            temp_vertex[0] = temp;
        }else if (temp.x > mid_x && temp.y < mid_y) {
            temp_vertex[1] = temp;
        }else if (temp.x > mid_x && temp.y > mid_y) {
            temp_vertex[2] = temp;
        }else {
            temp_vertex[3] = temp;
        }
    }
    for (int i = 0; i < 4; i ++) {
        vertex[i] = temp_vertex[i];
    }


}

/***************************************************************
  *  @brief     更新bbox，滤除不在同一平面的箱子，仅留下最前面(离相机最近)的箱子，并且更新可用箱子数量
  *  @param     bbox
  *  @result    bbox
 **************************************************************/
std::vector<std::vector<double>> BoxPoseDetection::update_front_bbox(std::vector<std::vector<double>> bbox) {
  double z_min = 100000000;
  std::vector<std::vector<double>> bbox_temp;
  for(int index=0;index<bbox_num_;index++){
    double depth = get_depth(bbox[index][0],bbox[index][1]);
    if(z_min>depth*0.0002500000118743628) {
      z_min = depth*0.0002500000118743628;
    }
  }
  for(int index=0;index<bbox_num_;index++) {
    double depth = get_depth(bbox[index][0],bbox[index][1]);
    if((depth*0.0002500000118743628-z_min)<5){
      bbox_temp.push_back(bbox[index]);
    }
  }
  bbox_num_= bbox_temp.size();
  return bbox_temp;
}

/***************************************************************
  *  @brief     计算获取到的箱子的姿态相对于相机的旋转矩阵
  *  @param     objsPointCloud
  *  @param     bbox_temp  输入bbox
  *  @result    bbox  具有11列的二维vector数组，在原基础增加了旋转矩阵的值
 **************************************************************/
std::vector<Eigen::Matrix4d> BoxPoseDetection::calculateGraspPose(std::vector<pcl::PointCloud<pcl::PointXYZ>> &objsPointCloud, std::vector<std::vector<double>> &bbox_temp)//s:检测到的箱子个数；k:平面拟合方法选择参数;path_pcd：读取pcd文件路径
{
  std::vector<Eigen::Matrix4d> RT(bbox_num_);
  for(int idx=0; idx<bbox_num_; ++idx)
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr clouds(new pcl::PointCloud<pcl::PointXYZ>);
    clouds->clear();
    *clouds = objsPointCloud[idx];
    clouds->height = 1;
    pcl::PointCloud<pcl::PointXYZ>::Ptr flatPoints(new pcl::PointCloud<pcl::PointXYZ>);
    // get the box plane: RANSAC
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr model_plane(new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(clouds));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac(model_plane);	
    ransac.setDistanceThreshold(0.03);	//设置距离阈值，与平面距离小于0.3的点作为内点
    ransac.computeModel();				      //执行模型估计
    std::vector<int> inliers;				    //存储内点索引的容器
    ransac.getInliers(inliers);			    //提取内点索引
    pcl::copyPointCloud<pcl::PointXYZ>(*clouds, inliers, *flatPoints);
    // 输出模型参数
    Eigen::VectorXf coefficient;
    ransac.getModelCoefficients(coefficient);

    // get the plane key data (format:vector)
    double x_vec=0, y_vec=0, z_vec=0;
    if(coefficient[2]<0) {
      x_vec = -coefficient[0];
      y_vec = -coefficient[1];
      z_vec = -coefficient[2];
    } else {
      x_vec = coefficient[0];
      y_vec = coefficient[1];
      z_vec = coefficient[2];
    }
    
    // get boxes pose rotation matrix
    Eigen::Vector3d x_vec_pose;  x_vec_pose<<x_vec,y_vec,z_vec;
    x_vec_pose = x_vec_pose.normalized();
    std::cout << "x_vec_pose: " << x_vec_pose << std::endl;
    Eigen::Vector3d y_vec_pre;   y_vec_pre<<bbox_temp[idx][2],bbox_temp[idx][3],bbox_temp[idx][4];
    double para = (x_vec_pose[0]*y_vec_pre[0]+x_vec_pose[1]*y_vec_pre[1]+x_vec_pose[2]*y_vec_pre[2])/(x_vec_pose[0]*x_vec_pose[0]+x_vec_pose[1]*x_vec_pose[1]+x_vec_pose[2]*x_vec_pose[2]);
    Eigen::Vector3d y_vec_pose;  y_vec_pose = y_vec_pre - para*x_vec_pose;
    y_vec_pose = y_vec_pose.normalized();
    std::cout << "y_vec_pose: " << y_vec_pose << std::endl;

    Eigen::Vector3d z_vec_pose;  z_vec_pose = x_vec_pose.cross(y_vec_pose);
    // Eigen::Vector3d z_vec_pose;  z_vec_pose << 0,0,1;
    
    std::cout << "z_vec_pose: " << z_vec_pose << std::endl;


    Eigen::Matrix4d RT_matrix;
    RT_matrix << x_vec_pose[0], y_vec_pose[0], z_vec_pose[0], 0,
                 x_vec_pose[1], y_vec_pose[1], z_vec_pose[1], 0,
                 x_vec_pose[2], y_vec_pose[2], z_vec_pose[2], 0,
                 0,             0,             0,             1;

    std::cout << " [RT_Matrix]:" << std::endl;
    std::cout << RT_matrix << std::endl;

    // RT_matrix << 1, 0, 0, 0,
    //              0, 1, 0, 0,
    //              0, 0, 1, 0,
    //              0,             0,             0,             1;
    // RT_matrix << -y_vec_pose[0], -z_vec_pose[0], x_vec_pose[0], 0,
    //              -y_vec_pose[1], -z_vec_pose[1], x_vec_pose[1], 0,
    //              -y_vec_pose[2], -z_vec_pose[2], x_vec_pose[2], 0,
    //              0,             0,             0,             1;



    RT[idx] = RT_matrix;
    clouds->clear();
    flatPoints->clear();
  }
  return RT;
}

/***************************************************************
  *  @brief     将 calculateGraspPose 函数中获取到的旋转矩阵的9个数赋值给 bbox 的[2~10]
  *  @param     bbox  std::vector<std::vector<double>>
  *  @param     RT_matrix   calculateGraspPose 返回的旋转矩阵
  *  @result    bbox  具有11列的二维vector数组，在原基础增加了旋转矩阵的值
 **************************************************************/
std::vector<std::vector<double>> BoxPoseDetection::update_rotation_matrix(std::vector<std::vector<double>> bbox, std::vector<Eigen::Matrix4d> RT_matrix) {
  for(int index=0;index<bbox_num_;index++) {
    Eigen::Matrix4d temp = RT_matrix[index];
    bbox[index][2] = temp(0,0);   bbox[index][3] = temp(0,1);   bbox[index][4] = temp(0,2);
    bbox[index][5] = temp(1,0);   bbox[index][6] = temp(1,1);   bbox[index][7] = temp(1,2);
    bbox[index][8] = temp(2,0);   bbox[index][9] = temp(2,1);   bbox[index][10] = temp(2,2);
  }

  ////////////////// bbox[index][0] = u, bbox[index][1] = v 
  return bbox;
}

/***************************************************************
  *  @brief     根据指定行范围内的bbox的uv坐标进行初步的轨迹规划，未进行语义分析
  *  @param     col_begin    排列开始位置
  *  @param     col_end      排列结束位置
  *  @param     bbox_after
  *  @param     sign         标志位，决定了范围内(可视化中每行的box)bbox是按从大到小还是从小到大排列
  *  @result    bbox_after
 **************************************************************/
std::vector<std::vector<double>> BoxPoseDetection::sort_in_range(int col_begin, int col_end, std::vector<std::vector<double>> bbox_after, bool sign) {
  
  int col_num = col_end - col_begin;
  std::vector<std::vector<double>> temp(col_num,std::vector<double>(11));

  for(int i=0;i<col_num;i++) {
    temp[i] = bbox_after[col_begin+i];
  }

  if(!sign) {
    std::sort(temp.begin(), temp.end());
  }
  else {
    std::sort(temp.begin(), temp.end(), cmp);
  }
  
  for(int i=0;i<col_num-1;i++) {
    bbox_after[col_begin+i] = temp[i+1];

  }
  bbox_after[col_begin + col_num-1] = temp[0];

  return bbox_after;
}

/***************************************************************
  *  @brief     根据bbox的u,v坐标以及旋转矩阵获取每个box在相机坐标系下的3D位姿
  *  @param     bbox_temp
  *  @result    threeD_pose    [x,y,z,pitch,roll,yaw]
 **************************************************************/
std::vector<std::vector<double>> BoxPoseDetection::get_3D_pose(std::vector<std::vector<double>> bbox_temp) {
  std::vector<std::vector<double>> threeD_pose(bbox_num_,std::vector<double>(6));
  for(int index=0;index<bbox_num_;index++) {
    Eigen::Matrix3d RT;  RT << bbox_temp[index][2], bbox_temp[index][3], bbox_temp[index][4],
                               bbox_temp[index][5], bbox_temp[index][6], bbox_temp[index][7],
                               bbox_temp[index][8], bbox_temp[index][9], bbox_temp[index][10];
    Eigen::Vector3d euler_angles = RT.eulerAngles(2,1,0);   // rz ry rx
    double depth = get_depth(bbox_temp[index][0],bbox_temp[index][1]);
    double z = depth*0.0002500000118743628;
    double x = ((bbox_temp[index][0] - K_(0,2))*z)/K_(0,0);
    double y = ((bbox_temp[index][1] - K_(1,2))*z)/K_(1,1);

    threeD_pose[index][0] = x;
    threeD_pose[index][1] = y;
    threeD_pose[index][2] = z;
    threeD_pose[index][3] = euler_angles(1);    // ry
    threeD_pose[index][4] = euler_angles(2);    // rx
    threeD_pose[index][5] = euler_angles(0);    // rz
  }
  return threeD_pose;
}

/***************************************************************
  *  @brief     对bbox进行轨迹规划并进行语义分析
  *  @param     bbox
  *  @param     up_angle        小臂向上抬起为正
  *  @param     down_angle      小臂向上抬起为正
  *  @param     indentation     左右两边抓取点向中心移动的距离
  *  @param     inde_angle      左右两边抓取点转动一定角度便于抓取，以tcp坐标系下绕z轴逆时针转动为正(delta_Rz)
 **************************************************************/
std::pair<std::vector<std::vector<double>>, std::vector<std::vector<double>>> BoxPoseDetection::box_tra_process(std::vector<std::vector<double>> bbox, double up_angle, double down_angle, double indentation, double inde_angle) { 

  std::vector<std::vector<double>> bbox_before = bbox;
  std::vector<std::vector<double>> bbox_after(bbox_num_,std::vector<double>(11));
  std::vector<std::vector<double>> threeD_pose(bbox_num_,std::vector<double>(6));

  int count_high=0, count_low=0;
  bool sign = false;
  while (count_high<bbox_num_) {
    std::vector<double> col_bbox;
    for (int i = 0; i < bbox_num_; ++i) {
      col_bbox.push_back(bbox_before[i][1]);
    }
    double v_min = *min_element(col_bbox.begin(), col_bbox.end());  //get the min_v of the bboxes
    if(v_min>900000.0) break;
    //下面的阈值70是假设在同一行上的箱子的抓取点像素值小于70pixel
    for(int index=0;index<bbox_num_;index++) {
      if(abs(bbox_before[index][1]-v_min)<50) {               
        bbox_after[count_high] = bbox_before[index];
        bbox_before[index][0] = 1000000.0;
        bbox_before[index][1] = 1000000.0;
        count_high++;
      }
    }
    
    bbox_after = sort_in_range(count_low, count_high, bbox_after, sign);
    count_low = count_high;

//    sign = change_sign(sign);
  }

  // get the u_min u_max v_min v_max in all bbox uv point
  std::vector<double> col_bbox,ln_bbox;
  for (int i = 0; i < bbox_num_; ++i) {
    col_bbox.push_back(bbox_after[i][1]);   // v
    ln_bbox.push_back(bbox_after[i][0]);    // u
  }
  double v_min = 0.0;
  double v_max = 0.0;
  double u_min = 0.0;
  double u_max = 0.0;

  if (bbox_num_ > 0)
  {
    v_min = *min_element(col_bbox.begin(), col_bbox.end());
    v_max = *max_element(col_bbox.begin(), col_bbox.end());
    u_min = *min_element(ln_bbox.begin(), ln_bbox.end());
    u_max = *max_element(ln_bbox.begin(), ln_bbox.end());
  }

  // 基于坐标的语义分割
  for(int i=0;i<bbox_num_;i++) {
    // top layer
    if(bbox_after[i][1]<v_min+30 && bbox_after[i][1]<250) {
      Eigen::Matrix3d RT;  RT << bbox_after[i][2], bbox_after[i][3], bbox_after[i][4],
                                 bbox_after[i][5], bbox_after[i][6], bbox_after[i][7],
                                 bbox_after[i][8], bbox_after[i][9], bbox_after[i][10];
      Eigen::Vector3d euler_angles = RT.eulerAngles(2,1,0);   // rz ry rx
      euler_angles(0) += -up_angle/180*M_PI;
      RT = eulerAnglesToRotationMatrix(euler_angles);
      bbox_after[i][2] = RT(0,0);    bbox_after[i][3] = RT(0,1);    bbox_after[i][4]  = RT(0,2);
      bbox_after[i][5] = RT(1,0);    bbox_after[i][6] = RT(1,1);    bbox_after[i][7]  = RT(1,2);
      bbox_after[i][8] = RT(2,0);    bbox_after[i][9] = RT(2,1);    bbox_after[i][10] = RT(2,2);
    }
    // bottom layer
    if(bbox_after[i][1]>v_max-30) {
      Eigen::Matrix3d RT;  RT << bbox_after[i][2], bbox_after[i][3], bbox_after[i][4],
                                 bbox_after[i][5], bbox_after[i][6], bbox_after[i][7],
                                 bbox_after[i][8], bbox_after[i][9], bbox_after[i][10];
      Eigen::Vector3d euler_angles = RT.eulerAngles(2,1,0);   // rz ry rx
      euler_angles(0) += -down_angle/180*M_PI;
      RT = eulerAnglesToRotationMatrix(euler_angles);
      bbox_after[i][2] = RT(0,0);    bbox_after[i][3] = RT(0,1);    bbox_after[i][4]  = RT(0,2);
      bbox_after[i][5] = RT(1,0);    bbox_after[i][6] = RT(1,1);    bbox_after[i][7]  = RT(1,2);
      bbox_after[i][8] = RT(2,0);    bbox_after[i][9] = RT(2,1);    bbox_after[i][10] = RT(2,2);
    }
    // left col   70所对应的单位是像素pixel
    if(bbox_after[i][0]<u_min+70) {
      double depth_base = get_depth(bbox_after[i][0],bbox_after[i][1]);
      double z = depth_base*0.0002500000118743628;
      double x_distance_per_pixel = z/K_(0,0);
      double y_distance_per_pixel = z/K_(1,1);
      bbox_after[i][0] += indentation/x_distance_per_pixel;

      Eigen::Matrix3d RT;  RT << bbox_after[i][2], bbox_after[i][3], bbox_after[i][4],
                                 bbox_after[i][5], bbox_after[i][6], bbox_after[i][7],
                                 bbox_after[i][8], bbox_after[i][9], bbox_after[i][10];
      Eigen::Vector3d euler_angles = RT.eulerAngles(2,1,0);   // rz ry rx
      euler_angles(1) += -inde_angle/180*M_PI;
      RT = eulerAnglesToRotationMatrix(euler_angles);
      bbox_after[i][2] = RT(0,0);    bbox_after[i][3] = RT(0,1);    bbox_after[i][4]  = RT(0,2);
      bbox_after[i][5] = RT(1,0);    bbox_after[i][6] = RT(1,1);    bbox_after[i][7]  = RT(1,2);
      bbox_after[i][8] = RT(2,0);    bbox_after[i][9] = RT(2,1);    bbox_after[i][10] = RT(2,2);
    }
    //right col
    if(bbox_after[i][0]>u_max-70) {
      double depth_base = get_depth(bbox_after[i][0],bbox_after[i][1]);
      double z = depth_base*0.0002500000118743628;
      double x_distance_per_pixel = z/K_(0,0);
      double y_distance_per_pixel = z/K_(1,1);
      bbox_after[i][0] -= indentation/x_distance_per_pixel;

      Eigen::Matrix3d RT;  RT << bbox_after[i][2], bbox_after[i][3], bbox_after[i][4],
                                 bbox_after[i][5], bbox_after[i][6], bbox_after[i][7],
                                 bbox_after[i][8], bbox_after[i][9], bbox_after[i][10];
      Eigen::Vector3d euler_angles = RT.eulerAngles(2,1,0);   // rz ry rx
      euler_angles(1) -= inde_angle/180*M_PI;
      RT = eulerAnglesToRotationMatrix(euler_angles);
      bbox_after[i][2] = RT(0,0);    bbox_after[i][3] = RT(0,1);    bbox_after[i][4]  = RT(0,2);
      bbox_after[i][5] = RT(1,0);    bbox_after[i][6] = RT(1,1);    bbox_after[i][7]  = RT(1,2);
      bbox_after[i][8] = RT(2,0);    bbox_after[i][9] = RT(2,1);    bbox_after[i][10] = RT(2,2);
    }
  }

  threeD_pose = get_3D_pose(bbox_after);

  return std::make_pair(bbox_after,threeD_pose);
}

  //预处理
void BoxPoseDetection::getRoiImage(cv::Mat cur_frame_img){
  roi_rgb_image_ = cur_frame_img(cv::Rect(roi_top_left_pt_.x, roi_top_left_pt_.y, roi_img_width_, roi_img_height_)).clone();
  // color_img_ = color_img_(cv::Rect(roi_top_left_pt_.x, roi_top_left_pt_.y, roi_img_width_, roi_img_height_));
  // depth_img_ = depth_img_(cv::Rect(roi_top_left_pt_.x, roi_top_left_pt_.y, roi_img_width_, roi_img_height_));
}

/***************************************************************
  *  @brief     运行的主程序
  *  @param     img   传入一张空白图片用于与类内的 color_img_ 进行区分
  *  @result    threeD_pose_back  
 **************************************************************/
std::queue<std::vector<double>> BoxPoseDetection::process(cv::Mat &img) {   // cv::Mat &img

  mmdeploy_detector_t detector = nullptr;
  int status{};
  status = mmdeploy_detector_create_by_path(str_model_path_.c_str(), str_device_name_.c_str(), 0, &detector);
  if (status != MMDEPLOY_SUCCESS) fprintf(stderr, "failed to create detector, code: %d\n", (int)status);


  //**********************************************************************************************************************//
  cv::Mat color_img_deep_copy = color_img_.clone();
  // cv::imshow("debug", color_img_deep_copy);
  // cv::waitKey();
  getRoiImage(color_img_deep_copy);

  
  // cv::imshow("debug", roi_rgb_image_);
  // cv::waitKey();
  mmdeploy_mat_t mat{roi_rgb_image_.data, roi_rgb_image_.rows, roi_rgb_image_.cols, 3, MMDEPLOY_PIXEL_FORMAT_BGR, MMDEPLOY_DATA_TYPE_UINT8};

  mmdeploy_detection_t* bboxes = nullptr;
  int* res_count = nullptr;

  status = mmdeploy_detector_apply(detector, &mat, 1, &bboxes, &res_count);
  if (status != MMDEPLOY_SUCCESS) fprintf(stderr, "failed to apply detector, code: %d\n", (int)status);

  fprintf(stdout, "bbox_count=%d\n", *res_count);

  std::vector<pcl::PointCloud<pcl::PointXYZ>> objsPointCloud;
  std::vector<std::vector<double>> bbox_pre = vis_pc_bboxuv_process(res_count, bboxes, objsPointCloud);
  std::vector<std::vector<double>> bbox = update_front_bbox(bbox_pre);

  cv::imwrite("output_detection.png", color_img_);
  std::cout<<"avaible box_num: "<<bbox_num_<<"\n";

  std::vector<Eigen::Matrix4d> RT_matrixs = calculateGraspPose(objsPointCloud,bbox);
  bbox = update_rotation_matrix(bbox, RT_matrixs);

  std::vector<std::vector<double>> bbox_after(bbox_num_,std::vector<double>(11));
  std::vector<std::vector<double>> threeD_pose(bbox_num_,std::vector<double>(6));

  // std::pair <std::vector<std::vector<double>>, std::vector<std::vector<double>>> pair_var = box_tra_process(bbox, 3, 0, 0.03, 3);
  std::pair <std::vector<std::vector<double>>, std::vector<std::vector<double>>> pair_var = box_tra_process(bbox, 0, 0, 0, 0);
  bbox_after = pair_var.first;
  threeD_pose = pair_var.second;
  
  // ********************************************************************************************************//
  // draw output result img
  for(int index=0;index<bbox_num_;index++) {    
    // draw circle and arrow
    int u = int(bbox_after[index][0]);  int v = int(bbox_after[index][1]); //set draw position
    int font = cv::FONT_HERSHEY_SIMPLEX;  //set txt word type
    cv::circle(color_img_, cv::Point(u,v), 20, (255,0,0), -1);
    cv::putText(color_img_,std::to_string(index+1),cv::Point(u+20,v+10), font, 1,(0,255,255), 2, cv::LINE_AA);
    if(index>0 && index<bbox_num_) {
      int u_bef = int(bbox_after[index-1][0]);  int v_bef = int(bbox_after[index-1][1]); //set draw position
//      cv::arrowedLine(color_img_,cv::Point(u_bef,v_bef),cv::Point(u,v), (0,0,255), 3, 2, 0, 0.02);
    }

    // draw cordinate
    cv::Mat rmatrix_m = (cv::Mat_<double>(3,3) << bbox_after[index][2],bbox_after[index][3],bbox_after[index][4],
                                                  bbox_after[index][5],bbox_after[index][6],bbox_after[index][7],
                                                  bbox_after[index][8],bbox_after[index][9],bbox_after[index][10]);

        // cv::Mat rmatrix_m = (cv::Mat_<double>(3,3) << 1,0,0,
        //                                               0,1,0,
        //                                               0,0,1);
    cv::Mat rvec = (cv::Mat_<double>(3,1));
    cv::Mat tvec = (cv::Mat_<double>(1,3) << threeD_pose[index][0],threeD_pose[index][1],threeD_pose[index][2]);
    std::cout<<tvec<<"\n";
    cv::Rodrigues(rmatrix_m,rvec);
    cv::drawFrameAxes(color_img_, K_m_, distCoeffs_m_, rvec, tvec, 0.1);
  }

  // save output result img
  color_img_.copyTo(img);
  cv::imwrite("out.png",color_img_);
//  cv::imshow("result", color_img_);
//  while (cv::waitKey(1) != 'q') {
//    continue;
//  }
  
  // ********************************************************************************************************//

  std::queue<std::vector<double>> threeD_pose_back;
  for(int index=0;index<bbox_num_;index++) {
    threeD_pose_back.push(threeD_pose[index]);
  }
  mmdeploy_detector_release_result(bboxes, res_count, 1);
  mmdeploy_detector_destroy(detector);

  return threeD_pose_back;
}

// process example 
//int main(int argc, char* argv[])
//{
// //-------------------------------------config--------------------------------------------------------------
// char const *color_img_path = "./rgb.png";
// char const *depth_img_path = "./depth.png";

// cv::Mat color_img = cv::imread(color_img_path, cv::IMREAD_COLOR);
// cv::Mat depth_img = cv::imread(depth_img_path, cv::IMREAD_ANYDEPTH);



// double fx = 1349.66, fy = 1349.17, cx = 982.689, cy = 545.792;
// Eigen::MatrixXd K(3,3);
// Eigen::VectorXd distCoeffs(5);
// K << fx,0,cx,0,fy,cy,0,0,1;
// distCoeffs << 0,0,0,0,0;
// //-------------------------------------config end-----------------------------------------------------------

// BoxPoseDetection box_arrange("../mmdeploy_model/mask_rcnn", color_img, depth_img, K, distCoeffs);

// std::queue<std::vector<double>> threeD_pose = box_arrange.process(color_img);   //vector内为  {x, y, z, pitch, roll. yaw}
  
// return 0;
//}
