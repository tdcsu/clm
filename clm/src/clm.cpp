#include <point_type_self.h>
#include <Eigen/Core>
#include <fstream>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <pcl/common/io.h>
#include <pcl/io/pcd_io.h>
#include <sstream>
#include <stdio.h>
#include <string>
#include <dynamic_reconfigure/server.h>

#include <pcl/visualization/cloud_viewer.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <clm/ExtrinsicConfig.h>
using namespace std;
vector<double> param_camera_matrix;
vector<double> param_dist_coeffs;
ros::NodeHandle *nh_;

int cloud_frame_ = 0, acc_num_ = 40, pre_acc_num_=40;
std::string cloud_topic_in;
std::string cloud_topic_out;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr acc_pc(new pcl::PointCloud<pcl::PointXYZRGB>);
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_in(new pcl::PointCloud<pcl::PointXYZ>);
vector<size_t> pc_size_v;

double roll,pitch,yaw,t_x,t_y,t_z; 

void param_callback(clm::ExtrinsicConfig &config, uint32_t level);
class CameraLidarCal
{
  public:
    
    ros::NodeHandle nh_;
    
    image_transport::ImageTransport it_;
    
    image_transport::Subscriber image_sub_;
    
    image_transport::Publisher image_pub_;

    ros::Subscriber cloud_sub_=nh_.subscribe("livox/points",1,&CameraLidarCal::cloudCb,this);
    
    ros::Publisher color_cloud_pub_=nh_.advertise<sensor_msgs::PointCloud2>("livox/colord_cloud",1);
    const int save_flag=1;
    int color_intensity_threshold_ = 5;
    const int density = 1;
    typedef Eigen::Matrix<double, 6, 1> Vector6d;

    const cv::Mat input_image;
  //所需变量
    
    
    CameraLidarCal()
      : it_(nh_)
      {
        ROS_INFO("init");
        image_sub_=it_.subscribe("/left_camera/image",1, &CameraLidarCal::imageCb, this);
        ROS_INFO("init success");
      }

      ~CameraLidarCal()
      {
        
      }

      void imageCb(const sensor_msgs::ImageConstPtr& msg)
      {
      cv_bridge::CvImagePtr cv_ptr;
      try
        {
          ROS_INFO("trying to convert data");
          cv_ptr=cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
          ROS_INFO("convert data success");

        }
        catch (cv_bridge::Exception& e)
        {
          ROS_INFO("unable to convert data");
          ROS_ERROR("cv_bridge exception: %s", e.what());
          return;    
        }
        input_image=cv_ptr->image;
        ROS_INFO("Image converted!!");
      }

      void cloudCb(const sensor_msgs::PointCloud2ConstPtr &cloud_msg) 
      {  
        ROS_INFO("real time maunual calibration!!");
        std::cout<<"roll:"<<roll<<", pitch:"<<pitch<<", yaw:"<<yaw<<std::endl;
        std::cout<<"x:"<<t_x<<", y:"<<t_y<<", z:"<<t_z<<std::endl;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr raw_lidar_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PCLPointCloud2 cloud_msg_temp;
        ROS_INFO("1!!");
        const cv::Mat rgb_img;
            // 相机内参
        int width_, height_;
        cv::Mat camera_matrix_ = (cv::Mat_<double>(3,3)<< param_camera_matrix[0],               0.,    param_camera_matrix[2],
                                                          0.,                   param_camera_matrix[4], param_camera_matrix[5],
                                                          0.,                   0.,   1.      );
        cv::Mat dist_coeffs_ = (cv::Mat_<double>(1,4)<< param_dist_coeffs[0],param_dist_coeffs[1], param_dist_coeffs[2], param_dist_coeffs[3], param_dist_coeffs[4]);
        cv::Mat init_extrinsic_;

        float fx_ = camera_matrix_.at<double>(0, 0);
        float cx_ = camera_matrix_.at<double>(0, 2);
        float fy_ = camera_matrix_.at<double>(1, 1);
        float cy_ = camera_matrix_.at<double>(1, 2);
        float k1_ = dist_coeffs_.at<double>(0, 0);
        float k2_ = dist_coeffs_.at<double>(0, 1);
        float p1_ = dist_coeffs_.at<double>(0, 2);
        float p2_ = dist_coeffs_.at<double>(0, 3);
        float k3_ = dist_coeffs_.at<double>(0, 4);

        ROS_INFO("2!!");
        pcl_conversions::toPCL(*cloud_msg, cloud_msg_temp);
        pcl::fromPCLPointCloud2(cloud_msg_temp, *raw_lidar_cloud);
        //if (input_image.type() == CV_8UC3) {
        rgb_img=input_image;
        //} 
        /*else if (input_image.type() == CV_8UC1) {
          ROS_INFO("data type need to be converted!!");
          cv::cvtColor(input_image, rgb_img, cv::COLOR_GRAY2BGR);
        } //确保输入类型正确
        */
        std::cout<<"当前帧点云的数量"<<raw_lidar_cloud->size()<<std::endl;
        std::vector<cv::Point3f> pts_3d;
        for (size_t i = 0; i < raw_lidar_cloud->size(); i += density) {//定义的密度
          pcl::PointXYZ point = raw_lidar_cloud->points[i];
          float depth = sqrt(pow(point.x, 2) + pow(point.y, 2) + pow(point.z, 2));//计算深度
          if (depth > 0.5 && depth < 50) { //如果深度在2到50之间并且强度有效
            pts_3d.emplace_back(cv::Point3f(point.x, point.y, point.z));//将当前点放入容器中
          }
        }
        Eigen::Matrix4d initial_extrinsic;
        initial_extrinsic<<0, -1, 0, 0,
                          0, 0, -1, 0,
                          1, 0, 0, 0,
                          0, 0, 0, 1;
        Eigen::AngleAxisd rotation_vector3;//定义3*1旋转向量
        // euler angle to rotation matrix
        Eigen::Vector3d eulerAngle_input(yaw,pitch,roll);
        Eigen::AngleAxisd rollAngle(Eigen::AngleAxisd(eulerAngle_input(2),Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd pitchAngle(Eigen::AngleAxisd(eulerAngle_input(1),Eigen::Vector3d::UnitY()));
        Eigen::AngleAxisd yawAngle(Eigen::AngleAxisd(eulerAngle_input(0),Eigen::Vector3d::UnitZ()));
        
        Eigen::Matrix3d rotation_matrix;
        rotation_matrix=yawAngle*pitchAngle*rollAngle;
        Eigen::Vector3d translation_vector(t_x,t_y,t_z);

        Eigen::Matrix4d right_extrinsic;
        right_extrinsic<<rotation_matrix(0,0),rotation_matrix(0,1),rotation_matrix(0,2),translation_vector[0],
                         rotation_matrix(1,0),rotation_matrix(1,1),rotation_matrix(1,2),translation_vector[1],
                         rotation_matrix(2,0),rotation_matrix(2,1),rotation_matrix(2,2),translation_vector[2],
                         0                   ,                   0,                   0,                     1;
        std::cout<<"初始转换矩阵为："<<rotation_matrix(0,0)<<","<<rotation_matrix(0,1)<<","<<rotation_matrix(0,2)<<","<<translation_vector[0]<<std::endl
                                  <<rotation_matrix(1,0)<<","<<rotation_matrix(1,1)<<","<<rotation_matrix(1,2)<<","<<translation_vector[1]<<std::endl
                                  <<rotation_matrix(2,0)<<","<<rotation_matrix(2,1)<<","<<rotation_matrix(2,2)<<","<<translation_vector[2]<<std::endl
                                  <<"0,0,0,1"<<std::endl;
        Eigen::Matrix4d extrinsic_matrix_inverse=right_extrinsic.inverse();

        Vector6d extrinsic_params;
      
        ROS_INFO("3!!");
        Eigen::Matrix3d extrinsic_rotation_matrix;
        extrinsic_rotation_matrix<<extrinsic_matrix_inverse(0,0),extrinsic_matrix_inverse(0,1),extrinsic_matrix_inverse(0,2),
                                  extrinsic_matrix_inverse(1,0),extrinsic_matrix_inverse(1,1),extrinsic_matrix_inverse(1,2),
                                  extrinsic_matrix_inverse(2,0),extrinsic_matrix_inverse(2,1),extrinsic_matrix_inverse(2,2);
        Eigen::Vector3d eulerAngle= extrinsic_rotation_matrix.matrix().eulerAngles(2,1,0);
        Eigen::Vector3d translation;
        translation[0]=extrinsic_matrix_inverse(0,3);
        translation[1]=extrinsic_matrix_inverse(1,3);
        translation[2]=extrinsic_matrix_inverse(2,3);

        extrinsic_params[0]=eulerAngle[0];
        extrinsic_params[1]=eulerAngle[1];
        extrinsic_params[2]=eulerAngle[2];
        extrinsic_params[3]=translation[0];
        extrinsic_params[4]=translation[1];
        extrinsic_params[5]=translation[2];
          
          
        ROS_INFO("4!!");
        rotation_vector3 =
            Eigen::AngleAxisd(extrinsic_params[0], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(extrinsic_params[1], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(extrinsic_params[2], Eigen::Vector3d::UnitX());
        cv::Mat camera_matrix =
            (cv::Mat_<double>(3, 3) << fx_, 0.0, cx_, 0.0, fy_, cy_, 0.0, 0.0, 1.0);
        cv::Mat distortion_coeff =
            (cv::Mat_<double>(1, 5) << k1_, k2_, p1_, p2_, k3_);
        cv::Mat r_vec =
            (cv::Mat_<double>(3, 1)
           << rotation_vector3.angle() * rotation_vector3.axis().transpose()[0],
              rotation_vector3.angle() * rotation_vector3.axis().transpose()[1],
              rotation_vector3.angle() * rotation_vector3.axis().transpose()[2]);
    
        cv::Mat t_vec =  (cv::Mat_<double>(3, 1) << extrinsic_params[3],
                                                    extrinsic_params[4], 
                                                    extrinsic_params[5]);

        std::vector<cv::Point2f> pts_2d;
        ROS_INFO("5!!");
        std::cout << "欧拉角["<<eulerAngle[0]<<","<<eulerAngle[1]<<","<<eulerAngle[2]<<"]"<< std::endl;
        std::cout << "平移向量["<<translation(0)<<","<<translation(1)<<","<<translation(2)<<"]"<< std::endl;
        // std::cout << '激光雷达点'<<cv::format(pts_3d, cv::Formatter::FMT_DEFAULT) << std::endl;
        // std::cout << 'camera_matrix'<<cv::format(camera_matrix, cv::Formatter::FMT_DEFAULT) << std::endl;
        // std::cout << 'distortion_coeff'<<cv::format(distortion_coeff, cv::Formatter::FMT_DEFAULT) << std::endl;
        cv::projectPoints(pts_3d, r_vec, t_vec, camera_matrix, distortion_coeff,
                          pts_2d);//这里的r_vec 是相机相对于激光雷达的旋转矩阵,t_vec是相机坐标圆点相对于激光雷达坐标系圆点的位移,所以并没有问题
        ROS_INFO("6!!");
        int image_rows = rgb_img.rows;
        int image_cols = rgb_img.cols;
        for (size_t i = 0; i < pts_2d.size(); i++) {
          if (pts_2d[i].x >= 0 && pts_2d[i].x < image_cols && pts_2d[i].y >= 0 &&
              pts_2d[i].y < image_rows) {
            cv::Scalar color =
                rgb_img.at<cv::Vec3b>((int)pts_2d[i].y, (int)pts_2d[i].x);
            if (color[0] == 0 && color[1] == 0 && color[2] == 0) {
              continue;
            }
            if (pts_3d[i].x > 100) {
              continue;
            }
            pcl::PointXYZRGB p;
            p.x = pts_3d[i].x;
            p.y = pts_3d[i].y;
            p.z = pts_3d[i].z;
            // p.a = 255;
            p.b = color[0];
            p.g = color[1];
            p.r = color[2];
            color_cloud->points.push_back(p);
          }
        }
        color_cloud->width = color_cloud->points.size();
        std::cout<<"彩色点云数量："<<color_cloud->points.size()<<std::endl;
        color_cloud->height = 1;
        sensor_msgs::PointCloud2 color_cloud_msg;
        //insert

      

        if(acc_num_ != pre_acc_num_ && pc_size_v.size() > acc_num_)
        {   
            int num_gap = pc_size_v.size() - acc_num_;
            size_t total_size_erase = 0;
            for(int i = 0; i < num_gap; i++)
            {
                total_size_erase += pc_size_v[i];
            }
            acc_pc->points.erase(acc_pc->points.begin(), acc_pc->points.begin()+total_size_erase);
            pc_size_v.erase(pc_size_v.begin(), pc_size_v.begin()+num_gap);
        }

        if(pc_size_v.size() >= acc_num_)
        {
            size_t front_pc_size = pc_size_v.front();
            pc_size_v.erase(pc_size_v.begin());
            acc_pc->points.erase(acc_pc->points.begin(), acc_pc->points.begin()+front_pc_size);
        }

        *acc_pc += *color_cloud;
        pc_size_v.push_back(color_cloud->points.size());
      
        if(pc_size_v.size() == acc_num_)
        {


            sensor_msgs::PointCloud2 acc_pc_ros;
            pcl::toROSMsg(*acc_pc, acc_pc_ros);
            acc_pc_ros.header.frame_id = cloud_msg->header.frame_id;
            acc_pc_ros.header.stamp=ros::Time::now();
            ROS_INFO("Color cloud success, publish !!");
            color_cloud_pub_.publish(acc_pc_ros);    
        }
      }
};

void param_callback(clm::ExtrinsicConfig &config, uint32_t level)
{
  roll = config.roll;
  pitch = config.pitch;
  yaw = config.yaw;
  t_x = config.x;
  t_y = config.y;
  t_z = config.z;
}


int main(int argc, char** argv)
{
  ROS_INFO("started!!");
  ros::init(argc,argv,"clm");
  nh_ = new ros::NodeHandle("~");
  nh_->param<vector<double>>("camera/camera_matrix", param_camera_matrix,
                           vector<double>());
  nh_->param<vector<double>>("camera/dist_coeffs", param_dist_coeffs, vector<double>());
  dynamic_reconfigure::Server<clm::ExtrinsicConfig> server;
  dynamic_reconfigure::Server<clm::ExtrinsicConfig>::CallbackType f;
  f = boost::bind(param_callback, _1, _2);
  server.setCallback(f);

  CameraLidarCal clc;
  ros::spin();
}

