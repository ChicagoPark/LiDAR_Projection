# include <iostream>
# include <string>
#include <math.h>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

//#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
using namespace std;

/*
projection material
https://github.com/azureology/kitti-velo2cam/blob/master/proj_velo2cam.py
*/

cv::Mat img(720,1280,CV_8UC3, cv::Scalar(255,255,255));


int main(int argc, char** argv)
{
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  double x_cloud; double y_cloud; double z_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr PCD_cloud_o(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr PCD_cloud_p(new pcl::PointCloud<pcl::PointXYZI>);
  
  ros::Publisher pub1 = nh.advertise<PointCloud> ("points_plane", 1);
  ros::Publisher pub2 = nh.advertise<PointCloud> ("points_obstacle", 1);
  ros::Publisher pub3 = nh.advertise<sensor_msgs::Image> ("projected_obstacle", 1);
  ros::Publisher pub4 = nh.advertise<sensor_msgs::Image> ("projected_plane", 1);

  ros::Publisher marker_pub_left = nh.advertise<visualization_msgs::Marker>("visualization_marker_left", 10);
  ros::Publisher marker_pub_right = nh.advertise<visualization_msgs::Marker>("visualization_marker_right", 10);

  //ros::Publisher pub1 = nh.advertise<sensor_msgs::ImagePtr> ("image", 1);
  //pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/kaai/chicago_ws/src/first_pkg/src/Kitti_File/pcd/1039.pcd", *cloud);
  //pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/kaai/chicago_ws/src/first_pkg/src/KITTI/pcd/0000000215.pcd", *cloud);
  pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/kaai/chicago_ws/src/first_pkg/src/KITTI/pcd/0000000200.pcd", *cloud);
  Mat color_img = imread("/home/kaai/chicago_ws/src/first_pkg/src/KITTI/image/0000000200.png",IMREAD_COLOR);

  //000021.png  2090899988.pcd

  //????????? : 000030.png  2101599309.pcd
  //RANSAC Section
  pcl::SACSegmentation<pcl::PointXYZI> seg;
  pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
  pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
  seg.setOptimizeCoefficients(true);
  seg.setModelType(pcl::SACMODEL_PLANE);
  seg.setMethodType(pcl::SAC_RANSAC);
  seg.setMaxIterations(100);
  seg.setDistanceThreshold(0.3);
  seg.setInputCloud(cloud);
  seg.segment(*inliers, *coefficients);

  if(inliers->indices.size() == 0)            // 0??????, ????????? ?????? ???????????? ????????? ????????? ?????? ???????????? ?????? ????????????.
  {
      std::cout << "Could not estimate a planar model for the given dataset." << std::endl;
  }

  for(int index : inliers->indices)       //inliers??? ????????? ?????? ????????? ?????? ???????????? cloud?????? planeCloud??? ????????????.
  {
      PCD_cloud_p->points.push_back(cloud->points[index]);
  }

  pcl::ExtractIndices<pcl::PointXYZI> extract;
  extract.setInputCloud(cloud);       //??? reference cloud ?????? inliers??? ???????????? ?????? ???????????? ???????????? obstCloud ??? ????????????.
  extract.setIndices(inliers);
  extract.setNegative(true);
  extract.filter(*PCD_cloud_o);

  sensor_msgs::PointCloud2 ROS_cloud_o;
  sensor_msgs::PointCloud2 ROS_cloud_p;

/*
  //toPCL??? ?????? fromPCL??? ?????????????????? ??????
  pcl::PCLPointCloud2 * PCL_cloud_o(new pcl::PCLPointCloud2);
  pcl::PCLPointCloud2 * PCL_cloud_p(new pcl::PCLPointCloud2);
  pcl::toPCLPointCloud2(*PCD_cloud_o, *PCL_cloud_o);
  pcl::toPCLPointCloud2(*PCD_cloud_p, *PCL_cloud_p);
  pcl_conversions::fromPCL(*PCL_cloud_o, ROS_cloud_o);
  pcl_conversions::fromPCL(*PCL_cloud_p, ROS_cloud_p);
*/
  pcl::toROSMsg(*PCD_cloud_o, ROS_cloud_o);
  pcl::toROSMsg(*PCD_cloud_p, ROS_cloud_p);
  
  ROS_cloud_o.header.frame_id = "livox_frame";
  ROS_cloud_p.header.frame_id = "livox_frame";




  //img=cv::imread("/home/kaai/chicago_ws/src/first_pkg/src/Kitti_File/001013.png");

  //sensor_msgs::ImagePtr msg1 = cv_bridge::CvImage(std_msgs::Header(), "bgr8", img).toImageMsg();

  //Mat color_img = imread("/home/kaai/chicago_ws/src/first_pkg/src/Kitti_File/image/001039.png",IMREAD_COLOR);
  

  

  cv::Mat p(3,4,cv::DataType<double>::type);
  cv::Mat r(3,3,cv::DataType<double>::type);
  cv::Mat tr(3,4,cv::DataType<double>::type);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// ????????? projection matrix (3 X 4)
  p.at<double>(0,0) = 7.215377000000e+02;    p.at<double>(0,1) = 0.000000000000e+00;    p.at<double>(0,2) = 6.095593000000e+02;    p.at<double>(0,3) = 0.000000000000e+0;
  p.at<double>(1,0) = 0.00000000;    p.at<double>(1,1) = 7.215377000000e+02;    p.at<double>(1,2) = 1.728540000000e+02;    p.at<double>(1,3) = 0.000000000000e+0;  
  p.at<double>(2,0) = 0.00000000;    p.at<double>(2,1) = 0.00000000;    p.at<double>(2,2) = 1.00000000;    p.at<double>(2,3) = 0.000000000000e+0;  

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Rotation matrix (3 X 3)
  r.at<double>(0,0) = 9.999239000000e-01;    r.at<double>(0,1) = 9.837760000000e-03;    r.at<double>(0,2) = -7.445048000000e-03;
  r.at<double>(1,0) = -9.869795000000e-033;    r.at<double>(1,1) = 9.999421000000e-01;    r.at<double>(1,2) = -4.278459000000e-03;
  r.at<double>(2,0) = 7.402527000000e-03;    r.at<double>(2,1) = 4.351614000000e-03;    r.at<double>(2,2) = 9.999631000000e-01;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Tr_velo_to_cam (3 X 4)
  tr.at<double>(0,0) = 7.533745000000e-03;    tr.at<double>(0,1) = -9.999714000000e-01;    tr.at<double>(0,2) = -6.166020000000e-04;    tr.at<double>(0,3) = -4.069766000000e-03;  // x??? (??????????????? -)
  tr.at<double>(1,0) = 1.480249000000e-02;    tr.at<double>(1,1) = 7.280733000000e-04;    tr.at<double>(1,2) = -9.998902000000e-01;    tr.at<double>(1,3) = -7.631618000000e-02;  // y??? ??????(?????? ????????? -)
  tr.at<double>(2,0) = 9.998621000000e-01;    tr.at<double>(2,1) = 7.523790000000e-03;    tr.at<double>(2,2) = 1.480755000000e-02;    tr.at<double>(2,3) = -2.717806000000e-01;  //?????? ?????? (??????(-))
  
  
  cv::Mat X(4,1,cv::DataType<double>::type);
  cv::Mat Y(3,1,cv::DataType<double>::type);
  cv::Mat Z(4,1,cv::DataType<double>::type);
  cv::Mat F(3,1,cv::DataType<double>::type);
  cv::Mat visImg_p = color_img.clone();
  cv::Mat visImg_o = color_img.clone();
  cv::Mat overlay_p = visImg_p.clone();
  cv::Mat overlay_o = visImg_o.clone();

  cv::Point *pt_Object = new cv::Point;
  cv::Point *pt_Lane = new cv::Point;
  cv::Point pt_o;
  cv::Point pt_p;
  float object_lidar_array[3]; // for getting 3D points for object
  float lane_lidar_array[3];   // for getting 3D points for lane
  

  // [access point on plane] ------------------------------------------
  for(auto it=PCD_cloud_p->begin(); it!=PCD_cloud_p->end(); ++it) {
    if(it->x >=0){

    X.at<double>(0,0) = it->x; X.at<double>(1,0) = it->y; X.at<double>(2,0) = it->z; X.at<double>(3,0) = 1;
    Y = r * tr * X;
    Z.at<double>(0,0) = Y.at<double>(0,0); Z.at<double>(1,0) = Y.at<double>(1,0); Z.at<double>(2,0) = Y.at<double>(2,0); Z.at<double>(3,0) = 1;
    F = p * Z;
    
    pt_p.x = F.at<double>(0,0) / F.at<double>(0,2);
    pt_p.y = F.at<double>(1,0) / F.at<double>(0,2);
    //std::cout << it->z <<std::endl;

    float val = it->x;
    float intensity = it-> x;
    //ROS_INFO("intensity : %f", intensity);
    
    float maxval = 10;
    int green = std::min(255, (int)(255*abs((val - maxval)/maxval)));
    //std::cout<<red<<std::endl;
    int red = std::min(255, (int)(255*(1-abs((val - maxval)/maxval))));

    // [put the point on Image]
    cv::circle(overlay_p, pt_p, 1., cv::Scalar(it->intensity , red,green), -1);
    }
  }
 
  // [access point on plane] ------------------------------------------
  for(auto it=PCD_cloud_o->begin(); it!=PCD_cloud_o->end(); ++it) {
    if(it->x >=0){

    X.at<double>(0,0) = it->x; X.at<double>(1,0) = it->y; X.at<double>(2,0) = it->z; X.at<double>(3,0) = 1;
    Y = r * tr * X;
    Z.at<double>(0,0) = Y.at<double>(0,0); Z.at<double>(1,0) = Y.at<double>(1,0); Z.at<double>(2,0) = Y.at<double>(2,0); Z.at<double>(3,0) = 1;
    F = p * Z;
    
    pt_o.x = F.at<double>(0,0) / F.at<double>(0,2);
    pt_o.y = F.at<double>(1,0) / F.at<double>(0,2);
    //std::cout << it->z <<std::endl;

    float val = it->x;
    float intensity = it-> x;
    //ROS_INFO("intensity : %f", intensity);
    
    float maxval = 10;
    int green = std::min(255, (int)(255*abs((val - maxval)/maxval)));
    //std::cout<<red<<std::endl;
    int red = std::min(255, (int)(255*(1-abs((val - maxval)/maxval))));

    // [put the point on Image]
    cv::circle(overlay_o, pt_o, 1., cv::Scalar(it->intensity , red,green), -1);
    }
    }
  
  float opacity = 0.7;
  cv::addWeighted(overlay_p, opacity, visImg_p, 1-opacity, 0, visImg_p);
  cv::addWeighted(overlay_o, opacity, visImg_o, 1-opacity, 0, visImg_o);

  ros::Rate loop_rate(4);

  sensor_msgs::ImagePtr img_msg_obs = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg_o).toImageMsg();
  sensor_msgs::ImagePtr img_msg_plane = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg_p).toImageMsg();

  
  while (nh.ok())
  {
    //msg->header.stamp = ros::Time::now().toNSec();
    pub1.publish (ROS_cloud_p);
    pub2.publish (ROS_cloud_o);
    pub3.publish(img_msg_obs);
    pub4.publish(img_msg_plane);

    //pub1.publish(msg);

    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
