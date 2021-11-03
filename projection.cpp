/*
Define Necessary Modules
*/

#include <iostream>
#include <string>
#include <math.h>

#include <ros/ros.h>

#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/io/pcd_io.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui.hpp>

#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>

using namespace cv;
typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;
using namespace std;

/*
Define Output Image Size
*/
cv::Mat img(720,1280,CV_8UC3, cv::Scalar(255,255,255));


int main(int argc, char** argv)
{
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  double x_cloud; double y_cloud; double z_cloud;
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr PCD_cloud(new pcl::PointCloud<pcl::PointXYZI>);
  
  ros::Publisher pub1 = nh.advertise<PointCloud> ("point_cloud", 1);
  ros::Publisher pub2 = nh.advertise<sensor_msgs::Image> ("projected_image", 1);
  
  pcl::io::loadPCDFile<pcl::PointXYZI> ("/home/kaai/chicago_ws/src/first_pkg/src/KITTI_cali_application/pcd/1313.pcd", *cloud);
  Mat color_img = imread("/home/kaai/chicago_ws/src/first_pkg/src/KITTI_cali_application/png/001313.png",IMREAD_COLOR);
  
  sensor_msgs::PointCloud2 ROS_cloud;

  pcl::toROSMsg(*PCD_cloud, ROS_cloud);
  
  ROS_cloud.header.frame_id = "livox_frame";

  cv::Mat p(3,4,cv::DataType<double>::type);
  cv::Mat r(3,3,cv::DataType<double>::type);
  cv::Mat tr(3,4,cv::DataType<double>::type);
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
  /// 카메라 projection matrix (3 X 4)
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
  tr.at<double>(0,0) = 7.533745000000e-03;    tr.at<double>(0,1) = -9.999714000000e-01;    tr.at<double>(0,2) = -6.166020000000e-04;    tr.at<double>(0,3) = -4.069766000000e-03;  // x축 (좌측이동시 -)
  tr.at<double>(1,0) = 1.480249000000e-02;    tr.at<double>(1,1) = 7.280733000000e-04;    tr.at<double>(1,2) = -9.998902000000e-01;    tr.at<double>(1,3) = -7.631618000000e-02;  // y축 이동(위로 이동시 -)
  tr.at<double>(2,0) = 9.998621000000e-01;    tr.at<double>(2,1) = 7.523790000000e-03;    tr.at<double>(2,2) = 1.480755000000e-02;    tr.at<double>(2,3) = -2.717806000000e-01;  //확대 축소 (줌인(-))
  
  
  cv::Mat X(4,1,cv::DataType<double>::type);
  cv::Mat Y(3,1,cv::DataType<double>::type);
  cv::Mat Z(4,1,cv::DataType<double>::type);
  cv::Mat F(3,1,cv::DataType<double>::type);

  cv::Mat visImg = color_img.clone();
  cv::Mat overlay = visImg.clone();

  cv::Point *pt_Object = new cv::Point;
  cv::Point *pt_Lane = new cv::Point;
  cv::Point pt;

 
  // [access point on everything] ------------------------------------------
  for(auto it=PCD_cloud->begin(); it!=PCD_cloud->end(); ++it)
  {
    if(it->x >=0)
    {
      X.at<double>(0,0) = it->x; X.at<double>(1,0) = it->y; X.at<double>(2,0) = it->z; X.at<double>(3,0) = 1;
      Y = r * tr * X;
      Z.at<double>(0,0) = Y.at<double>(0,0); Z.at<double>(1,0) = Y.at<double>(1,0); Z.at<double>(2,0) = Y.at<double>(2,0); Z.at<double>(3,0) = 1;
      F = p * Z;

      pt.x = F.at<double>(0,0) / F.at<double>(0,2);
      pt.y = F.at<double>(1,0) / F.at<double>(0,2);
      
      float val = it->x;
      float intensity = it-> intensity;

      float maxval = 10;
      int green = std::min(255, (int)(255*abs((val - maxval)/maxval)));
      //std::cout<<red<<std::endl;
      int red = std::min(255, (int)(255*(1-abs((val - maxval)/maxval))));

      // [put the point on Image]
      cv::circle(overlay, pt, 1., cv::Scalar(it->intensity , red,green), -1);
    }
  }
  float opacity = 0.7;
  cv::addWeighted(overlay, opacity, visImg, 1-opacity, 0, visImg);
  ros::Rate loop_rate(4);
  sensor_msgs::ImagePtr img_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", visImg).toImageMsg();

  
  while (nh.ok())
  {
    pub1.publish(ROS_cloud);
    pub2.publish(img_msg);
    ros::spinOnce ();
    loop_rate.sleep ();
  }
}
