// cpp
#include <iostream>
#include <algorithm>
// ROS
#include <ros/ros.h>
#include <ros/console.h>
// Marker
#include <visualization_msgs/Marker.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/common/pca.h>
// rgb threshold
int r_thres;
int g_thres;
int b_thres;
// Publisher
ros::Publisher pub_cloud;
ros::Publisher pub_eigen;
ros::Publisher pub_sphere;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{

  // Container for original & filtered data
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_vg;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg_T(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_r(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PCLPointCloud2 cloud_pca;
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (cloudPtr);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (cloud_vg);
  pcl::fromPCLPointCloud2(cloud_vg, *cloud_vg_T);
  std::cout<<"After voxel grid downsamlpling, there are " << cloud_vg_T->points.size() <<" points."<< std::endl;
  // Color filter
  int count = 0;
  int length = cloud_vg_T->points.size();
  for(int i= 0; i< length; ++i)
  {
    if(int(cloud_vg_T->points[i].r) > r_thres )//&& int(cloud_vg_T->points[i].g) < g_thres && int(cloud_vg_T->points[i].b) < b_thres)
    {
      ++count;
    }
  }

  std::cout << count << std::endl;
  cloud_r->width = count;
  cloud_r->height = 1;
  cloud_r->is_dense = true;
  cloud_r->points.resize(cloud_r->width * cloud_r->height);
  int j= 0;
  for(int i= 0; i< length; ++i)
  {
    if(int(cloud_vg_T->points[i].r) > r_thres )//&& int(cloud_vg_T->points[i].g) < g_thres && int(cloud_vg_T->points[i].b) < b_thres)
    {
      cloud_r->points[j].x = cloud_vg_T->points[i].x;
      cloud_r->points[j].y = cloud_vg_T->points[i].y;
      cloud_r->points[j].z = cloud_vg_T->points[i].z;
      cloud_r->points[j].r = cloud_vg_T->points[i].r;
      cloud_r->points[j].g = cloud_vg_T->points[i].g;
      cloud_r->points[j].b = cloud_vg_T->points[i].b;
      ++j;
    }
  }
  std::cout<<"After color filter, there are " << cloud_r->points.size() <<" points."<< std::endl;
  
  // Statistical outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_r);
  sor.setMeanK(200);
  sor.setStddevMulThresh (0.2);
  sor.filter (*cloud_sor);
  std::cout<<"After statistical outlier removal, there are " << cloud_sor->points.size() <<" points."<< std::endl;
  /*
  // pca
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(cloud_sor);
  Eigen::Matrix3f eigen = pca.getEigenVectors();
  Eigen::Vector4f center = pca.getMean();
  // Publish sphere
  visualization_msgs::Marker marker_sphere;
  marker_sphere.header = cloud_msg->header;
  marker_sphere.type = visualization_msgs::Marker::SPHERE;
  marker_sphere.action = visualization_msgs::Marker::ADD;
  // Pose
  marker_sphere.pose.position.x = center(0);
  marker_sphere.pose.position.y = center(1);
  marker_sphere.pose.position.z = center(2);
  marker_sphere.pose.orientation.x = 0.0;
  marker_sphere.pose.orientation.y = 0.0;
  marker_sphere.pose.orientation.z = 0.0;
  marker_sphere.pose.orientation.w = 1.0;
  // Scale
  marker_sphere.scale.x = 0.01;
  marker_sphere.scale.x = 0.01;
  marker_sphere.scale.x = 0.01;
  // Color
  marker_sphere.color.r = 1.0;
  marker_sphere.color.g = 0.0;
  marker_sphere.color.b = 0.0;
  marker_sphere.color.a = 1.0;
  // Publish
  pub_sphere.publish(marker_sphere);
  ROS_INFO("Publish sphere");

  // Publish eigenvector
  visualization_msgs::Marker marker_eigen;
  marker_eigen.header = cloud_msg->header;
  marker_eigen.type = visualization_msgs::Marker::ARROW;
  marker_eigen.action = visualization_msgs::Marker::ADD;
  // Pose
  marker_eigen.pose.position.x = center(0);
  marker_eigen.pose.position.x = center(0);
  marker_eigen.pose.position.x = center(0);
  marker_eigen.pose.orientation.x = 0.0;
  marker_eigen.pose.orientation.y = 0.0;
  marker_eigen.pose.orientation.z = 0.0;
  marker_eigen.pose.orientation.w = 1.0;
  // Scale 
  marker_eigen.scale.x = 0.01;
  marker_eigen.scale.y = 0.01;
  marker_eigen.scale.z = 0.01;
  // Color
  marker_eigen.color.r = 0.3;
  marker_eigen.color.g = 0.3;
  marker_eigen.color.b = 0.0;
  marker_eigen.color.a = 1.0;
  // End point
  marker_eigen.points[0].x = center(0) + 0.05 * eigen(0, 0);
  marker_eigen.points[0].y = center(1) + 0.05 * eigen(0, 1);
  marker_eigen.points[0].z = center(2) + 0.05 * eigen(0, 2);
  // Publish
  pub_eigen.publish(marker_eigen);
  ROS_INFO("Publish eigen");
  */
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_vg, output);

  // Publish the data
  pub_cloud.publish (output);
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  try{
    nh.getParam("r_thres", r_thres);
    nh.getParam("g_thres", g_thres);
    nh.getParam("b_thres", b_thres);
    ROS_INFO("Successfully loaded threshold.");
  }
  catch(int e)
  {
    ROS_WARN("Threshold is not properly loaded from file, using default value.");

  }

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("my_pcl_tutorial/vg", 1);
  pub_eigen = nh.advertise<visualization_msgs::Marker> ("my_pcl_tutorial/marker/eigen", 1);
  pub_sphere = nh.advertise<visualization_msgs::Marker> ("my_pcl_tutorial/marker/sphere", 1);

  // Spin
  ros::spin ();
}
