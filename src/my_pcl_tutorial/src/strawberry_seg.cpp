// CPP
#include <iostream>
#include <vector>
#include <cmath>
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
#include <pcl/segmentation/min_cut_segmentation.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/common/pca.h>
// Publisher
ros::Publisher pub_cloud;
//ros::Publisher pub_eigen;
//ros::Publisher pub_sphere;

void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // PCL type PCL2
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_vg;
  pcl::PCLPointCloud2 cloud_sor;
  // PCL type Point T
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg_T(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg_cp(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor_T(new pcl::PointCloud<pcl::PointXYZRGB>);
  //pcl::PCLPointCloud2 cloud_pca;
  // Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // Voxel grid downsampling
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (cloudPtr);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (cloud_vg);
  // Convert to type Point T
  pcl::fromPCLPointCloud2(cloud_vg, *cloud_vg_T);
  //std::cout<<"After voxel grid downsamlpling, there are " << cloud_vg_T->points.size() <<" points."<< std::endl;
  //*cloud_vg_cp = *cloud_vg_T;
  // Color filter
  int count = 0;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it=cloud_vg_T->begin(); it!= cloud_vg_T->end(); it++)
  {
    if(it->r<=255 && it->r >=86 && it->g>=0 && it->g <=70 && it->b >=0 && it->b <= 82)
    {
        pcl::PointXYZRGB point_to_add;
        point_to_add.x = it->x;
        point_to_add.y = it->y;
        point_to_add.z = it->z;
        point_to_add.r = it->r;
        point_to_add.g = it->g;
        point_to_add.b = it->b;
        cloud_rgb->points.push_back(point_to_add);
        count ++;
    }
  }

  cloud_rgb->width = count;
  cloud_rgb->height = 1;
  cloud_rgb->points.resize(count);

  //std::cout<<"After color filter, there are " << cloud_rgb->points.size() <<" points."<< std::endl;
    
  // Statistical outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_rgb);
  sor.setMeanK(200);
  sor.setStddevMulThresh (0.2);
  sor.filter (*cloud_sor_T);
  //std::cout<<"After statistical outlier removal, there are " << cloud_sor_T->points.size() <<" points."<< std::endl;
  // Find center for min-cut
  Eigen::Vector4f centroid; 
  pcl::compute3DCentroid(*cloud_sor_T, centroid);
  //std::cout << "Center: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")" << std::endl;
  // Min-cut segmentation
  pcl::MinCutSegmentation<pcl::PointXYZRGB> seg;
  seg.setInputCloud (cloud_vg_T);
  // Set foreground and parameters
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr foreground_points(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointXYZRGB point;
  point.x = centroid[0];
  point.y = centroid[1];
  point.z = centroid[2];
  point.r = 255;
  point.g = 255;
  point.b = 255;
  foreground_points->points.push_back(point);
  seg.setForegroundPoints (foreground_points);
  seg.setSigma (0.25);
  seg.setRadius (0.03);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);
  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr segmentation(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud <pcl::PointXYZRGB>::Ptr segmentation_cp(new pcl::PointCloud<pcl::PointXYZRGB> ());
  //pcl::visualization::CloudViewer viewer ("Cluster viewer");

  //std::cout << "Maximum flow is " << seg.getMaxFlow () << std::endl;

  pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
  
  count = 0;
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it_1 = colored_cloud->begin(); it_1 != colored_cloud->end(); it_1 ++)
  {
    //std::cout << std::distance(colored_cloud->begin(), it_1) <<std::endl;
    if(it_1->r == 255 && it_1->g== 255 && it_1->b == 255) // background with white points
    {
      count++;
      pcl::PointXYZRGB point_to_add;
      point_to_add.x = it_1->x;
      point_to_add.y = it_1->y;
      point_to_add.z = it_1->z;
      point_to_add.r = it_1->r;
      point_to_add.g = it_1->g;
      point_to_add.b = it_1->b;
      segmentation->points.push_back(point_to_add);
    }
  }
  //std::cout << count << std::endl;
  segmentation->width = count;
  segmentation->height = 1;
  segmentation->points.resize(count);
  *segmentation_cp = *segmentation;
  /*viewer.showCloud(segmentation);
  while (!viewer.wasStopped ())
  {
  } */
  // PCA
  std::cout << "PCA" << std::endl;
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(segmentation);
  Eigen::Matrix3f normal = pca.getEigenVectors(); // plane normal
  Eigen::Vector4f center = pca.getMean();         // center
  //std::cout << "Normal: " << normal << std::endl;
  //std::cout << "Center: " << center << std::endl;
  /*pcl::PointXYZ point1, point2;
  point1.x = center(0);
  point1.y = center(1);
  point1.z = center(2);
  point2.x = center(0) + normal(0, 0) * 0.1;
  point2.y = center(1) + normal(0, 1) * 0.1;
  point2.z = center(2) + normal(0, 2) * 0.1;
  pcl::visualization::PCLVisualizer viewer ("Strawberry with vector");
  viewer.addPointCloud (segmentation, "point_cloud");
  viewer.addArrow(point2, point1, 1, 0, 0, false);
  viewer.addSphere(point1, 0.005);

  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }*/
  // Sectional plane
  pcl::visualization::PCLVisualizer viewer ("Strawberry with vector");
  viewer.addPointCloud (cloud_vg_T, "point_cloud");
  pcl::PointXYZ point1; // sphere center
  double dis_array[28];
  int num[28];
  double cut_x = 0;
  double cut_y = 0;
  double cut_z = 0;
  for(int i=0; i<28; i++)
  {
    //std::cout << i <<std::endl;
    // point = center + normal_unit * (10-i)
    // point and normal determine a plane
    double x_0 = center[0] + 0.0025*normal(0,0) * (14 - i);
    double y_0 = center[1] + 0.0025*normal(0,1) * (14 - i);
    double z_0 = center[2] + 0.0025*normal(0,2) * (14 - i);
    double dis_sum = 0;
    //std::cout <<"Center: "<< x_0 << " " << y_0 << " " << z_0 << std::endl;
    int count = 0;
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = segmentation_cp->begin(); it != segmentation_cp->end(); it++)
    {
      double dis = (normal(0,0)*(it->x - x_0) + normal(0,1)*(it->y - y_0) + normal(0,2)*(it->z - z_0));
      if(dis < 0) dis = -dis;
      if(dis <= 0.0025)
      {
        dis_sum += dis;
        count ++;
        //std::cout << dis << " ";
        //segmentation_cp->erase(it);
      }
      num[i] = count;
      if(count!=0)
        {dis_array[i] = dis_sum / count;}
      else
        {dis_array[i] = 0;}
      //std::cout << it->x << " " << it->y << " " << it->z << std::endl; 
      //std::cout << dis << " ";
    }
    //std::cout << dis_array[i] << " " << num[i] << " ";
  }
  for(int i=0; i<28;i++)
  {
    std::cout << num[i] << " ";
  }
  std::cout << std::endl;
  //std::cout << std::endl;
  //count=0;
  // Method 1. Find the minimum of the array
  /*
  int min= *std::min_element(num, num+28);
  int index;
  for(int i=0;i<28;i++)
  {
    if(num[i] == min) {index = i;break;}
  }

  double x_0 = center[0] + 0.0025*normal(0,0) * (14 - index);
  double y_0 = center[1] + 0.0025*normal(0,1) * (14 - index);
  double z_0 = center[2] + 0.0025*normal(0,2) * (14 - index);
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = segmentation_cp->begin(); it != segmentation_cp->end(); it++)
  {
    double dis = (normal(0,0)*(it->x - x_0) + normal(0,1)*(it->y - y_0) + normal(0,2)*(it->z - z_0));
    if(dis < 0) dis = -dis;
    if(dis <= 0.0025)
    {
      cut_x += it->x;
      cut_y += it->y;
      cut_z += it->z;
    }
  }
  cut_x /= num[index];
  cut_y /= num[index];
  cut_z /= num[index];
  point1.x = cut_x;
  point1.y = cut_y;
  point1.z = cut_z;
  std::cout << "Stem to cut: " << cut_x << ", " << cut_y << ", " << cut_z << std::endl; 
  viewer.addSphere(point1, 0.005);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  */
  // Method 2. Find the maximum derivative
  int der[27];
  for(int i=0;i<27; i++)
  {
    der[i] = num[i+1] - num[i];
  }
  int index;
  int min = *std::min_element(der, der+27);
  for(int i=0;i<27;i++)
  {
    if(der[i] == min) {index=i+1; break;}
  }
  double x_0 = center[0] + 0.0025*normal(0,0) * (14 - index);
  double y_0 = center[1] + 0.0025*normal(0,1) * (14 - index);
  double z_0 = center[2] + 0.0025*normal(0,2) * (14 - index);
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = segmentation_cp->begin(); it != segmentation_cp->end(); it++)
  {
    double dis = (normal(0,0)*(it->x - x_0) + normal(0,1)*(it->y - y_0) + normal(0,2)*(it->z - z_0));
    if(dis < 0) dis = -dis;
    if(dis <= 0.0025)
    {
      cut_x += it->x;
      cut_y += it->y;
      cut_z += it->z;
    }
  }
  cut_x /= num[index];
  cut_y /= num[index];
  cut_z /= num[index];
  point1.x = cut_x;
  point1.y = cut_y;
  point1.z = cut_z;
  std::cout << "Stem to cut: " << cut_x << ", " << cut_y << ", " << cut_z << std::endl; 
  viewer.addSphere(point1, 0.005);
  viewer.setBackgroundColor(0.05, 0.05, 0.05, 0); // Setting background to a dark grey
  //viewer.setPosition(800, 400); // Setting visualiser window position

  while (!viewer.wasStopped ()) { // Display the visualiser until 'q' key is pressed
    viewer.spinOnce ();
  }
  // To PCL2
  pcl::toPCLPointCloud2(*segmentation, cloud_sor);
  // Convert to ROS data type
  sensor_msgs::PointCloud2 output;
  pcl_conversions::fromPCL(cloud_sor, output);

  // Publish the data
  pub_cloud.publish (output);
  
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "my_pcl_tutorial");
  ros::NodeHandle nh;
  /*try{
    nh.getParam("r_thres", r_thres);
    nh.getParam("g_thres", g_thres);
    nh.getParam("b_thres", b_thres);
    ROS_INFO("Successfully loaded threshold.");
  }
  catch(int e)
  {
    ROS_WARN("Threshold is not properly loaded from file, using default value.");

  }*/

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("/pcl2trans", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("my_pcl_tutorial/vg", 1);
  //pub_eigen = nh.advertise<visualization_msgs::Marker> ("my_pcl_tutorial/marker/eigen", 1);
  //pub_sphere = nh.advertise<visualization_msgs::Marker> ("my_pcl_tutorial/marker/sphere", 1);

  // Spin
  ros::spin ();
}
