// CPP
#include <iostream>
#include <vector>
#include <string>
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
// For debug
#include <pcl/visualization/pcl_visualizer.h>
// Publisher
ros::Publisher pub_cloud;
ros::Publisher pub_sphere;
void 
pub_marker(double x, double y, double z, std::string frame_id)
{
  
  visualization_msgs::Marker marker;
  marker.header.frame_id = frame_id;
  marker.header.stamp = ros::Time::now();
  marker.type = visualization_msgs::Marker::SPHERE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = x;
  marker.pose.position.y = y;
  marker.pose.position.z = z;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;
  // radius: 1 cm
  marker.scale.x = 0.01;
  marker.scale.y = 0.01;
  marker.scale.z = 0.01;
  marker.color.r = 0.0;
  marker.color.g = 0.0;
  marker.color.b = 1.0;
  marker.color.a = 1.0;
  marker.lifetime = ros::Duration();
  pub_sphere.publish(marker);
}
void 
cloud_cb (const sensor_msgs::PointCloud2ConstPtr& cloud_msg)
{
  // PCL type PCL2
  ros::Time now = ros::Time::now();
  pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
  pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);
  pcl::PCLPointCloud2 cloud_vg;
  pcl::PCLPointCloud2 cloud_sor;
  // PCL type Point T
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_vg_T(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_rgb(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_sor_T(new pcl::PointCloud<pcl::PointXYZRGB>);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentation(new pcl::PointCloud<pcl::PointXYZRGB> ());
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmentation_cp(new pcl::PointCloud<pcl::PointXYZRGB> ());
  // Set true to visualization
  bool debug = true;
  // Visualization
  //pcl::visualization::PCLVisualizer viewer ("Strawberry with vector");

  // 1.Convert to PCL data type
  pcl_conversions::toPCL(*cloud_msg, *cloud);

  // 2.Voxel grid downsampling
  pcl::VoxelGrid<pcl::PCLPointCloud2> vg;
  vg.setInputCloud (cloudPtr);
  vg.setLeafSize (0.001f, 0.001f, 0.001f);
  vg.filter (cloud_vg);
  // Convert to type Point T
  pcl::fromPCLPointCloud2(cloud_vg, *cloud_vg_T);
  // Information
  //std::cout<<"After voxel grid downsamlpling, there are " << cloud_vg_T->points.size() <<" points."<< std::endl;

  // 3.Color filter
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
  // Information
  //std::cout<<"After color filter, there are " << cloud_rgb->points.size() <<" points."<< std::endl;
    
  // 4.Statistical outlier removal
  pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
  sor.setInputCloud(cloud_rgb);
  sor.setMeanK(200);
  sor.setStddevMulThresh (0.2);
  sor.filter (*cloud_sor_T);
  // Information
  //std::cout<<"After statistical outlier removal, there are " << cloud_sor_T->points.size() <<" points."<< std::endl;
  // Find center for min-cut
  Eigen::Vector4f centroid; 
  pcl::compute3DCentroid(*cloud_sor_T, centroid);
  //std::cout << "Center: (" << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << ")" << std::endl;
  // Debug
  /*if(debug)
  {
    pcl::visualization::PCLVisualizer viewer("Strawberry");
    viewer.addPointCloud(cloud_sor_T, "point_cloud");
    while(!viewer.wasStopped())
    {
      viewer.spinOnce();
    }
  }*/
  // 5.Min-cut segmentation
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
  // Maybe have some more robust method for strawberry radius
  seg.setRadius (0.06);
  seg.setNumberOfNeighbours (14);
  seg.setSourceWeight (0.8);
  std::vector <pcl::PointIndices> clusters;
  seg.extract (clusters);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud = seg.getColoredCloud ();
  count = 0;
  // Find white points (background)
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = colored_cloud->begin(); it != colored_cloud->end(); it ++)
  {
    if(it->r == 255 && it->g== 255 && it->b == 255)
    {
      count++;
      pcl::PointXYZRGB point_to_add;
      point_to_add.x = it->x;
      point_to_add.y = it->y;
      point_to_add.z = it->z;
      point_to_add.r = it->r;
      point_to_add.g = it->g;
      point_to_add.b = it->b;
      segmentation->points.push_back(point_to_add);
    }
  }
  //std::cout << count << std::endl;
  segmentation->width = count;
  segmentation->height = 1;
  segmentation->points.resize(count);
  *segmentation_cp = *segmentation;
  // SOR again
  sor.setInputCloud(segmentation);
  sor.setMeanK(200);
  sor.setStddevMulThresh (0.2);
  sor.filter (*cloud_sor_T);
  // Information
  //std::cout << "After min-cut segmentation, there are " << segmentation->points.size() << " points for strawberry." << std::endl;
  // Debug
  /*if(debug)
  {
    pcl::visualization::PCLVisualizer viewer("Strawberry");
    viewer.addPointCloud(cloud_sor_T, "point_cloud");
    while(!viewer.wasStopped())
    {
      viewer.spinOnce();
    }
  }*/
  // 6.PCA
  pcl::PCA<pcl::PointXYZRGB> pca;
  pca.setInputCloud(cloud_sor_T);
  Eigen::Matrix3f normal = pca.getEigenVectors(); // plane normal
  // Ideally, dominant eigenvector of strawberry must toward beneath (positive Y), 
  // So, if dominant eigenvector toward beneath, convert it to another direction. 
  if(normal(0,1) < 0) // eigenvector toward negative Y
  {
    normal = -normal;
  }
  Eigen::Vector4f center = pca.getMean(); // center
  // Debug
  /*if(debug)
  {
    pcl::visualization::PCLVisualizer viewer("Strawberry");
    pcl::PointXYZ point1, point2;
    point1.x = center(0);
    point1.y = center(1);
    point1.z = center(2);
    point2.x = center(0) + normal(0, 0) * 0.1;
    point2.y = center(1) + normal(0, 1) * 0.1;
    point2.z = center(2) + normal(0, 2) * 0.1;
    viewer.addArrow(point2, point1, 1, 0, 0, false);
    viewer.addSphere(point1, 0.005);
    viewer.addPointCloud(cloud_sor_T, "point_cloud");
    while(!viewer.wasStopped())
    {
      viewer.spinOnce();
    }
  }*/
  //7.Sectional plane
  double dis_array[28];
  int num[28];
  double cut_x = 0;
  double cut_y = 0;
  double cut_z = 0;
  for(int i=0; i<28; i++)
  {
    // (x_0, y_0, z_0) belongs to the sectional plane
    double x_0 = center[0] + 0.0025*normal(0,0) * (14 - i);
    double y_0 = center[1] + 0.0025*normal(0,1) * (14 - i);
    double z_0 = center[2] + 0.0025*normal(0,2) * (14 - i);
    double dis_sum = 0;
    int count = 0;
    for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud_sor_T->begin(); it != cloud_sor_T->end(); it++)
    {
      double dis = (normal(0,0)*(it->x - x_0) + normal(0,1)*(it->y - y_0) + normal(0,2)*(it->z - z_0)); // Distance from points to sectinal plane
      // Take absolute value, using cmath abs function gets 0, so using if.
      if(dis < 0) dis = -dis;
      // Distance less than threshold
      if(dis <= 0.0025)
      {
        dis_sum += dis;
        count ++;
      }
      num[i] = count;
      if(count!=0)
        {dis_array[i] = dis_sum / count;}
      else
        {dis_array[i] = 0;}
    }
    // Information
    //std::cout << dis_array[i] << " " << num[i] << " ";
  }
  //8. Find the stem to cut
  //Method 1. Find the minimum of the array from center toward the eigenvector direction
  int min = num[14];
  int index = 14;
  for(int i=14;i<28;i++)
  {
    if(num[i]<= min && num[i] != 0 )
    {
      min = num[i];
      index = i;
    }  
  }
  // Information
  //std::cout << "Min :" << min << " ; index: " << index << std::endl;
  double x_0 = center[0] + 0.0025*normal(0,0) * (14 - index);
  double y_0 = center[1] + 0.0025*normal(0,1) * (14 - index);
  double z_0 = center[2] + 0.0025*normal(0,2) * (14 - index);
  for(pcl::PointCloud<pcl::PointXYZRGB>::iterator it = cloud_sor_T->begin(); it != cloud_sor_T->end(); it++)
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
  // Method 2. Find the minimum derivative
  /*
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
  */
  // Information
  //std::cout << "Stem to cut w.r.t camera_rgb_optical_frame: " << cut_x << ", " << cut_y << ", " << cut_z << std::endl; 
  // Process time
  std::cout << "Process time: " << ros::Time::now() - now << std::endl;
  // Publish sphere marker
  // Frame: camera_rgb_optical_frame
  pub_marker(cut_x, cut_y, cut_z, cloud_msg->header.frame_id);
  //ROS_INFO("(%d, %d, %d)", cut_x, cut_y, cut_z);
  // To PCL2
  //pcl::toPCLPointCloud2(*segmentation, cloud_sor);
  // Convert to ROS data type
  //sensor_msgs::PointCloud2 output;
  //output.header.frame_id = cloud_msg->header.frame_id;
  //pcl_conversions::fromPCL(cloud_sor, output);

  // Publish the data
  //pub_cloud.publish (output);
  
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
  ros::Subscriber sub = nh.subscribe ("/camera/depth_registered/points", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  
  pub_cloud = nh.advertise<sensor_msgs::PointCloud2> ("my_pcl_tutorial/segmentation", 1);
  pub_sphere = nh.advertise<visualization_msgs::Marker> ("my_pcl_tutorial/marker/stem_to_cut", 1);

  // Spin
  ros::spin ();
}
