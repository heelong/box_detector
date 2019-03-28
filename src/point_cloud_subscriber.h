#include <ros/ros.h>
// PCL specific includes
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
//ÃÌº”“˝”√
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <iostream>
class CloudHandler{
public:
  CloudHandler();
  void cloudCB(const sensor_msgs::PointCloud2& cloud_in);
  void estimateBoxCenter(pcl::PointCloud<pcl::PointXYZ>&cloud_box);
  void saveBox(const sensor_msgs::PointCloud2& cloud_box);
private:
  ros::NodeHandle nh;
  ros::Subscriber pcl_sub;
  ros::Subscriber pcd_sub;
  ros::Publisher pcl_pub;
};
