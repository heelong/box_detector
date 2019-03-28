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
#include "point_cloud_subscriber.h"

CloudHandler::CloudHandler(){
    pcl_sub = nh.subscribe("/cloud", 5,&CloudHandler::cloudCB,this);
    pcl_pub = nh.advertise<sensor_msgs::PointCloud2>("/cloud_filter", 5);
  }

void CloudHandler::cloudCB(const sensor_msgs::PointCloud2& cloud_in){
    pcl::PointCloud<pcl::PointXYZ>cloud;
    pcl::PointCloud<pcl::PointXYZ>cloud_filter;
    sensor_msgs::PointCloud2 outPut;
    pcl::fromROSMsg(cloud_in,cloud);

    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.01);

    seg.setInputCloud (cloud.makeShared());
    seg.segment (*inliers, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud (cloud.makeShared());
    extract.setIndices (inliers);
    extract.setNegative (true);
    extract.filter (cloud_filter);
    //seg.setNegative (true);
    //seg.filter (cloud_filter);
    if (inliers->indices.size () == 0)
    {
        PCL_ERROR ("Could not estimate a planar model for the given dataset.");
        return ;
    }

    std::cerr << "Model coefficients: " 
	<< coefficients->values[0] << " " 
	<< coefficients->values[1] << " "
	<< coefficients->values[2] << " "
	<< coefficients->values[3] << std::endl;

    pcl::toROSMsg(cloud_filter,outPut);
    outPut.header.frame_id = "point_cloud";
    pcl_pub.publish(outPut);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_subscriber"); //node name
  std::cout << "Generating example point_cloud_subscriber" << std::endl;
  CloudHandler handler;
  ros::spin();
  return 0;
}
