
#include <ros/ros.h> 
#include <stdlib.h>
#include <math.h>

#include <sensor_msgs/PointCloud2.h> 
#include <pcl_ros/point_cloud.h> 
#include <pcl/ros/conversions.h>

#include <pcl/point_cloud.h>
#include <pcl/PCLHeader.h>

using namespace std;

#ifndef POINT_SIZE
#define POINT_SIZE 2500
#endif

#ifndef BOX_SIZE
#define BOX_SIZE 400
#endif

#ifndef PI
#define PI 3.1415926
#endif


#ifndef DOUBLE_PI
#define DOUBLE_PI 6.2831852
#endif


static float center_x = 0.0f;
static float center_y = 0.0f;
static float theta = 0.0f;

bool judge_moveable(const float&x, const float&y, const float&x_max, const float&x_min, const float&y_max, const float&y_min){
  if (x > x_min&&x<x_max&&y>y_min&&y < y_max)
    return true;
  else
    return false;
}

void move_center(){

  const float move_step = 0.1f;

  float theta_tmp = theta;
  float center_x_tmp = center_x + move_step*cos(theta);
  float center_y_tmp = center_y + move_step*sin(theta);

  while (!judge_moveable(center_x_tmp, center_y_tmp, 2.0f, -2.0f, 2.0f, -2.0f)){
    theta_tmp = static_cast<float>(DOUBLE_PI) * rand() / (RAND_MAX + 1.0f);
    center_x_tmp = center_x + move_step*cos(theta_tmp);
    center_y_tmp = center_y + move_step*sin(theta_tmp);
  }
  theta = theta_tmp;
  center_x = center_x_tmp;
  center_y = center_y_tmp;
}

void make_clouds(pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr) {
  double detla_move = 0;
  double sigma_plane_z = 0.2;
  double sigma_point[3] = {0.2,0.2,0.2};
  pcl::PointXYZ basic_point;
  int total_point = POINT_SIZE + BOX_SIZE;
  basic_cloud_ptr->points.resize(total_point);
  for (size_t i = 0; i < POINT_SIZE; ++i)
  {
    basic_cloud_ptr->points[i].x = 10.0f * rand() / (RAND_MAX + 1.0f) - 5.0f;
    basic_cloud_ptr->points[i].y = 10.0f * rand() / (RAND_MAX + 1.0f) - 5.0f;
    basic_cloud_ptr->points[i].z = 0.0f;
  }
  move_center();
  for (size_t i = POINT_SIZE; i < total_point; ++i)
  {
    basic_cloud_ptr->points[i].x = 1.0f * rand() / (RAND_MAX + 1.0f) - 0.5f+center_x;
    basic_cloud_ptr->points[i].y = 1.0f * rand() / (RAND_MAX + 1.0f) - 0.5f+center_y;
    basic_cloud_ptr->points[i].z = 1.0f * rand() / (RAND_MAX + 1.0f);
  }

  basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
  basic_cloud_ptr->height = 1;
  basic_cloud_ptr->header.frame_id = "point_cloud";
  std::cout << "publish a example point cloud with "<<(int)basic_cloud_ptr->points.size()<<"points" << std::endl;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "point_cloud_publish"); //node name
  ros::NodeHandle nh;

  //publish cloud into topic "/cloud" at 5Hz.
  ros::Publisher pubCloud = nh.advertise<sensor_msgs::PointCloud2>("/cloud", 5);
  ros::Rate loop_rate(5);
  while (ros::ok()) {

    pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr(new pcl::PointCloud<pcl::PointXYZ>);
    make_clouds(basic_cloud_ptr);
    cout << "Generating example point-cloud-publisher" << std::endl;
    cout << "cloud size: " << basic_cloud_ptr->points.size() << std::endl;
    cout << "view in rviz; choose: topic= cloud; and fixed frame= point_cloud" << endl;

    sensor_msgs::PointCloud2 ros_cloud;
    pcl::toROSMsg(*basic_cloud_ptr, ros_cloud);

    ros_cloud.header.stamp = ros::Time::now();
    ros_cloud.header.frame_id = "point_cloud";
    pubCloud.publish(ros_cloud);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}
