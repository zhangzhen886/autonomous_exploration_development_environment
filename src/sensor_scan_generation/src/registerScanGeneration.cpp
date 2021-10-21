#include <math.h>
#include <time.h>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud2.h>

#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

using namespace std;

pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudIn(new pcl::PointCloud<pcl::PointXYZI>());
pcl::PointCloud<pcl::PointXYZI>::Ptr laserCLoudInSensorFrame(new pcl::PointCloud<pcl::PointXYZI>());

double robotX = 0;
double robotY = 0;
double robotZ = 0;
double roll = 0;
double pitch = 0;
double yaw = 0;

bool newTransformToMap = false;

string global_frame_ = "map";
string base_frame_ = "base_link";
string sensor_frame_ = "velodyne";

std::shared_ptr<tf::TransformBroadcaster> tf_broadcaster_;
std::shared_ptr<tf::TransformListener> tf_listener_;

tf::StampedTransform transform_map2sensor;
tf::StampedTransform transform_map2base;
tf::StampedTransform transform_base2sensor;

nav_msgs::Odometry odometryIn;
ros::Publisher *pubOdometryPointer = NULL;
ros::Publisher pubLaserCloud;

void laserCloudAndOdometryHandler(const nav_msgs::Odometry::ConstPtr& odometry,
                                  const sensor_msgs::PointCloud2ConstPtr& laserCloud2)
{
  // ROS_INFO("laserCloudAndOdometryHandler.");
  laserCloudIn->clear();
  laserCLoudInSensorFrame->clear();

  pcl::fromROSMsg(*laserCloud2, *laserCloudIn);

  odometryIn = *odometry;

  // "map" to "base"
  transform_map2base.setOrigin(
      tf::Vector3(odometryIn.pose.pose.position.x, odometryIn.pose.pose.position.y, odometryIn.pose.pose.position.z));
  transform_map2base.setRotation(tf::Quaternion(odometryIn.pose.pose.orientation.x, odometryIn.pose.pose.orientation.y,
                                            odometryIn.pose.pose.orientation.z, odometryIn.pose.pose.orientation.w));

  transform_map2sensor.mult(transform_map2base, transform_base2sensor);

  int laserCloudInNum = laserCloudIn->points.size();

  pcl::PointXYZI p1;
  tf::Vector3 vec;

  // 点云转换到scan(sensor)坐标系
  for (int i = 0; i < laserCloudInNum; i++)
  {
    p1 = laserCloudIn->points[i];
    vec.setX(p1.x);
    vec.setY(p1.y);
    vec.setZ(p1.z);

    vec = transform_map2sensor * vec;

    p1.x = vec.x();
    p1.y = vec.y();
    p1.z = vec.z();

    laserCLoudInSensorFrame->points.push_back(p1);
  }

  // odometry换用"/sensor_at_scan"frame后直接转发
  odometryIn.header.stamp = laserCloud2->header.stamp;
  odometryIn.header.frame_id = "/map";
  odometryIn.child_frame_id = "/sensor";
  odometryIn.pose.pose.position.x = transform_map2sensor.getOrigin().x();
  odometryIn.pose.pose.position.y = transform_map2sensor.getOrigin().y();
  odometryIn.pose.pose.position.z = transform_map2sensor.getOrigin().z();
  odometryIn.pose.pose.orientation.x = transform_map2sensor.getRotation().x();
  odometryIn.pose.pose.orientation.y = transform_map2sensor.getRotation().y();
  odometryIn.pose.pose.orientation.z = transform_map2sensor.getRotation().z();
  odometryIn.pose.pose.orientation.w = transform_map2sensor.getRotation().w();
  pubOdometryPointer->publish(odometryIn);

  // transform_map2sensor.stamp_ = laserCloud2->header.stamp;
  // transform_map2sensor.frame_id_ = "/map";
  // transform_map2sensor.child_frame_id_ = "/sensor";
  // tfBroadcasterPointer->sendTransform(transform_map2sensor);

  sensor_msgs::PointCloud2 scan_data;
  pcl::toROSMsg(*laserCLoudInSensorFrame, scan_data);
  scan_data.header.stamp = laserCloud2->header.stamp;
  scan_data.header.frame_id = "/map";
  pubLaserCloud.publish(scan_data);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "sensor_scan");
  ros::NodeHandle nh;
  ros::NodeHandle nhPrivate = ros::NodeHandle("~");

  // ROS message filters
  message_filters::Subscriber<nav_msgs::Odometry> subOdometry;
  message_filters::Subscriber<sensor_msgs::PointCloud2> subLaserCloud;
  typedef message_filters::sync_policies::ApproximateTime<nav_msgs::Odometry, sensor_msgs::PointCloud2> syncPolicy;
  typedef message_filters::Synchronizer<syncPolicy> Sync;
  boost::shared_ptr<Sync> sync_;
  subOdometry.subscribe(nh, "/odom", 1);
  subLaserCloud.subscribe(nh, "/velodyne_points", 1);
  sync_.reset(new Sync(syncPolicy(100), subOdometry, subLaserCloud));
  sync_->registerCallback(boost::bind(laserCloudAndOdometryHandler, _1, _2));

  ros::Publisher pubOdometry = nh.advertise<nav_msgs::Odometry> ("/state_estimation_at_scan", 5);
  pubOdometryPointer = &pubOdometry;
  pubLaserCloud = nh.advertise<sensor_msgs::PointCloud2>("/registered_scan", 10);

  tf_broadcaster_ = std::make_shared<tf::TransformBroadcaster>();
  tf_listener_ = std::make_shared<tf::TransformListener>();
  try {
    tf_listener_->waitForTransform(base_frame_, sensor_frame_, ros::Time(0.0), ros::Duration(5.0));
    tf_listener_->lookupTransform(base_frame_, sensor_frame_, ros::Time(0.0), transform_base2sensor);
  } catch (tf::TransformException &ex) {
    ROS_ERROR_STREAM("Error getting TF transform: " << ex.what());
    return 1;
  }

  ros::spin();

  return 0;
}
