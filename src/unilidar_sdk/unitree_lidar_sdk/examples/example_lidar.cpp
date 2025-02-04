/**********************************************************************
 Copyright (c) 2020-2023, Unitree Robotics.Co.Ltd. All rights reserved.
***********************************************************************/

#include "unitree_lidar_sdk.h"
#include "unitree_lidar_sdk_pcl.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <rosbag/bag.h>

using namespace unitree_lidar_sdk;

int main(int argc, char** argv){

  ros::init(argc, argv, "unitree_pointcloud_publisher");
  ros::NodeHandle nh;
  ros::Publisher pointcloud_pub = nh.advertise<sensor_msgs::PointCloud2>("lidar_points", 10);

  // Initialize Lidar Object
  UnitreeLidarReader* lreader = createUnitreeLidarReader();
  int cloud_scan_num = 18;
  std::string port_name = "/dev/ttyUSB0";

  if ( lreader->initialize(cloud_scan_num, port_name) ){
    printf("Unilidar initialization failed! Exit here!\n");
    exit(-1);
  }else{
    printf("Unilidar initialization succeed!\n");
  }

  // Set Lidar Working Mode
  printf("Set Lidar working mode to: STANDBY ... \n");
  lreader->setLidarWorkingMode(STANDBY);
  sleep(1);

  printf("Set Lidar working mode to: NORMAL ... \n");
  lreader->setLidarWorkingMode(NORMAL);
  sleep(1);

  printf("\n");

  // Print Lidar Version
  while(true){
    if (lreader->runParse() == VERSION){
      printf("lidar firmware version = %s\n", lreader->getVersionOfFirmware().c_str() );
      break;
    }
    usleep(500);
  }
  printf("lidar sdk version = %s\n\n", lreader->getVersionOfSDK().c_str());
  sleep(2);

  // Check lidar dirty percentange
  int count_percentage = 0;
  while(true){
    if( lreader->runParse() == AUXILIARY){
      printf("Dirty Percentage = %f %%\n", lreader->getDirtyPercentage());
      if (++count_percentage > 2){
        break;
      }
      if (lreader->getDirtyPercentage() > 10){
        printf("The protection cover is too dirty! Please clean it right now! Exit here ...\n");
        exit(0);
      }
    }
    usleep(500);
  }
  printf("\n");
  sleep(2);

  // Set LED
  printf("Turn on all the LED lights ...\n");
  uint8_t led_table[45];
  for (int i=0; i < 45; i++){
    led_table[i] = 0xFF;
  }
  lreader->setLEDDisplayMode(led_table);
  sleep(2);

  printf("Turn off all the LED lights ...\n");
  for (int i=0; i < 45; i++){
    led_table[i] = 0x00;
  }
  lreader->setLEDDisplayMode(led_table);
  sleep(2);

  printf("Set LED mode to: FORWARD_SLOW ...\n");
  lreader->setLEDDisplayMode(FORWARD_SLOW);
  sleep(2);

  printf("Set LED mode to: REVERSE_SLOW ...\n");
  lreader->setLEDDisplayMode(REVERSE_SLOW);
  sleep(2);

  printf("Set LED mode to: SIXSTAGE_BREATHING ...\n");
  lreader->setLEDDisplayMode(SIXSTAGE_BREATHING);

  printf("\n");
  sleep(2);

  rosbag::Bag bag;
  bag.open("lidar_data.bag", rosbag::bagmode::Write);
  MessageType result;
  pcl::PointCloud<pcl::PointXYZI> pcl_cloud;

  // Parse PointCloud and IMU data

  while (ros::ok()) {
    result = lreader->runParse();

    if (result == POINTCLOUD) {
      // Convert SDK cloud to PCL format
      convertUnitreeLidarToPCL(lreader->getCloud(), pcl_cloud);  // SDK conversion function

      // Process all the points in the point cloud
      pcl_cloud.clear();  // Clear the previous cloud data
      for (const auto& point : lreader->getCloud().points) {
        pcl::PointXYZI pcl_point;
        pcl_point.x = point.x;
        pcl_point.y = point.y;
        pcl_point.z = point.z;
        pcl_point.intensity = point.intensity;
        pcl_cloud.push_back(pcl_point);  // Add all points to PCL cloud
      }

      // Create a PointCloud2 message from the PCL point cloud
      sensor_msgs::PointCloud2 output_msg;
      pcl::toROSMsg(pcl_cloud, output_msg);
      output_msg.header.frame_id = "lidar_frame";
      output_msg.header.stamp = ros::Time::now();

      // Publish the point cloud
      pointcloud_pub.publish(output_msg);

      // Write to rosbag
      bag.write("lidar_points", ros::Time::now(), output_msg);
    }

    ros::spinOnce();
  }

  // Close the bag
  bag.close();

  return 0;
}