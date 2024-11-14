#include "featureAssociation.h"
#include "imageProjection.h"
#include "mapOptimization.h"
#include "transformFusion.h"
#include <chrono>

int main(int argc, char** argv) {
  ros::init(argc, argv, "lego_loam");

  ros::NodeHandle nh("~");
  std::string imu_topic;
  std::string lidar_topic;

  nh.getParam("imu_topic", imu_topic);
  nh.getParam("lidar_topic", lidar_topic);

  Channel<ProjectionOut> projection_out_channel(true);
  Channel<AssociationOut> association_out_channel(false);

  ImageProjection IP(nh, projection_out_channel);

  FeatureAssociation FA(nh, projection_out_channel,
                        association_out_channel);

  MapOptimization MO(nh, association_out_channel);

  TransformFusion TF(nh);

  ROS_INFO("\033[1;32m---->\033[0m LeGO-LOAM Started.");

  ROS_INFO("SPINNER");
  ros::MultiThreadedSpinner spinner(4);  // Use 4 threads
  spinner.spin();

  // must be called to cleanup threads
  ros::shutdown();

  return 0;
}


