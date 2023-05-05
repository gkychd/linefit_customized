#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <tf2_ros/transform_listener.h>

#include "ground_segmentation/ground_segmentation.h"

#include "ground_segmentation/kitti_loader.hpp"

#include <visualization_msgs/Marker.h>

using namespace std;
std::string data_path;
std::string pcd_savepath;

std::string output_filename;
string output_gtpoints_name;
bool save_csv;
bool save_flag;
ros::Publisher ground_pub_;
ros::Publisher obstacle_pub_;

ros::Publisher CloudPublisher;
ros::Publisher TPPublisher;
ros::Publisher FPPublisher;
ros::Publisher FNPublisher;
ros::Publisher PrecisionPublisher;
ros::Publisher RecallPublisher;

void pub_score(std::string mode, double measure) {
    static int                 SCALE = 5;
    visualization_msgs::Marker marker;
    marker.header.frame_id                  = "map";
    marker.header.stamp                     = ros::Time();
    marker.ns                               = "my_namespace";
    marker.id                               = 0;
    marker.type                             = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action                           = visualization_msgs::Marker::ADD;
    if (mode == "p") marker.pose.position.x = 28.5;
    if (mode == "r") marker.pose.position.x = 25;
    marker.pose.position.y                  = 30;

    marker.pose.position.z    = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x            = SCALE;
    marker.scale.y            = SCALE;
    marker.scale.z            = SCALE;
    marker.color.a            = 1.0; // Don't forget to set the alpha!
    marker.color.r            = 0.0;
    marker.color.g            = 1.0;
    marker.color.b            = 0.0;
    //only if using a MESH_RESOURCE marker type:
    marker.text               = mode + ": " + std::to_string(measure);
    if (mode == "p") PrecisionPublisher.publish(marker);
    if (mode == "r") RecallPublisher.publish(marker);
}

template<typename T>
sensor_msgs::PointCloud2 cloud2msg(pcl::PointCloud<T> cloud, std::string frame_id = "map") {
    sensor_msgs::PointCloud2 cloud_ROS;
    pcl::toROSMsg(cloud, cloud_ROS);
    cloud_ROS.header.frame_id = frame_id;
    return cloud_ROS;
}


int main(int argc, char** argv) {
  ros::init(argc, argv, "ground_segmentation");

  ros::NodeHandle nh("~");

  ground_pub_ = nh.advertise<pcl::PointCloud<PointType>>("/ground_cloud", 1);
  obstacle_pub_ = nh.advertise<pcl::PointCloud<PointType>>("obstacle_cloud", 1);

  CloudPublisher     = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/cloud", 100, true);
  TPPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/TP", 100, true);
  FPPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FP", 100, true);
  FNPublisher        = nh.advertise<sensor_msgs::PointCloud2>("/benchmark/FN", 100, true);
  PrecisionPublisher = nh.advertise<visualization_msgs::Marker>("/precision", 1, true);
  RecallPublisher    = nh.advertise<visualization_msgs::Marker>("/recall", 1, true);


  // Do parameter stuff.
  GroundSegmentationParams params;
  //nh.param("visualize", params.visualize, params.visualize);
  nh.param("n_bins", params.n_bins, params.n_bins);
  nh.param("n_segments", params.n_segments, params.n_segments);
  nh.param("max_dist_to_line", params.max_dist_to_line, params.max_dist_to_line);
  nh.param("max_slope", params.max_slope, params.max_slope);
  nh.param("min_slope", params.min_slope, params.min_slope);
  nh.param("long_threshold", params.long_threshold, params.long_threshold);
  nh.param("max_long_height", params.max_long_height, params.max_long_height);
  nh.param("max_start_height", params.max_start_height, params.max_start_height);
  nh.param("sensor_height", params.sensor_height, params.sensor_height);
  nh.param("line_search_angle", params.line_search_angle, params.line_search_angle);
  nh.param("n_threads", params.n_threads, params.n_threads);
  // Params that need to be squared.
  double r_min, r_max, max_fit_error;
  if (nh.getParam("r_min", r_min)) {
    params.r_min_square = r_min*r_min;
  }
  if (nh.getParam("r_max", r_max)) {
    params.r_max_square = r_max*r_max;
  }
  if (nh.getParam("max_fit_error", max_fit_error)) {
    params.max_error_square = max_fit_error * max_fit_error;
  }

  std::string ground_topic, obstacle_topic, input_topic;
  bool latch;
  nh.param<std::string>("input_topic", input_topic, "input_cloud");
  nh.param<std::string>("ground_output_topic", ground_topic, "ground_cloud");
  nh.param<std::string>("obstacle_output_topic", obstacle_topic, "obstacle_cloud");
  nh.param("latch", latch, false);

  //for evo
  nh.param<string>("data_path", data_path, "/");
  nh.param<bool>("save_flag", save_flag, false);
  nh.param<bool>("save_csv", save_csv, false);
  nh.param<string>("pcd_savepath", pcd_savepath, "/");
  nh.param<string>("output_gtpoints_name", output_gtpoints_name, "/");
  nh.param<string>("output_filename", output_filename, "/");


  cout << input_topic << endl;
  cout << data_path << endl;
  KittiLoader loader(data_path);
  int      N = loader.size();
  for (int n = 0; n < N; ++n) {
    std::vector<int> labels;
    double time_taken;
    cout << n << "th node come" << endl;
    pcl::PointCloud<PointType> pc_curr;
    loader.get_cloud(n, pc_curr);
    GroundSegmentation segmenter_(params);
    const pcl::PointCloud<PointType>& cloud_proc = pc_curr;
    segmenter_.segment(cloud_proc, &labels, time_taken);
    pcl::PointCloud<PointType> ground_cloud, obstacle_cloud;
    ground_cloud.header = cloud_proc.header;
    obstacle_cloud.header = cloud_proc.header;
    for (size_t i = 0; i < cloud_proc.size(); ++i) {
      if (labels[i] == 1) ground_cloud.push_back(cloud_proc[i]);
      else obstacle_cloud.push_back(cloud_proc[i]);
    }
    // Estimation
    double precision, recall, precision_naive, recall_naive;
    int gt_ground_nums;
    calculate_precision_recall(pc_curr, ground_cloud, precision, recall, gt_ground_nums);
    calculate_precision_recall(pc_curr, ground_cloud, precision_naive, recall_naive, gt_ground_nums, false);
    cout << "\033[1;33m" << n << "th, " << "gt ground nums: " << gt_ground_nums << "\033[0m" << endl;

    cout << "\033[1;32m P: " << precision << " | R: " << recall << "\033[0m" << endl;

    ground_pub_.publish(ground_cloud);
    obstacle_pub_.publish(obstacle_cloud);

    // -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save precision/recall in a text file, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
if(save_csv){
    cout << "output_filename" << output_filename << endl;
    ofstream ground_output(output_filename, ios::app);
    ground_output << n << "," << time_taken << "," << precision << "," << recall << "," << precision_naive << "," << recall_naive;
    ground_output << std::endl;
    ground_output.close();
}


// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        // Publish msg
        pcl::PointCloud<PointType> TP;
        pcl::PointCloud<PointType> FP;
        pcl::PointCloud<PointType> FN;
        pcl::PointCloud<PointType> TN;
        discern_ground(ground_cloud, TP, FP);
        discern_ground(obstacle_cloud, FN, TN);

// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
//        If you want to save the output of pcd, revise this part
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -
        if (save_flag) {
            std::map<int, int> pc_curr_gt_counts, g_est_gt_counts;
            double             accuracy;
            //save_all_accuracy(pc_curr, pc_ground, acc_filename, accuracy, pc_curr_gt_counts, g_est_gt_counts);

            std::string count_str        = std::to_string(n);
            std::string count_str_padded = std::string(NUM_ZEROS - count_str.length(), '0') + count_str;
            std::string pcd_filename     = pcd_savepath + "/" + count_str_padded + ".pcd";
            pc2pcdfile(TP, FP, FN, TN, pcd_filename);
        }
// -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -  -

        CloudPublisher.publish(cloud2msg(pc_curr));
        TPPublisher.publish(cloud2msg(TP));
        FPPublisher.publish(cloud2msg(FP));
        FNPublisher.publish(cloud2msg(FN));
        pub_score("p", precision);
        pub_score("r", recall);


  }
  ros::spin();

  return 0;

}
