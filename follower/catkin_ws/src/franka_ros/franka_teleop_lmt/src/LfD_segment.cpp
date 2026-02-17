//============================================================================
// Name        : LfD_segment.cpp
// Author      : Basak Gülecyüz (basak.guelecyuez@tum.de)
//               Kaiyuan Luo (kaiyuan.luo@tum.de)
// Version     : Feb 2026
// Description : LfD Reproduction node
//============================================================================

// General
#include <assert.h>
#include <gtest/gtest.h>
#include <pthread.h>
#include <time.h>
#include <queue>
#include <string>
#include <vector>
#include <fstream>

// Eigen
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/SVD>
#include <eigen3/unsupported/Eigen/MatrixFunctions>

// ROS
//#include <controller_manager_msgs/SwitchController.h>
#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/String.h>
#include <ros/package.h>


////////////////////////////////////////////////////////////////
// Declared Variables for ROS
////////////////////////////////////////////////////////////////
ros::NodeHandle* n;                 // LfD ROS Node
ros::Publisher* LfD_Pub;            // LfD publisher

////////////////////////////////////////////////////////////////
// Declared Variables for LfD Reproduction
////////////////////////////////////////////////////////////////

// Path to package
std::string package_path = ros::package::getPath("franka_teleop_lmt");


// Variables for learned trajectory
const int repro_duration = 50000; // ms
double TimerPeriodHaptic = 0.001;
double learned_trajectory[3][int(repro_duration)] = {0};
double Vl[3] = {0, 0, 0}; // learned velocity
double number_columns[3] = {0, 0, 0};

// Variables to be communicated to follower robot 
std::vector<double> command(3, 0.0);        

// Load one segment CSV into learned_trajectory and return its length and gripper_state
static bool loadSegment(const std::string& path,
                        double learned_trajectory[3][int(repro_duration)],
                        double number_columns[3],
                        int& path_length,
                        int& gripper_state) {
  std::ifstream iFile(path.c_str());
  if (!iFile.good()) {
    ROS_WARN_STREAM("Cannot open segment: " << path);
    return false;
  }

  // Clear buffers
  for (int d = 0; d < 3; ++d) {
    for (int t = 0; t < int(repro_duration); ++t) {
      learned_trajectory[d][t] = 0.0;
    }
    number_columns[d] = 0.0;
  }

  std::string line;
  int dim_count = 0;           // 0: gripper row, 1..3: x,y,z rows
  int local_path_length = 0;
  int local_gripper_state = 0;

  while (std::getline(iFile, line)) {
    std::stringstream ss(line);
    int time_count = 0;
    while (ss) {
      std::string token;
      if (!std::getline(ss, token, ',')) break;
      std::stringstream sss(token);
      double tm = 0.0;
      sss >> tm;

      if (dim_count == 0) {
        // First row: gripper state (last value in the row used)
        local_gripper_state = static_cast<int>(tm);
      } else if (dim_count - 1 < 3 && time_count < int(repro_duration)) {
        // Rows 1..3 -> indices 0..2 in learned_trajectory
        learned_trajectory[dim_count - 1][time_count] = tm;  // x, y, z
      }
      ++time_count;
    }

    if (dim_count > 0) {
      number_columns[dim_count - 1] = time_count;
      if (local_path_length < time_count) local_path_length = time_count;
    }
    ++dim_count;
  }

  path_length = local_path_length;
  gripper_state = local_gripper_state;
  ROS_INFO_STREAM("Loaded segment " << path << " length=" << path_length << " gripper=" << gripper_state);
  return path_length > 0;
}


int main(int argc, char** argv) {
  // Initialize ROS
  ros::init(argc, argv, "LfD_segment");
  n = new ros::NodeHandle;

  // Leader publisher for commands
  LfD_Pub = new ros::Publisher;
  *LfD_Pub = n->advertise<std_msgs::Float64MultiArray>("/cartesian_impedance_example_controller/LFcommand", 1, false);

  // Sleep for 5 seconds
  ros::Duration(5.0).sleep();
  ROS_INFO("Publisher is set!");

  ros::Rate loop_rate(1000);

  // Discover segments: learned_seg_1.csv, learned_seg_2.csv, ...
  std::vector<std::string> segments;
  for (int idx = 1;; ++idx) {
    std::string seg_path = package_path + "/TeleopData/learned_seg_" + std::to_string(idx) + ".csv";
    std::ifstream test(seg_path.c_str());
    if (!test.good()) break;
    segments.push_back(seg_path);
  }
  // Fallback to single file if no segments found
  if (segments.empty()) {
    // segments.push_back(package_path + "/TeleopData/learned.csv");
	ROS_WARN_STREAM("No segments found.");
	return 0;
  }

  std_msgs::Float64MultiArray msg;

  for (const auto& seg_path : segments) {
    int path_length = 0;
    int gripper_state = 0;
    if (!loadSegment(seg_path, learned_trajectory, number_columns, path_length, gripper_state)) {
      ROS_WARN_STREAM("Skipping segment due to load failure: " << seg_path);
      continue;
    }

    int time_counter = 1;
    while (ros::ok() && time_counter <= path_length) {
      // Compute velocity from position
      if (time_counter <= path_length - 10) {
        for (int i = 0; i < 3; ++i) {
          Vl[i] = (learned_trajectory[i][time_counter] - learned_trajectory[i][time_counter - 1]) / TimerPeriodHaptic;
        }
      } else {
        for (int i = 0; i < 3; ++i) Vl[i] = 0.0;
      }

      // Debug prints
      std::cout << "Vl: [" << Vl[0] << ", " << Vl[1] << ", " << Vl[2]
                << "] gripper: " << gripper_state
                << " path_length: " << path_length
                << " seg: " << seg_path << " t: " << time_counter << std::endl;

      // Build and publish command: [Vl_x, Vl_y, Vl_z, 0, 0, 0, gripper]
      command.clear();
      command.insert(command.end(), Vl, Vl + 3);
      command.insert(command.end(), 3, 0.0);                       // euler angle error (zeros)
      command.push_back(static_cast<double>(gripper_state));        // gripper

      msg.data = command;
      LfD_Pub->publish(msg);

      ++time_counter;
      ros::spinOnce();
      loop_rate.sleep();
    }
  }

  return 0;
}
