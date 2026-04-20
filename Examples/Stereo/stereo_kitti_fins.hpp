/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include <atomic>
#include <condition_variable>
#include <fins/node.hpp>
#include <mutex>
#include <queue>
#include <thread>

#include <opencv2/opencv.hpp>
#include <System.h>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <fins/utils/time.hpp>

struct ImageData {
  double timestamp_sec;
  fins::AcqTime acq_time;
  cv::Mat image;
};

class StereoKittiNode : public fins::Node {
public:
  void define() override {
    set_name("StereoKitti");
    set_description("ORB-SLAM3 stereo visual odometry for KITTI dataset.");
    set_category("SLAM");

    register_parameter<std::string>("vocabulary_file",
                                    &StereoKittiNode::on_vocabulary_update, "");
    
    register_parameter<std::string>("settings_file",
                                    &StereoKittiNode::on_settings_update, "");
    
    register_parameter<std::string>("sequence_path",
                                    &StereoKittiNode::on_sequence_path_update, "");

    register_input<cv::Mat>("left", &StereoKittiNode::on_left_image);
    register_input<cv::Mat>("right", &StereoKittiNode::on_right_image);

    register_output<nav_msgs::msg::Odometry>("odometry");
    register_output<geometry_msgs::msg::PoseStamped>("pose");
    register_output<geometry_msgs::msg::TransformStamped>("transform");
    register_output<nav_msgs::msg::Path>("path");
  }

  void initialize() override {
    logger->info("Initializing Stereo KITTI Node...");

    cv::setNumThreads(0);

    if (vocabulary_file_.empty() || settings_file_.empty()) {
      logger->error("Vocabulary file and settings file must be specified!");
      return;
    }

    // Create SLAM system
    slam_system_ = std::make_unique<ORB_SLAM3::System>(
        vocabulary_file_, settings_file_, ORB_SLAM3::System::STEREO, true);
    
    image_scale_ = slam_system_->GetImageScale();

    is_running_ = true;
    sync_thread_ = std::thread(&StereoKittiNode::sync_process, this);

    logger->info("Stereo KITTI Node initialized.");
  }

  void deinitialize() {
    is_running_ = false;
    if (sync_thread_.joinable()) {
      sync_thread_.join();
    }
    if (slam_system_) {
      slam_system_->Shutdown();
    }
  }

  void run() override {}
  void pause() override {
    deinitialize();
  }
  void reset() override {
    deinitialize();
    path_msg_.poses.clear();
    initialize();
  }

  ~StereoKittiNode() { deinitialize(); }

  void on_vocabulary_update(const std::string &val) {
    vocabulary_file_ = val;
  }

  void on_settings_update(const std::string &val) {
    settings_file_ = val;
  }

  void on_sequence_path_update(const std::string &val) {
    sequence_path_ = val;
  }

  void on_left_image(const cv::Mat &image, fins::AcqTime acq_time) {
    if (image.empty())
      return;

    std::lock_guard<std::mutex> lock(buf_mutex_);
    double t_sec = fins::to_seconds(acq_time);
    img0_buf_.push({t_sec, acq_time, image.clone()});
  }

  void on_right_image(const cv::Mat &image, fins::AcqTime acq_time) {
    if (image.empty())
      return;

    std::lock_guard<std::mutex> lock(buf_mutex_);
    double t_sec = fins::to_seconds(acq_time);
    img1_buf_.push({t_sec, acq_time, image.clone()});
  }

private:
  nav_msgs::msg::Path path_msg_;
  
  void sync_process() {
    while (is_running_) {
      cv::Mat image0, image1;
      double time = 0;
      bool new_pair_found = false;

      {
        std::lock_guard<std::mutex> lock(buf_mutex_);
        if (!img0_buf_.empty() && !img1_buf_.empty()) {
          double time0 = img0_buf_.front().timestamp_sec;
          double time1 = img1_buf_.front().timestamp_sec;

          if (time0 < time1 - 0.003) {
            img0_buf_.pop();
          } else if (time0 > time1 + 0.003) {
            img1_buf_.pop();
          } else {
            time = img0_buf_.front().timestamp_sec;
            image0 = img0_buf_.front().image;
            img0_buf_.pop();

            image1 = img1_buf_.front().image;
            img1_buf_.pop();

            new_pair_found = true;
          }
        }
      }

      if (new_pair_found && slam_system_) {
        cv::Mat img0_processed, img1_processed;

        // Apply image scaling if needed
        if (image_scale_ != 1.f) {
          int width = image0.cols * image_scale_;
          int height = image0.rows * image_scale_;
          cv::resize(image0, img0_processed, cv::Size(width, height));
          cv::resize(image1, img1_processed, cv::Size(width, height));
        } else {
          img0_processed = image0;
          img1_processed = image1;
        }

        try {
          // Track stereo images with ORB-SLAM3
          Sophus::SE3f pose = slam_system_->TrackStereo(img0_processed, img1_processed, time);
          
          // Publish results if tracking was successful
          if (!pose.matrix().isZero()) {
            publish_pose(time, pose);
          }
        } catch (const std::exception &e) {
          logger->error("SLAM TrackStereo exception: {}", e.what());
        }
      } else {
        std::this_thread::sleep_for(std::chrono::milliseconds(2));
      }
    }
  }

  void publish_pose(double timestamp_sec, const Sophus::SE3f &pose) {
    // Extract position and orientation from SE3
    Eigen::Vector3f translation = pose.translation();
    Eigen::Quaternionf rotation = pose.unit_quaternion();

    // Convert to double for ROS messages
    Eigen::Vector3d position = translation.cast<double>();
    Eigen::Quaterniond orientation = rotation.cast<double>();

    // Create odometry message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp.sec = static_cast<int32_t>(timestamp_sec);
    odom_msg.header.stamp.nanosec = static_cast<uint32_t>(
        (timestamp_sec - odom_msg.header.stamp.sec) * 1e9);
    odom_msg.header.frame_id = "world";
    odom_msg.child_frame_id = "body";

    odom_msg.pose.pose.position.x = position.x();
    odom_msg.pose.pose.position.y = position.y();
    odom_msg.pose.pose.position.z = position.z();
    odom_msg.pose.pose.orientation.w = orientation.w();
    odom_msg.pose.pose.orientation.x = orientation.x();
    odom_msg.pose.pose.orientation.y = orientation.y();
    odom_msg.pose.pose.orientation.z = orientation.z();

    this->send("odometry", odom_msg, fins::from_seconds(timestamp_sec));

    // Create pose message
    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header = odom_msg.header;
    pose_msg.pose = odom_msg.pose.pose;
    this->send("pose", pose_msg, fins::from_seconds(timestamp_sec));

    // Create transform message
    geometry_msgs::msg::TransformStamped tf_msg;
    tf_msg.header = odom_msg.header;
    tf_msg.child_frame_id = odom_msg.child_frame_id;
    tf_msg.transform.translation.x = position.x();
    tf_msg.transform.translation.y = position.y();
    tf_msg.transform.translation.z = position.z();
    tf_msg.transform.rotation.w = orientation.w();
    tf_msg.transform.rotation.x = orientation.x();
    tf_msg.transform.rotation.y = orientation.y();
    tf_msg.transform.rotation.z = orientation.z();
    this->send("transform", tf_msg, fins::from_seconds(timestamp_sec));

    // Update path
    pose_msg.header = odom_msg.header;
    path_msg_.header = odom_msg.header;
    path_msg_.poses.push_back(pose_msg);
    this->send("path", path_msg_, fins::from_seconds(timestamp_sec));
  }

  std::string vocabulary_file_;
  std::string settings_file_;
  std::string sequence_path_;
  
  std::unique_ptr<ORB_SLAM3::System> slam_system_;
  float image_scale_;

  std::queue<ImageData> img0_buf_;
  std::queue<ImageData> img1_buf_;
  std::mutex buf_mutex_;

  std::thread sync_thread_;
  std::atomic<bool> is_running_{false};
};

EXPORT_NODE(StereoKittiNode)
