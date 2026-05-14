/**
 * @file lidar_mapper_rviz.cpp
 * @author jango175
 * @brief LiDAR mapper visualizer ROS2 node
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include <filesystem>
#include <string>
#include <random>
#include <fstream>
#include <limits>
#include <memory>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <tf2/LinearMath/Quaternion.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <tf2_eigen/tf2_eigen.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>
#include <pcl_ros/transforms.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>


// LidarMappervisualizer node class
class LidarMappervisualizer : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new LidarMapperVisualizer object
   * 
   */
  LidarMappervisualizer() : Node("lidar_mapper_visualizer")
  {
    // parameters
    auto lidar_mount_roll_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_roll_param_desc.description = "LiDAR mount roll angle in degrees";
    this->declare_parameter("lidar_mount_roll_deg", 0.0, lidar_mount_roll_param_desc);

    auto lidar_mount_pitch_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_pitch_param_desc.description = "LiDAR mount pitch angle in degrees";
    this->declare_parameter("lidar_mount_pitch_deg", 30.0, lidar_mount_pitch_param_desc);

    auto lidar_mount_yaw_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_yaw_param_desc.description = "LiDAR mount yaw angle in degrees";
    this->declare_parameter("lidar_mount_yaw_deg", 0.0, lidar_mount_yaw_param_desc);

    auto lidar_mount_offset_x_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_offset_x_param_desc.description = "LiDAR mount offset in X axis in metres";
    this->declare_parameter("lidar_mount_offset_x", 0.088, lidar_mount_offset_x_param_desc);

    auto lidar_mount_offset_y_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_offset_y_param_desc.description = "LiDAR mount offset in Y axis in metres";
    this->declare_parameter("lidar_mount_offset_y", 0.0, lidar_mount_offset_y_param_desc);

    auto lidar_mount_offset_z_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_offset_z_param_desc.description = "LiDAR mount offset in Z axis in metres";
    this->declare_parameter("lidar_mount_offset_z", 0.073, lidar_mount_offset_z_param_desc);

    auto world_link_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    world_link_param_desc.description = "World link name";
    this->declare_parameter("world_link", "map", world_link_param_desc);

    auto drone_link_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    drone_link_param_desc.description = "Drone link name";
    this->declare_parameter("drone_link", "base_link", drone_link_param_desc);

    auto lidar_link_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_link_param_desc.description = "LiDAR link name";
    this->declare_parameter("lidar_link", "ldlidar_link", lidar_link_param_desc);

    auto mf_timeout_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    mf_timeout_param_desc.description = "Timeout for message filter tf buffer in seconds";
    this->declare_parameter("mf_timeout", 0.25, mf_timeout_param_desc);

    auto timestamp_tolerance_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timestamp_tolerance_param_desc.description = "Timestamp tolerance for laser interpolation in seconds";
    this->declare_parameter("timestamp_tolerance", 0.11, timestamp_tolerance_param_desc);

    auto window_size_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    window_size_param_desc.description = "Window size for global map in metres";
    this->declare_parameter("window_size", 20.0, window_size_param_desc);

    auto sor_mean_k_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    sor_mean_k_param_desc.description = "Number of neighbors to analyze for SOR filter";
    this->declare_parameter("sor_mean_k", 50, sor_mean_k_param_desc);

    auto sor_std_dev_mult_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    sor_std_dev_mult_param_desc.description = "standard deviation multiplier for SOR filter";
    this->declare_parameter("sor_std_dev_mult", 1.0, sor_std_dev_mult_param_desc);

    auto fake_3d_lidar_scan_num_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    fake_3d_lidar_scan_num_param_desc.description = "Number of scans to accumulate for fake 3D LiDAR point cloud";
    this->declare_parameter("fake_3d_lidar_scan_num", 20, fake_3d_lidar_scan_num_param_desc);

    auto fake_3d_lidar_overlap_scan_num_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    fake_3d_lidar_overlap_scan_num_param_desc.description = "Number of overlapping scans for fake 3D LiDAR point cloud";
    this->declare_parameter("fake_3d_lidar_overlap_scan_num", 10, fake_3d_lidar_overlap_scan_num_param_desc);

    auto simulate_noise_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    simulate_noise_param_desc.description = "Set to true to add a noise to pose for simulation";
    this->declare_parameter("simulate_noise", false, simulate_noise_param_desc);

    const double lidar_mount_roll_deg = this->get_parameter("lidar_mount_roll_deg").as_double();
    const double lidar_mount_pitch_deg = this->get_parameter("lidar_mount_pitch_deg").as_double();
    const double lidar_mount_yaw_deg = this->get_parameter("lidar_mount_yaw_deg").as_double();
    const double lidar_mount_offset_x = this->get_parameter("lidar_mount_offset_x").as_double();
    const double lidar_mount_offset_y = this->get_parameter("lidar_mount_offset_y").as_double();
    const double lidar_mount_offset_z = this->get_parameter("lidar_mount_offset_z").as_double();
    RCLCPP_INFO(this->get_logger(), "LiDAR mounting roll: %f deg", lidar_mount_roll_deg);
    RCLCPP_INFO(this->get_logger(), "LiDAR mounting pitch: %f deg", lidar_mount_pitch_deg);
    RCLCPP_INFO(this->get_logger(), "LiDAR mounting yaw: %f deg", lidar_mount_yaw_deg);
    RCLCPP_INFO(this->get_logger(), "LiDAR mounting offset X: %f m", lidar_mount_offset_x);
    RCLCPP_INFO(this->get_logger(), "LiDAR mounting offset Y: %f m", lidar_mount_offset_y);
    RCLCPP_INFO(this->get_logger(), "LiDAR mounting offset Z: %f m", lidar_mount_offset_z);

    world_link_ = this->get_parameter("world_link").as_string();
    drone_link_ = this->get_parameter("drone_link").as_string();
    lidar_link_ = this->get_parameter("lidar_link").as_string();
    RCLCPP_INFO(this->get_logger(), "TF: %s -> %s -> %s", world_link_.c_str(), drone_link_.c_str(), lidar_link_.c_str());

    double mf_timeout = this->get_parameter("mf_timeout").as_double();
    double timestamp_tolerance = this->get_parameter("timestamp_tolerance").as_double();
    RCLCPP_INFO(this->get_logger(), "Using message filter timeout: %f s", mf_timeout);
    RCLCPP_INFO(this->get_logger(), "Using timestamp tolerance: %f s", timestamp_tolerance);

    if (timestamp_tolerance >= mf_timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "mf_timeout is smaller than timestamp_tolerance! Exiting...");
      return;
    }

    window_size_ = this->get_parameter("window_size").as_double();
    RCLCPP_INFO(this->get_logger(), "Using global map window size: %f m", window_size_);

    sor_mean_k_ = this->get_parameter("sor_mean_k").as_int();
    sor_std_dev_mult_ = this->get_parameter("sor_std_dev_mult").as_double();
    RCLCPP_INFO(this->get_logger(), "Using SOR filter mean k parameter: %d", sor_mean_k_);
    RCLCPP_INFO(this->get_logger(), "Using SOR filter std dev mult parameter: %f", sor_std_dev_mult_);

    if (sor_mean_k_ < 0 || sor_std_dev_mult_ < 0.0)
    {
      RCLCPP_ERROR(this->get_logger(), "Invalid SOR filter parameters! Exiting...");
      return;
    }

    fake_3d_lidar_scan_num_ = this->get_parameter("fake_3d_lidar_scan_num").as_int();
    fake_3d_lidar_overlap_scan_num_ = this->get_parameter("fake_3d_lidar_overlap_scan_num").as_int();
    RCLCPP_INFO(this->get_logger(), "Number of fake 3D LiDAR scans to accumulate: %lu", fake_3d_lidar_scan_num_);
    RCLCPP_INFO(this->get_logger(), "Number of fake 3D LiDAR scans to overlap: %lu", fake_3d_lidar_overlap_scan_num_);

    if (fake_3d_lidar_overlap_scan_num_ > fake_3d_lidar_scan_num_)
    {
      RCLCPP_ERROR(this->get_logger(), "Scan overlapping larger than point cloud save divider! Exiting...");
      return;
    }

    simulate_noise_ = this->get_parameter("simulate_noise").as_bool();

    // create directories
    if (!std::filesystem::exists(map_dir_))
      std::filesystem::create_directory(map_dir_);

    if (std::filesystem::exists(local_map_dir_))
      std::filesystem::remove_all(local_map_dir_);
    std::filesystem::create_directory(local_map_dir_);
    std::filesystem::create_directory(local_map_tf_dir_);

    auto qos = rclcpp::SensorDataQoS();

    // tf
    double lidar_roll = lidar_mount_roll_deg * M_PI / 180.0;
    double lidar_pitch = lidar_mount_pitch_deg * M_PI / 180.0;
    double lidar_yaw = lidar_mount_yaw_deg * M_PI / 180.0;
    tf2::Quaternion q_lidar;
    q_lidar.setRPY(lidar_roll, lidar_pitch, lidar_yaw);
    q_lidar.normalize();

    drone_lidar_tf_.header.frame_id = drone_link_;
    drone_lidar_tf_.child_frame_id = lidar_link_;
    drone_lidar_tf_.transform.translation.x = lidar_mount_offset_x;
    drone_lidar_tf_.transform.translation.y = lidar_mount_offset_y;
    drone_lidar_tf_.transform.translation.z = lidar_mount_offset_z;
    drone_lidar_tf_.transform.rotation.w = q_lidar.w();
    drone_lidar_tf_.transform.rotation.x = q_lidar.x();
    drone_lidar_tf_.transform.rotation.y = q_lidar.y();
    drone_lidar_tf_.transform.rotation.z = q_lidar.z();

    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface()
    );
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // publishers
    slice_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(slice_point_cloud_topic_, qos);
    sync_slice_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(sync_slice_point_cloud_topic_, qos);
    fake_3d_lidar_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(fake_3d_lidar_point_cloud_topic_, qos);
    global_map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(global_map_topic_, qos);

    // subscribers
    mf_slice_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());

    mf_slice_scan_tf2_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      mf_slice_scan_sub_, *tf_buffer_, drone_link_, 10,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      tf2::durationFromSec(mf_timeout)
    );
    mf_slice_scan_tf2_->setTolerance(rclcpp::Duration::from_seconds(timestamp_tolerance));
    mf_slice_scan_tf2_->registerCallback(&LidarMappervisualizer::sync_scan_callback, this);

    orientation_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      orientation_topic_,
      qos,
      std::bind(&LidarMappervisualizer::orientation_callback, this, std::placeholders::_1)
    );

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic_,
      qos,
      std::bind(&LidarMappervisualizer::odom_callback, this, std::placeholders::_1)
    );

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      qos,
      std::bind(&LidarMappervisualizer::scan_callback, this, std::placeholders::_1)
    );

    point_cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      sync_slice_point_cloud_topic_,
      qos,
      std::bind(&LidarMappervisualizer::point_cloud_callback, this, std::placeholders::_1)
    );

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
      octomap_topic_,
      qos,
      std::bind(&LidarMappervisualizer::octomap_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "LiDAR mapper visualizer node has been started!");
  }


  /**
   * @brief Destroy the LidarMapperVisualizer object
   * 
   */
  ~LidarMappervisualizer()
  {

  }


private:
  const std::string scan_topic_ = "/ldlidar_node/scan";
  const std::string orientation_topic_ = "/mavros/imu/data";
  const std::string odom_topic_ = "/mavros/local_position/odom";
  const std::string octomap_topic_ = "/octomap_full";

  const std::string slice_point_cloud_topic_ = "/lidar_mapper_visualizer/slice_point_cloud";
  const std::string sync_slice_point_cloud_topic_ = "/lidar_mapper_visualizer/sync_slice_point_cloud";
  const std::string fake_3d_lidar_point_cloud_topic_ = "/lidar_mapper_visualizer/fake_3d_lidar_point_cloud";
  const std::string global_map_topic_ = "/lidar_mapper_visualizer/global_map";

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  std::string world_link_ = "map";
  std::string drone_link_ = "base_link";
  std::string lidar_link_ = "ldlidar_link";

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slice_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_slice_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fake_3d_lidar_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_slice_scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> mf_slice_scan_tf2_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_sub_;
  rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;

  sensor_msgs::msg::Imu::ConstSharedPtr latest_orientation_msg_;
  nav_msgs::msg::Odometry::ConstSharedPtr latest_odom_msg_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_msg_;

  laser_geometry::LaserProjection laser_projector_;
  geometry_msgs::msg::TransformStamped drone_lidar_tf_;

  pcl::PointCloud<pcl::PointXYZI>::Ptr sync_pcl_slice_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr global_map_point_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr local_point_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr prev_overlapping_point_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  pcl::PointCloud<pcl::PointXYZI>::Ptr next_overlapping_point_cloud_ = pcl::make_shared<pcl::PointCloud<pcl::PointXYZI>>();

  double window_size_ = 20.0;

  int sor_mean_k_ = 50;
  double sor_std_dev_mult_ = 1.0;

  const char* home_ = std::getenv("HOME");
  const std::string home_dir_ = home_ ? std::string(home_) : std::string(".");
  const std::string map_dir_ = home_dir_ + "/ros2_ws/src/lidar_mapper_visualizer/maps/";
  const std::string global_map_path_ = map_dir_ + "/global_map.pcd";
  const std::string local_map_dir_ = map_dir_ + "local_map/";
  const std::string local_map_tf_dir_ = local_map_dir_ + "tf/";
  unsigned long int fake_3d_lidar_scan_num_ = 20;
  unsigned long int fake_3d_lidar_overlap_scan_num_ = 10;
  unsigned long int map_pub_cnt_ = 0;
  Eigen::Isometry3d prev_tf_ = Eigen::Isometry3d::Identity();

  bool simulate_noise_ = false;
  std::random_device rd_;
  std::mt19937 gen_{rd_()};
  std::uniform_real_distribution<double> unif_pos_{-0.01, 0.01};
  std::uniform_real_distribution<double> unif_orient_{-0.002, 0.002};
  double rand_pos_noise_[3] = {0.0, 0.0, 0.0};
  double rand_orient_noise_[4] = {0.0, 0.0, 0.0, 0.0};


  /**
   * @brief Callback for orientation data
   * 
   * @param orientation_msg Orientation message pointer
   */
  void orientation_callback(const sensor_msgs::msg::Imu::SharedPtr orientation_msg)
  {
    latest_orientation_msg_ = orientation_msg;

    // for debug purposes

    geometry_msgs::msg::TransformStamped orientation_tf;
    orientation_tf.header.stamp = orientation_msg->header.stamp;
    orientation_tf.header.frame_id = world_link_;
    orientation_tf.child_frame_id = "drone_test_link";
    orientation_tf.transform.translation.x = 0.0;
    orientation_tf.transform.translation.y = 0.0;
    orientation_tf.transform.translation.z = 0.0;
    orientation_tf.transform.rotation.w = orientation_msg->orientation.w;
    orientation_tf.transform.rotation.x = orientation_msg->orientation.x;
    orientation_tf.transform.rotation.y = orientation_msg->orientation.y;
    orientation_tf.transform.rotation.z = orientation_msg->orientation.z;

    tf_broadcaster_->sendTransform(orientation_tf);
  }


  /**
   * @brief Callback for odometry data
   * 
   * @param odom_msg Odometry message pointer
   */
  void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
  {
    latest_odom_msg_ = odom_msg;

    geometry_msgs::msg::TransformStamped world_drone_tf;
    world_drone_tf.header.stamp = odom_msg->header.stamp;
    world_drone_tf.header.frame_id = world_link_;
    world_drone_tf.child_frame_id = drone_link_;
    world_drone_tf.transform.translation.x = odom_msg->pose.pose.position.x;
    world_drone_tf.transform.translation.y = odom_msg->pose.pose.position.y;
    world_drone_tf.transform.translation.z = odom_msg->pose.pose.position.z;
    world_drone_tf.transform.rotation.w = odom_msg->pose.pose.orientation.w;
    world_drone_tf.transform.rotation.x = odom_msg->pose.pose.orientation.x;
    world_drone_tf.transform.rotation.y = odom_msg->pose.pose.orientation.y;
    world_drone_tf.transform.rotation.z = odom_msg->pose.pose.orientation.z;

    if (simulate_noise_)
    {
      // add noise for simulation
      for (int i = 0; i < 3; ++i)
      {
        rand_pos_noise_[i] += unif_pos_(gen_);
        rand_orient_noise_[i] += unif_orient_(gen_);
      }
      rand_orient_noise_[3] += unif_orient_(gen_);

      tf2::Quaternion q_drone = {
        world_drone_tf.transform.rotation.w + rand_orient_noise_[0],
        world_drone_tf.transform.rotation.x + rand_orient_noise_[1],
        world_drone_tf.transform.rotation.y + rand_orient_noise_[2],
        world_drone_tf.transform.rotation.z + rand_orient_noise_[3]
      };
      q_drone.normalize();

      world_drone_tf.transform.translation.x += rand_pos_noise_[0];
      world_drone_tf.transform.translation.y += rand_pos_noise_[1];
      world_drone_tf.transform.translation.z += rand_pos_noise_[2];
      world_drone_tf.transform.rotation.w = q_drone.w();
      world_drone_tf.transform.rotation.x = q_drone.x();
      world_drone_tf.transform.rotation.y = q_drone.y();
      world_drone_tf.transform.rotation.z = q_drone.z();
    }

    // update just timestamp (static broadcaster causes mf to trigger all the time)
    drone_lidar_tf_.header.stamp = odom_msg->header.stamp;

    tf_broadcaster_->sendTransform(world_drone_tf);
    tf_broadcaster_->sendTransform(drone_lidar_tf_);
  }


  /**
   * @brief Callback for raw scan data
   * 
   * @param scan_msg Scan message pointer
   */
  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    latest_scan_msg_ = scan_msg;

    sensor_msgs::msg::PointCloud2 slice_point_cloud_msg;
    laser_projector_.projectLaser(*scan_msg, slice_point_cloud_msg);
    slice_point_cloud_pub_->publish(slice_point_cloud_msg);
  }


  /**
   * @brief Callback for deskewed scan data
   * 
   * @param scan_msg Scan message pointer
   */
  void sync_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    rclcpp::Duration scan_duration = rclcpp::Duration::from_seconds(scan_msg->ranges.size() * scan_msg->time_increment);
    rclcpp::Time end_of_scan = rclcpp::Time(scan_msg->header.stamp) + scan_duration;
    if (!tf_buffer_->canTransform(drone_link_,
                                  scan_msg->header.frame_id,
                                  end_of_scan,
                                  rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF data to cover the entire scan duration...");
      return;
    }

    sensor_msgs::msg::PointCloud2 sync_slice_point_cloud_msg;
    laser_projector_.transformLaserScanToPointCloud(drone_link_, *scan_msg, sync_slice_point_cloud_msg, *tf_buffer_);

    // point cloud filtering
    sync_pcl_slice_->clear();
    pcl::fromROSMsg(sync_slice_point_cloud_msg, *sync_pcl_slice_);
    if (sync_pcl_slice_->empty())
    {
      RCLCPP_WARN(this->get_logger(), "PCL slice empty...");
      return;
    }

    pcl::StatisticalOutlierRemoval<pcl::PointXYZI> sor;
    sor.setInputCloud(sync_pcl_slice_);
    sor.setMeanK(sor_mean_k_);
    sor.setStddevMulThresh(sor_std_dev_mult_);
    sor.filter(*sync_pcl_slice_);

    sensor_msgs::msg::PointCloud2 filtered_sync_slice_point_cloud_msg;
    pcl::toROSMsg(*sync_pcl_slice_, filtered_sync_slice_point_cloud_msg);
    filtered_sync_slice_point_cloud_msg.header = sync_slice_point_cloud_msg.header;

    // sync_slice_point_cloud_pub_->publish(sync_slice_point_cloud_msg); // unfiltered
    sync_slice_point_cloud_pub_->publish(filtered_sync_slice_point_cloud_msg);
  }


  /**
   * @brief Callback for fake 3D LiDAR point clouds
   * 
   * @param sync_slice_point_cloud_msg Point cloud message pointer
   */
  void point_cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr sync_slice_point_cloud_msg)
  {
    pcl::PointCloud<pcl::PointXYZI> pcl_slice;
    pcl::fromROSMsg(*sync_slice_point_cloud_msg, pcl_slice);
    pcl_slice.header.frame_id = sync_slice_point_cloud_msg->header.frame_id;

    rclcpp::Time transform_time = sync_slice_point_cloud_msg->header.stamp;
    if (!tf_buffer_->canTransform(world_link_,
                                  pcl_slice.header.frame_id,
                                  transform_time,
                                  rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform not available yet, skipping this scan...");
      return;
    }

    // transform to world frame
    geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
      world_link_,
      pcl_slice.header.frame_id,
      sync_slice_point_cloud_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.0)
    );
    pcl_ros::transformPointCloud(pcl_slice, pcl_slice, tf);
    pcl_slice.header.frame_id = world_link_;

    // accumulate point clouds
    *local_point_cloud_ += pcl_slice;

    if (map_pub_cnt_ % fake_3d_lidar_scan_num_ >= fake_3d_lidar_scan_num_ - fake_3d_lidar_overlap_scan_num_)
    {
      *next_overlapping_point_cloud_ += pcl_slice;
    }

    if ((map_pub_cnt_ + 1) % fake_3d_lidar_scan_num_ == 0)
    {
      // local map
      *local_point_cloud_ += *prev_overlapping_point_cloud_;
      local_point_cloud_->header.frame_id = world_link_;

      transform_time = sync_slice_point_cloud_msg->header.stamp;
      if (!tf_buffer_->canTransform(drone_link_,
                                    local_point_cloud_->header.frame_id,
                                    transform_time,
                                    rclcpp::Duration::from_seconds(0.0)))
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform not available yet, skipping this scan...");
        return;
      }

      // transform back to drone frame
      tf = tf_buffer_->lookupTransform(
        drone_link_,
        local_point_cloud_->header.frame_id,
        sync_slice_point_cloud_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.0)
      );
      pcl_ros::transformPointCloud(*local_point_cloud_, *local_point_cloud_, tf);
      local_point_cloud_->header.frame_id = drone_link_;

      // publish fake 3D LiDAR data
      sensor_msgs::msg::PointCloud2 fake_3d_lidar_point_cloud_msg;
      pcl::toROSMsg(*local_point_cloud_, fake_3d_lidar_point_cloud_msg);
      fake_3d_lidar_point_cloud_msg.header.frame_id = local_point_cloud_->header.frame_id;
      fake_3d_lidar_point_cloud_msg.header.stamp = sync_slice_point_cloud_msg->header.stamp;
      fake_3d_lidar_point_cloud_pub_->publish(fake_3d_lidar_point_cloud_msg);

      // save point cloud to file
      std::string local_point_cloud_num = std::to_string((map_pub_cnt_ - 1) / fake_3d_lidar_scan_num_);
      pcl::io::savePCDFile(local_map_dir_ + "local_map_" + local_point_cloud_num + ".pcd",
                           *local_point_cloud_);

      local_point_cloud_->clear();
      prev_overlapping_point_cloud_->clear();
      *prev_overlapping_point_cloud_ = *next_overlapping_point_cloud_;
      next_overlapping_point_cloud_->clear();

      // local tf for ICP
      Eigen::Isometry3d curr_tf = tf2::transformToEigen(tf);
      Eigen::Isometry3d delta_tf = prev_tf_ * curr_tf.inverse();
      Eigen::Matrix4d delta_matrix = delta_tf.matrix();

      std::string file_name = local_map_tf_dir_ + "/tf_" + local_point_cloud_num + ".txt";
      std::ofstream file(file_name);
      if (file.is_open())
      {
        file << std::fixed << std::setprecision(std::numeric_limits<double>::max_digits10);
        file << delta_matrix << "\n";
        file.close();
      }
      else
      {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file for writing...");
      }

      prev_tf_ = curr_tf;
    }

    map_pub_cnt_++;
  }


  /**
   * @brief Callback for octomap data
   * 
   * @param octomap_msg Octomap message pointer
   */
  void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr octomap_msg)
  {
    if (!tf_buffer_->canTransform(world_link_,
                                  drone_link_,
                                  octomap_msg->header.stamp,
                                  rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform not available yet, skipping this octomap...");
      return;
    }

    geometry_msgs::msg::TransformStamped transformStamped;
    transformStamped = tf_buffer_->lookupTransform(
      world_link_,
      drone_link_,
      octomap_msg->header.stamp,
      rclcpp::Duration::from_seconds(0.0));

    double drone_x = transformStamped.transform.translation.x;
    double drone_y = transformStamped.transform.translation.y;
    double drone_z = transformStamped.transform.translation.z;

    octomap::AbstractOcTree* raw_tree = octomap_msgs::msgToMap(*octomap_msg);
    std::unique_ptr<octomap::AbstractOcTree> abstract_tree(raw_tree);
    if (!abstract_tree)
    {
      return;
    }

    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_tree.get());
    if (!octree)
    {
      return;
    }

    global_map_point_cloud_->clear();
    global_map_point_cloud_->header.frame_id = world_link_;

    octomap::point3d min_pt(drone_x - window_size_,
                            drone_y - window_size_,
                            drone_z - window_size_);
    octomap::point3d max_pt(drone_x + window_size_,
                            drone_y + window_size_,
                            drone_z + window_size_);

    for (auto it = octree->begin_leafs_bbx(min_pt, max_pt), end = octree->end_leafs_bbx(); it != end; ++it)
    {
      if (octree->isNodeOccupied(*it))
      {
        global_map_point_cloud_->push_back(pcl::PointXYZI(it.getX(), it.getY(), it.getZ(), it->getOccupancy()));
      }
    }

    sensor_msgs::msg::PointCloud2 global_map_point_cloud_msg;
    pcl::toROSMsg(*global_map_point_cloud_, global_map_point_cloud_msg);
    global_map_point_cloud_msg.header.frame_id = global_map_point_cloud_->header.frame_id;
    global_map_point_cloud_msg.header.stamp = octomap_msg->header.stamp;
    global_map_pub_->publish(global_map_point_cloud_msg);

    // save point cloud filez
    pcl::io::savePCDFile(global_map_path_, *global_map_point_cloud_);
  }
};


/**
 * @brief Main function
 * 
 * @param argc Argument count
 * @param argv Argument vector
 * 
 * @return Exit status
 */
int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting LiDAR mapper visualizer node...");

  rclcpp::spin(std::make_shared<LidarMappervisualizer>());

  rclcpp::shutdown();

  return 0;
}
