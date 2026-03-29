/**
 * @file lidar_mapper_rviz.cpp
 * @author jango175
 * @brief LIDAR mapper visualiser ROS2 node
 * 
 * @copyright Copyright (c) 2026
 * 
 */

#include "sensor_msgs/msg/point_cloud2.hpp"
#include <filesystem>
#include <rclcpp/logging.hpp>
#include <string>
#include <random>
#include <boost/qvm/quat.hpp>
#include <boost/qvm/quat_operations.hpp>
#include <rclcpp/node.hpp>
#include <rclcpp/executors.hpp>
#include <rclcpp/logger.hpp>
#include <laser_geometry/laser_geometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/nav_sat_fix.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_ros/message_filter.hpp>
#include <tf2_ros/create_timer_ros.hpp>
#include <tf2_ros/transform_broadcaster.hpp>
#include <tf2_ros/transform_listener.hpp>
#include <tf2_ros/buffer.hpp>
#include <pcl_ros/transforms.hpp>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>


// LidarMapperVisualiser node class
class LidarMapperVisualiser : public rclcpp::Node
{
public:
  /**
   * @brief Construct a new Lidar Mapper Visualiser object
   * 
   */
  LidarMapperVisualiser() : Node("lidar_mapper_visualiser")
  {
    // parameters
    auto lidar_mount_angle_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_mount_angle_param_desc.description = "LIDAR mount angle in degrees";
    this->declare_parameter("lidar_mount_angle_deg", 30.0, lidar_mount_angle_param_desc);

    auto mf_timeout_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    mf_timeout_param_desc.description = "Timeout for message filter tf buffer (seconds)";
    this->declare_parameter("mf_timeout", 0.25, mf_timeout_param_desc);

    auto timestamp_tolerance_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timestamp_tolerance_param_desc.description = "Timestamp tolerance for laser interpolation (seconds)";
    this->declare_parameter("timestamp_tolerance", 0.11, timestamp_tolerance_param_desc);

    auto fake_3d_lidar_scan_num_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    fake_3d_lidar_scan_num_param_desc.description = "Number of scans to accumulate for fake 3D LIDAR point cloud";
    this->declare_parameter("fake_3d_lidar_scan_num", 20, fake_3d_lidar_scan_num_param_desc);

    auto fake_3d_lidar_overlap_scan_num_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    fake_3d_lidar_overlap_scan_num_param_desc.description = "Number of overlapping scans for fake 3D LIDAR point cloud";
    this->declare_parameter("fake_3d_lidar_overlap_scan_num", 10, fake_3d_lidar_overlap_scan_num_param_desc);

    auto simulate_noise_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    simulate_noise_param_desc.description = "Set to true to add a noise to pose for simulation";
    this->declare_parameter("simulate_noise", false, simulate_noise_param_desc);

    double mf_timeout = this->get_parameter("mf_timeout").as_double();
    RCLCPP_INFO(this->get_logger(), "Using message filter timeout: %f seconds", mf_timeout);

    double timestamp_tolerance = this->get_parameter("timestamp_tolerance").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp tolerance: %f seconds", timestamp_tolerance);

    if (timestamp_tolerance >= mf_timeout)
    {
      RCLCPP_ERROR(this->get_logger(), "mf_timeout is smaller than timestamp_tolerance! Exiting...");
      return;
    }

    double lidar_mount_angle_deg = this->get_parameter("lidar_mount_angle_deg").as_double();

    fake_3d_lidar_scan_num_ = this->get_parameter("fake_3d_lidar_scan_num").as_int();
    fake_3d_lidar_overlap_scan_num_ = this->get_parameter("fake_3d_lidar_overlap_scan_num").as_int();
    if (fake_3d_lidar_overlap_scan_num_ > fake_3d_lidar_scan_num_)
    {
      RCLCPP_ERROR(this->get_logger(), "Scan overlapping larger than point cloud save divider! Exiting...");
      return;
    }

    simulate_noise_ = this->get_parameter("simulate_noise").as_bool();

    global_point_cloud_.header.frame_id = world_link_;

    // create directories
    if (!std::filesystem::exists(map_dir_))
      std::filesystem::create_directory(map_dir_);

    if (std::filesystem::exists(local_map_dir_))
      std::filesystem::remove_all(local_map_dir_);
    std::filesystem::create_directory(local_map_dir_);

    auto qos = rclcpp::SensorDataQoS();

    // tf
    boost::qvm::quat<double> q_x = boost::qvm::rotx_quat(0.0);
    boost::qvm::quat<double> q_y = boost::qvm::roty_quat(lidar_mount_angle_deg * M_PI / 180.0);
    boost::qvm::quat<double> q_z = boost::qvm::rotz_quat(0.0);
    boost::qvm::quat<double> q_lidar = q_z * q_y * q_x;
    boost::qvm::normalize(q_lidar);

    drone_lidar_tf_.header.frame_id = drone_link_;
    drone_lidar_tf_.child_frame_id = lidar_link_;
    drone_lidar_tf_.transform.translation.x = lidar_offset_x_;
    drone_lidar_tf_.transform.translation.y = lidar_offset_y_;
    drone_lidar_tf_.transform.translation.z = lidar_offset_z_;
    drone_lidar_tf_.transform.rotation.w = q_lidar.a[0];
    drone_lidar_tf_.transform.rotation.x = q_lidar.a[1];
    drone_lidar_tf_.transform.rotation.y = q_lidar.a[2];
    drone_lidar_tf_.transform.rotation.z = q_lidar.a[3];

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
    mf_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());

    mf_tf2_slice_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      mf_scan_sub_, *tf_buffer_, drone_link_, 10,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      tf2::durationFromSec(mf_timeout)
    );
    mf_tf2_slice_->setTolerance(rclcpp::Duration::from_seconds(timestamp_tolerance));
    mf_tf2_slice_->registerCallback(&LidarMapperVisualiser::sync_scan_callback, this);

    mf_tf2_global_map_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      mf_scan_sub_, *tf_buffer_, world_link_, 10,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      tf2::durationFromSec(mf_timeout)
    );
    mf_tf2_global_map_->setTolerance(rclcpp::Duration::from_seconds(timestamp_tolerance));
    mf_tf2_global_map_->registerCallback(&LidarMapperVisualiser::global_map_callback, this);

    orientation_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
      orientation_topic_,
      qos,
      std::bind(&LidarMapperVisualiser::orientation_callback, this, std::placeholders::_1)
    );

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_,
      qos,
      std::bind(&LidarMapperVisualiser::gps_callback, this, std::placeholders::_1)
    );

    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic_,
      qos,
      std::bind(&LidarMapperVisualiser::scan_callback, this, std::placeholders::_1)
    );

    pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      pose_topic_,
      qos,
      std::bind(&LidarMapperVisualiser::pose_callback, this, std::placeholders::_1)
    );

    RCLCPP_INFO(this->get_logger(), "LIDAR mapper visualiser node has been started!");
  }


  /**
   * @brief Destroy the Lidar Mapper Visualiser object
   * 
   */
  ~LidarMapperVisualiser()
  {

  }


private:
  const std::string scan_topic_ = "/ldlidar_node/scan";
  const std::string orientation_topic_ = "/mavros/imu/data";
  const std::string gps_topic_ = "/mavros/global_position/global";
  const std::string pose_topic_ = "/mavros/local_position/pose";

  const std::string slice_point_cloud_topic_ = "/drone/slice_point_cloud";
  const std::string sync_slice_point_cloud_topic_ = "/drone/sync_slice_point_cloud";
  const std::string fake_3d_lidar_point_cloud_topic_ = "/drone/fake_3d_lidar_point_cloud";
  const std::string global_map_topic_ = "/drone/global_map";

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  const std::string world_link_ = "map";
  const std::string drone_link_ = "base_link";
  const std::string lidar_link_ = "ldlidar_link";

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr slice_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_slice_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr fake_3d_lidar_point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr global_map_pub_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> mf_tf2_slice_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> mf_tf2_global_map_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  sensor_msgs::msg::Imu::ConstSharedPtr latest_orientation_msg_;
  sensor_msgs::msg::Imu::ConstSharedPtr previous_orientation_msg_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr latest_gps_msg_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr previous_gps_msg_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_msg_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr previous_scan_msg_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_pose_msg_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr previous_pose_msg_;

  laser_geometry::LaserProjection laser_projector_;
  geometry_msgs::msg::TransformStamped drone_lidar_tf_;
  const double lidar_offset_x_ = 0.088;
  const double lidar_offset_y_ = 0.0;
  const double lidar_offset_z_ = 0.088;

  pcl::PointCloud<pcl::PointXYZI> global_point_cloud_;
  pcl::PointCloud<pcl::PointXYZI> local_point_cloud_;
  pcl::PointCloud<pcl::PointXYZI> prev_overlapping_point_cloud_;
  pcl::PointCloud<pcl::PointXYZI> next_overlapping_point_cloud_;
  const char* home_ = std::getenv("HOME");
  const std::string home_dir_ = home_ ? std::string(home_) : std::string(".");
  const std::string map_dir_ = home_dir_ + "/ros2_ws/src/lidar_mapper_visualiser/maps/";
  const std::string global_map_path_ = map_dir_ + "global_map.pcd";
  const std::string local_map_dir_ = map_dir_ + "local_map/";
  unsigned long int fake_3d_lidar_scan_num_ = 20;
  unsigned long int fake_3d_lidar_overlap_scan_num_ = 10;
  unsigned long int map_pub_cnt_ = 0;

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
    if (latest_orientation_msg_ != nullptr)
    {
      rclcpp::Time prev_timestamp(latest_orientation_msg_->header.stamp);
      rclcpp::Time new_timestamp(orientation_msg->header.stamp);
      if (new_timestamp > prev_timestamp)
        previous_orientation_msg_ = latest_orientation_msg_;
    }

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
   * @brief Callback for GPS data
   * 
   * @param gps_msg GPS message pointer
   */
  void gps_callback(const sensor_msgs::msg::NavSatFix::ConstSharedPtr gps_msg)
  {
    if (latest_gps_msg_ != nullptr)
    {
      rclcpp::Time prev_timestamp(latest_gps_msg_->header.stamp);
      rclcpp::Time new_timestamp(gps_msg->header.stamp);
      if (new_timestamp > prev_timestamp)
        previous_gps_msg_ = latest_gps_msg_;
    }

    latest_gps_msg_ = gps_msg;
  }


  /**
   * @brief Callback for local position data
   * 
   * @param pose_msg Local position message pointer
   */
  void pose_callback(const geometry_msgs::msg::PoseStamped::ConstSharedPtr pose_msg)
  {
    if (latest_pose_msg_ != nullptr)
    {
      rclcpp::Time prev_timestamp(latest_pose_msg_->header.stamp);
      rclcpp::Time new_timestamp(pose_msg->header.stamp);
      if (new_timestamp > prev_timestamp)
        previous_pose_msg_ = latest_pose_msg_;
    }

    latest_pose_msg_ = pose_msg;

    geometry_msgs::msg::TransformStamped world_drone_tf;
    world_drone_tf.header.stamp = pose_msg->header.stamp;
    world_drone_tf.header.frame_id = world_link_;
    world_drone_tf.child_frame_id = drone_link_;
    world_drone_tf.transform.translation.x = pose_msg->pose.position.x;
    world_drone_tf.transform.translation.y = pose_msg->pose.position.y;
    world_drone_tf.transform.translation.z = pose_msg->pose.position.z;
    world_drone_tf.transform.rotation.w = pose_msg->pose.orientation.w;
    world_drone_tf.transform.rotation.x = pose_msg->pose.orientation.x;
    world_drone_tf.transform.rotation.y = pose_msg->pose.orientation.y;
    world_drone_tf.transform.rotation.z = pose_msg->pose.orientation.z;

    if (simulate_noise_)
    {
      // add noise for simulation
      for (int i = 0; i < 3; ++i)
      {
        rand_pos_noise_[i] += unif_pos_(gen_);
        rand_orient_noise_[i] += unif_orient_(gen_);
      }
      rand_orient_noise_[3] += unif_orient_(gen_);

      world_drone_tf.transform.translation.x += rand_pos_noise_[0];
      world_drone_tf.transform.translation.y += rand_pos_noise_[1];
      world_drone_tf.transform.translation.z += rand_pos_noise_[2];
      world_drone_tf.transform.rotation.w += rand_orient_noise_[0];
      world_drone_tf.transform.rotation.x += rand_orient_noise_[1];
      world_drone_tf.transform.rotation.y += rand_orient_noise_[2];
      world_drone_tf.transform.rotation.z += rand_orient_noise_[3];
    }

    // update just timestamp
    drone_lidar_tf_.header.stamp = pose_msg->header.stamp;

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
    if (latest_scan_msg_ != nullptr)
    {
      rclcpp::Time prev_timestamp(latest_scan_msg_->header.stamp);
      rclcpp::Time new_timestamp(scan_msg->header.stamp);
      if (new_timestamp > prev_timestamp)
        previous_scan_msg_ = latest_scan_msg_;
    }

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

    sync_slice_point_cloud_pub_->publish(sync_slice_point_cloud_msg);
  }


  /**
   * @brief Callback for global point clouds
   * 
   * @param scan_msg Scan message pointer
   */
  void global_map_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    rclcpp::Duration scan_duration = rclcpp::Duration::from_seconds(scan_msg->ranges.size() * scan_msg->time_increment);
    rclcpp::Time end_of_scan = rclcpp::Time(scan_msg->header.stamp) + scan_duration;
    if (!tf_buffer_->canTransform(world_link_,
                                  scan_msg->header.frame_id,
                                  end_of_scan,
                                  rclcpp::Duration::from_seconds(0.0)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF data to cover the entire scan duration...");
      return;
    }

    sensor_msgs::msg::PointCloud2 sync_slice_point_cloud_msg;
    laser_projector_.transformLaserScanToPointCloud(world_link_, *scan_msg, sync_slice_point_cloud_msg, *tf_buffer_);

    // accumulate point clouds
    pcl::PointCloud<pcl::PointXYZI> pcl_slice;
    pcl::fromROSMsg(sync_slice_point_cloud_msg, pcl_slice);
    global_point_cloud_ += pcl_slice;
    local_point_cloud_ += pcl_slice;

    if (map_pub_cnt_ % fake_3d_lidar_scan_num_ >= fake_3d_lidar_scan_num_ - fake_3d_lidar_overlap_scan_num_)
    {
      next_overlapping_point_cloud_ += pcl_slice;
    }

    if (map_pub_cnt_ > 0 && map_pub_cnt_ % fake_3d_lidar_scan_num_ == 0)
    {
      // local map
      local_point_cloud_ += prev_overlapping_point_cloud_;
      local_point_cloud_.header.frame_id = world_link_;

      rclcpp::Time transform_time = scan_msg->header.stamp;
      if (!tf_buffer_->canTransform(drone_link_,
                                    local_point_cloud_.header.frame_id,
                                    transform_time,
                                    rclcpp::Duration::from_seconds(0.0)))
      {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Transform not available yet, skipping this scan");
        return;
      }

      // transform back to drone frame
      geometry_msgs::msg::TransformStamped tf = tf_buffer_->lookupTransform(
        drone_link_,
        local_point_cloud_.header.frame_id,
        scan_msg->header.stamp,
        rclcpp::Duration::from_seconds(0.0)
      );
      pcl_ros::transformPointCloud(local_point_cloud_, local_point_cloud_, tf);
      local_point_cloud_.header.frame_id = drone_link_;

      // publish fake 3D LIDAR data
      sensor_msgs::msg::PointCloud2 fake_3d_lidar_point_cloud_msg;
      pcl::toROSMsg(local_point_cloud_, fake_3d_lidar_point_cloud_msg);
      fake_3d_lidar_point_cloud_msg.header.frame_id = local_point_cloud_.header.frame_id;
      fake_3d_lidar_point_cloud_msg.header.stamp = scan_msg->header.stamp;
      fake_3d_lidar_point_cloud_pub_->publish(fake_3d_lidar_point_cloud_msg);

      // save point cloud to file
      pcl::io::savePCDFileBinary(local_map_dir_ +
                                  "local_map_" +
                                  std::to_string((map_pub_cnt_ - 1) / fake_3d_lidar_scan_num_) +
                                  ".pcd", local_point_cloud_);

      local_point_cloud_.clear();
      prev_overlapping_point_cloud_.clear();
      prev_overlapping_point_cloud_ = next_overlapping_point_cloud_;
      next_overlapping_point_cloud_.clear();

      // global map
      sensor_msgs::msg::PointCloud2 sync_point_cloud_msg;
      pcl::toROSMsg(global_point_cloud_, sync_point_cloud_msg);
      sync_point_cloud_msg.header.frame_id = global_point_cloud_.header.frame_id;
      sync_point_cloud_msg.header.stamp = scan_msg->header.stamp;
      // global_map_pub_->publish(sync_point_cloud_msg);

      // save point cloud to file
      pcl::io::savePCDFileBinary(global_map_path_, global_point_cloud_);
    }

    map_pub_cnt_++;
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

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting LIDAR mapper visualiser node...");

  rclcpp::spin(std::make_shared<LidarMapperVisualiser>());

  rclcpp::shutdown();

  return 0;
}
