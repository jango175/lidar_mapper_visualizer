#include <iostream>
#include <string>
#include <ctime>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "geometry_msgs/msg/quaternion_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "tf2_ros/transform_broadcaster.h"


class LidarMapperVisualiser : public rclcpp::Node
{
public:
  LidarMapperVisualiser() : Node("lidar_mapper_visualiser")
  {
    auto timestamp_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    timestamp_param_desc.description = "Threshold for timestamp difference in approximate sync (seconds)";
    this->declare_parameter("timestamp_diff_threshold", 0.025, timestamp_param_desc);

    auto lidar_pitch_deg_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    lidar_pitch_deg_param_desc.description = "Pitch angle of the LIDAR relative to the drone body frame (degrees)";
    this->declare_parameter("lidar_pitch_deg", 30.0, lidar_pitch_deg_param_desc);

    auto play_bag_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    play_bag_param_desc.description = "Whether to play a rosbag for testing";
    this->declare_parameter("play_bag", false, play_bag_param_desc);

    // subscribers
    auto qos = rclcpp::SensorDataQoS();

    mf_scan_sub_.subscribe(this, "/ldlidar_node/scan", qos.get_rmw_qos_profile());
    mf_orientation_sub_.subscribe(this, "/msp/orientation", qos.get_rmw_qos_profile());
    mf_gps_sub_.subscribe(this, "/msp/gps", qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
      ApproximateSyncPolicy(10),
      mf_scan_sub_,
      mf_orientation_sub_,
      mf_gps_sub_
    );

    double timestamp_diff_threshold = this->get_parameter("timestamp_diff_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp difference threshold: %f seconds", timestamp_diff_threshold);

    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(timestamp_diff_threshold));
    sync_->registerCallback(std::bind(&LidarMapperVisualiser::approximate_sync_callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2,
                                      std::placeholders::_3));

    // tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/drone_pose", qos);

    RCLCPP_INFO(this->get_logger(), "LIDAR mapper visualiser node has been started!");
  }


  ~LidarMapperVisualiser()
  {

  }


private:
  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> mf_orientation_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> mf_gps_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    geometry_msgs::msg::QuaternionStamped,
    sensor_msgs::msg::NavSatFix> ApproximateSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string world_link_ = "map";
  std::string drone_base_link_ = "drone_base";
  std::string lidar_base_link_ = "ldlidar_base";
  std::string lidar_link_ = "ldlidar_link";

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;

  bool got_first_fix_ = false;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_alt_ = 0.0;


  void approximate_sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                                 const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& orientation_msg,
                                 const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg)
  {
    if (gps_msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
      RCLCPP_WARN(this->get_logger(), "No GPS fix available.");
      return;
    }

    if (got_first_fix_ == false)
    {
      origin_lat_ = gps_msg->latitude;
      origin_lon_ = gps_msg->longitude;
      got_first_fix_ = true;
      RCLCPP_INFO(this->get_logger(), "Set origin to lat: %f, lon: %f", origin_lat_, origin_lon_);
    }

    double utm_x = 0.0;
    double utm_y = 0.0;
    latlon_to_xy(gps_msg->latitude, gps_msg->longitude, utm_x, utm_y);
    double alt = gps_msg->altitude - origin_alt_;

    double lqx, lqy, lqz, lqw;
    double lidar_pitch_deg = this->get_parameter("lidar_pitch_deg").as_double();
    euler_deg_to_quaternion(0.0, lidar_pitch_deg, 0.0, lqx, lqy, lqz, lqw);

    auto timestamp = scan_msg->header.stamp; // To avoid issues with bag playback timing
    bool play_bag = this->get_parameter("play_bag").as_bool();

    if (play_bag == false)
      timestamp = this->now();

    geometry_msgs::msg::TransformStamped world_drone_tf;
    world_drone_tf.header.stamp = timestamp;
    world_drone_tf.header.frame_id = world_link_;
    world_drone_tf.child_frame_id = drone_base_link_;
    world_drone_tf.transform.translation.x = utm_x;
    world_drone_tf.transform.translation.y = utm_y;
    world_drone_tf.transform.translation.z = alt;
    world_drone_tf.transform.rotation = orientation_msg->quaternion;

    tf_broadcaster_->sendTransform(world_drone_tf);

    geometry_msgs::msg::TransformStamped drone_lidar_tf;
    drone_lidar_tf.header.stamp = timestamp;
    drone_lidar_tf.header.frame_id = drone_base_link_;
    drone_lidar_tf.child_frame_id = lidar_base_link_;
    drone_lidar_tf.transform.translation.x = 0.0;
    drone_lidar_tf.transform.translation.y = 0.0;
    drone_lidar_tf.transform.translation.z = 0.0;
    drone_lidar_tf.transform.rotation.x = lqx;
    drone_lidar_tf.transform.rotation.y = lqy;
    drone_lidar_tf.transform.rotation.z = lqz;
    drone_lidar_tf.transform.rotation.w = lqw;

    tf_broadcaster_->sendTransform(drone_lidar_tf);

    // LIDAR link tf (only when playing bag to avoid conflicts with real LIDAR TF)
    if (play_bag)
    {
      geometry_msgs::msg::TransformStamped lidar_lidar_tf;
      lidar_lidar_tf.header.stamp = timestamp;
      lidar_lidar_tf.header.frame_id = lidar_base_link_;
      lidar_lidar_tf.child_frame_id = lidar_link_;
      lidar_lidar_tf.transform.translation.x = 0.0;
      lidar_lidar_tf.transform.translation.y = 0.0;
      lidar_lidar_tf.transform.translation.z = 0.02745;
      lidar_lidar_tf.transform.rotation.x = 0.0;
      lidar_lidar_tf.transform.rotation.y = 0.0;
      lidar_lidar_tf.transform.rotation.z = 0.0;
      lidar_lidar_tf.transform.rotation.w = 1.0;

      tf_broadcaster_->sendTransform(lidar_lidar_tf);
    }

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = world_link_;
    pose_msg.pose.position.x = utm_x;
    pose_msg.pose.position.y = utm_y;
    pose_msg.pose.position.z = alt;
    pose_msg.pose.orientation = orientation_msg->quaternion;

    pose_pub_->publish(pose_msg);
  }


  void latlon_to_xy(double lat, double lon, double& x, double& y)
  {
    const double R = 6371000.0; // Earth radius in meters

    double lat_rad = lat * M_PI / 180.0;
    double lon_rad = lon * M_PI / 180.0;
    double origin_lat_rad = origin_lat_ * M_PI / 180.0;
    double origin_lon_rad = origin_lon_ * M_PI / 180.0;

    double dlat = lat_rad - origin_lat_rad;
    double dlon = lon_rad - origin_lon_rad;

    // Simple equirectangular projection (ENU)
    x = R * dlon * cos(origin_lat_rad); // East
    y = R * dlat; // North
  }


  void euler_deg_to_quaternion(double roll_deg, double pitch_deg, double yaw_deg,
                               double& qx, double& qy, double& qz, double& qw)
  {
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    double cy = cos(yaw * 0.5);
    double sy = sin(yaw * 0.5);
    double cr = cos(roll * 0.5);
    double sr = sin(roll * 0.5);
    double cp = cos(pitch * 0.5);
    double sp = sin(pitch * 0.5);

    qw = cy * cr * cp + sy * sr * sp;
    qx = cy * sr * cp - sy * cr * sp;
    qy = cy * cr * sp + sy * sr * cp;
    qz = sy * cr * cp - cy * sr * sp;
  }
};


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Starting LIDAR mapper visualiser node...");

  rclcpp::spin(std::make_shared<LidarMapperVisualiser>());

  rclcpp::shutdown();

  return 0;
}