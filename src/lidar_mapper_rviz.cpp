#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include <boost/qvm/quat.hpp>
#include <boost/qvm/quat_operations.hpp>

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

    auto use_ned_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    use_ned_param_desc.description = "Use NED frame for orientation data (true) or ENU frame (false)";
    this->declare_parameter("use_ned", true, use_ned_param_desc);

    use_ned_ = this->get_parameter("use_ned").as_bool();
    if (use_ned_)
      orientation_topic_ = "/msp/orientation_ned";
    else
      orientation_topic_ = "/msp/orientation_enu";

    // subscribers
    auto qos = rclcpp::SensorDataQoS();

    mf_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());
    mf_orientation_sub_.subscribe(this, orientation_topic_, qos.get_rmw_qos_profile());
    mf_gps_sub_.subscribe(this, gps_topic_, qos.get_rmw_qos_profile());

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

    orientation_sub_ = this->create_subscription<geometry_msgs::msg::QuaternionStamped>(
      orientation_topic_,
      qos,
      std::bind(&LidarMapperVisualiser::orientation_callback, this, std::placeholders::_1)
    );

    gps_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
      gps_topic_,
      qos,
      std::bind(&LidarMapperVisualiser::gps_callback, this, std::placeholders::_1)
    );

    // tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // publisher
    pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, qos);

    RCLCPP_INFO(this->get_logger(), "LIDAR mapper visualiser node has been started!");
  }


  ~LidarMapperVisualiser()
  {

  }


private:
  std::string scan_topic_ = "/ldlidar_node/scan";
  std::string orientation_topic_ = "/msp/orientation_ned";
  std::string gps_topic_ = "/msp/gps";

  bool use_ned_ = true;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  message_filters::Subscriber<geometry_msgs::msg::QuaternionStamped> mf_orientation_sub_;
  message_filters::Subscriber<sensor_msgs::msg::NavSatFix> mf_gps_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    geometry_msgs::msg::QuaternionStamped,
    sensor_msgs::msg::NavSatFix> ApproximateSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  rclcpp::Subscription<geometry_msgs::msg::QuaternionStamped>::SharedPtr orientation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  geometry_msgs::msg::QuaternionStamped::SharedPtr latest_orientation_msg_;
  geometry_msgs::msg::QuaternionStamped::SharedPtr previous_orientation_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr latest_gps_msg_;
  sensor_msgs::msg::NavSatFix::SharedPtr previous_gps_msg_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string world_link_ = "map";
  std::string drone_base_link_ = "drone_base";

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
  std::string pose_topic_ = "/drone_pose";

  bool got_first_fix_ = false;
  double origin_lat_ = 0.0;
  double origin_lon_ = 0.0;
  double origin_alt_ = 0.0;


  void orientation_callback(const geometry_msgs::msg::QuaternionStamped::SharedPtr orientation_msg)
  {
    if (latest_orientation_msg_ != nullptr)
      previous_orientation_msg_ = latest_orientation_msg_;

    latest_orientation_msg_ = orientation_msg;
  }


  void gps_callback(const sensor_msgs::msg::NavSatFix::SharedPtr gps_msg)
  {
    if (latest_gps_msg_ != nullptr)
      previous_gps_msg_ = latest_gps_msg_;

    latest_gps_msg_ = gps_msg;
  }


  void approximate_sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                                 const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& orientation_msg,
                                 const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg)
  {
    if (previous_orientation_msg_ == nullptr || previous_gps_msg_ == nullptr)
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for previous messages to be available for synchronization.");
      return;
    }

    if (gps_msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
        previous_gps_msg_->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
      RCLCPP_WARN(this->get_logger(), "No GPS fix available.");
      return;
    }

    if (got_first_fix_ == false)
    {
      origin_lat_ = previous_gps_msg_->latitude;
      origin_lon_ = previous_gps_msg_->longitude;
      origin_alt_ = previous_gps_msg_->altitude;
      got_first_fix_ = true;
      RCLCPP_INFO(this->get_logger(), "Set origin to lat: %f, lon: %f", origin_lat_, origin_lon_);
    }

    double coord_x = 0.0;
    double coord_y = 0.0;
    latlon_to_xy(gps_msg->latitude, gps_msg->longitude, coord_x, coord_y);
    double alt = gps_msg->altitude; // - origin_alt_;

    boost::qvm::quat<double> q = {
      orientation_msg->quaternion.w,
      orientation_msg->quaternion.x,
      orientation_msg->quaternion.y,
      orientation_msg->quaternion.z
    };

    // interpolate pose to scan timestamp
    q = interpolate_pose(scan_msg, orientation_msg, gps_msg);

    // drone data is in NED frame, but ROS2 uses ENU frame
    if (use_ned_)
    {
      alt = -alt;
      ned_to_enu(coord_x, coord_y, alt, q);
    }

    auto timestamp = scan_msg->header.stamp;

    geometry_msgs::msg::TransformStamped world_drone_tf;
    world_drone_tf.header.stamp = timestamp;
    world_drone_tf.header.frame_id = world_link_;
    world_drone_tf.child_frame_id = drone_base_link_;
    world_drone_tf.transform.translation.x = coord_x;
    world_drone_tf.transform.translation.y = coord_y;
    world_drone_tf.transform.translation.z = alt;
    world_drone_tf.transform.rotation.w = q.a[0];
    world_drone_tf.transform.rotation.x = q.a[1];
    world_drone_tf.transform.rotation.y = q.a[2];
    world_drone_tf.transform.rotation.z = q.a[3];

    tf_broadcaster_->sendTransform(world_drone_tf);

    geometry_msgs::msg::PoseStamped pose_msg;
    pose_msg.header.stamp = timestamp;
    pose_msg.header.frame_id = world_link_;
    pose_msg.pose.position.x = coord_x;
    pose_msg.pose.position.y = coord_y;
    pose_msg.pose.position.z = alt;
    pose_msg.pose.orientation.w = q.a[0];
    pose_msg.pose.orientation.x = q.a[1];
    pose_msg.pose.orientation.y = q.a[2];
    pose_msg.pose.orientation.z = q.a[3];

    pose_pub_->publish(pose_msg);
  }


  boost::qvm::quat<double> interpolate_pose(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                                            const geometry_msgs::msg::QuaternionStamped::ConstSharedPtr& orientation_msg,
                                            const sensor_msgs::msg::NavSatFix::ConstSharedPtr& gps_msg)
  {
    double scan_timestamp = static_cast<double>(scan_msg->header.stamp.sec) +
                            static_cast<double>(scan_msg->header.stamp.nanosec) * 1e-9;

    double prev_orientation_timestamp = static_cast<double>(previous_orientation_msg_->header.stamp.sec) +
                                        static_cast<double>(previous_orientation_msg_->header.stamp.nanosec) * 1e-9;
    double orientation_timestamp = static_cast<double>(orientation_msg->header.stamp.sec) +
                                   static_cast<double>(orientation_msg->header.stamp.nanosec) * 1e-9;

    double prev_gps_timestamp = static_cast<double>(previous_gps_msg_->header.stamp.sec) +
                                static_cast<double>(previous_gps_msg_->header.stamp.nanosec) * 1e-9;
    double gps_timestamp = static_cast<double>(gps_msg->header.stamp.sec) +
                           static_cast<double>(gps_msg->header.stamp.nanosec) * 1e-9;

    // linear interpolation for orientation
    boost::qvm::quat<double> q_prev = {
      previous_orientation_msg_->quaternion.w,
      previous_orientation_msg_->quaternion.x,
      previous_orientation_msg_->quaternion.y,
      previous_orientation_msg_->quaternion.z
    };

    boost::qvm::quat<double> q_curr = {
      orientation_msg->quaternion.w,
      orientation_msg->quaternion.x,
      orientation_msg->quaternion.y,
      orientation_msg->quaternion.z
    };

    double t = (scan_timestamp - prev_orientation_timestamp) /
               (orientation_timestamp - prev_orientation_timestamp);

    boost::qvm::quat<double> q_interp = boost::qvm::slerp180(q_prev, q_curr, t);

    return q_interp;
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

    // simple equirectangular projection
    if (use_ned_)
    {
      x = R * dlat; // North
      y = R * dlon * cos(origin_lat_rad); // East
    }
    else
    {
      x = R * dlon * cos(origin_lat_rad); // East
      y = R * dlat; // North
    }
  }


  boost::qvm::quat<double> euler_deg_to_quaternion(double roll_deg, double pitch_deg, double yaw_deg)
  {
    double roll = roll_deg * M_PI / 180.0;
    double pitch = pitch_deg * M_PI / 180.0;
    double yaw = yaw_deg * M_PI / 180.0;

    boost::qvm::quat<double> q_x = boost::qvm::rotx_quat(roll);
    boost::qvm::quat<double> q_y = boost::qvm::roty_quat(pitch);
    boost::qvm::quat<double> q_z = boost::qvm::rotz_quat(yaw);

    boost::qvm::quat<double> q = q_z * q_y * q_x;

    return q;
  }


  void ned_to_enu(double& x, double& y, double& z, boost::qvm::quat<double>& q)
  {
    double temp_x = x;
    x = y;      // East
    y = temp_x; // North
    z = -z;     // Up

    boost::qvm::quat<double> q_rot = euler_deg_to_quaternion(180.0, 0.0, 90.0);
    q = q_rot * q;
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
