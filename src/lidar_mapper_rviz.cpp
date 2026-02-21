#include <iostream>
#include <string>
#include <ctime>
#include <chrono>
#include <boost/qvm/quat.hpp>
#include <boost/qvm/quat_operations.hpp>
#include <boost/qvm/vec.hpp>
#include <boost/qvm/vec_operations.hpp>
#include <boost/qvm/all.hpp>

#include "rclcpp/rclcpp.hpp"
#include "message_filters/subscriber.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "message_filters/time_synchronizer.h"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/imu.hpp"
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

    auto interpolation_param_desc = rcl_interfaces::msg::ParameterDescriptor{};
    interpolation_param_desc.description = "Threshold for interpolation timestamp difference (seconds)";
    this->declare_parameter("interpolation_timestamp_threshold", 0.25, interpolation_param_desc);

    interpolation_timestamp_threshold_ = this->get_parameter("interpolation_timestamp_threshold").as_double();

    // subscribers
    auto qos = rclcpp::SensorDataQoS();

    mf_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());
    mf_pose_sub_.subscribe(this, pose_topic_, qos.get_rmw_qos_profile());

    sync_ = std::make_shared<message_filters::Synchronizer<ApproximateSyncPolicy>>(
      ApproximateSyncPolicy(10),
      mf_scan_sub_,
      mf_pose_sub_
    );

    double timestamp_diff_threshold = this->get_parameter("timestamp_diff_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp difference threshold: %f seconds", timestamp_diff_threshold);

    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(timestamp_diff_threshold));
    sync_->registerCallback(std::bind(&LidarMapperVisualiser::approximate_sync_callback, this,
                                      std::placeholders::_1,
                                      std::placeholders::_2));

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

    // tf broadcaster
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // publishers
    sync_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>(sync_pose_topic_, qos);
    sync_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(sync_scan_topic_, qos);

    RCLCPP_INFO(this->get_logger(), "LIDAR mapper visualiser node has been started!");
  }


  ~LidarMapperVisualiser()
  {

  }


private:
  std::string scan_topic_ = "/ldlidar_node/scan";
  std::string orientation_topic_ = "/mavros/imu/data";
  std::string gps_topic_ = "/mavros/global_position/global";
  std::string pose_topic_ = "/mavros/local_position/pose";

  std::string sync_pose_topic_ = "/drone/sync_pose";
  std::string sync_scan_topic_ = "/drone/sync_scan";

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  message_filters::Subscriber<geometry_msgs::msg::PoseStamped> mf_pose_sub_;

  typedef message_filters::sync_policies::ApproximateTime<
    sensor_msgs::msg::LaserScan,
    geometry_msgs::msg::PoseStamped> ApproximateSyncPolicy;
  std::shared_ptr<message_filters::Synchronizer<ApproximateSyncPolicy>> sync_;

  sensor_msgs::msg::Imu::ConstSharedPtr latest_orientation_msg_;
  sensor_msgs::msg::Imu::ConstSharedPtr previous_orientation_msg_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr latest_gps_msg_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr previous_gps_msg_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_pose_msg_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr previous_pose_msg_;

  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string world_link_ = "map";
  std::string drone_base_link_ = "drone_base_link";

  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr sync_pose_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sync_scan_pub_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;

  double interpolation_timestamp_threshold_ = 0.25;

  bool got_first_fix_ = false;
  double origin_x_ = 0.0;
  double origin_y_ = 0.0;
  double origin_alt_ = 0.0;


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

    boost::qvm::quat<double> q = {
      orientation_msg->orientation.w,
      orientation_msg->orientation.x,
      orientation_msg->orientation.y,
      orientation_msg->orientation.z
    };

    geometry_msgs::msg::TransformStamped orientation_tf;
    orientation_tf.header.stamp = orientation_msg->header.stamp;
    orientation_tf.header.frame_id = world_link_;
    orientation_tf.child_frame_id = "drone_test_link";
    orientation_tf.transform.translation.x = 0.0;
    orientation_tf.transform.translation.y = 0.0;
    orientation_tf.transform.translation.z = 0.0;
    orientation_tf.transform.rotation.w = q.a[0];
    orientation_tf.transform.rotation.x = q.a[1];
    orientation_tf.transform.rotation.y = q.a[2];
    orientation_tf.transform.rotation.z = q.a[3];

    tf_broadcaster_->sendTransform(orientation_tf);
  }


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


  void approximate_sync_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                                 const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg)
  {
    if (latest_pose_msg_ != nullptr)
    {
      rclcpp::Time prev_timestamp(latest_pose_msg_->header.stamp);
      rclcpp::Time new_timestamp(pose_msg->header.stamp);
      if (new_timestamp > prev_timestamp)
        previous_pose_msg_ = latest_pose_msg_;
    }

    latest_pose_msg_ = pose_msg;

    if (previous_pose_msg_ == nullptr ||
        previous_gps_msg_ == nullptr ||
        latest_gps_msg_ == nullptr)
    {
      RCLCPP_WARN(this->get_logger(), "Waiting for previous messages to be available for synchronization.");
      return;
    }

    if (latest_gps_msg_->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX ||
        previous_gps_msg_->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
      RCLCPP_WARN(this->get_logger(), "No GPS fix available.");
      return;
    }

    if (got_first_fix_ == false)
    {
      origin_x_ = previous_pose_msg_->pose.position.x;
      origin_y_ = previous_pose_msg_->pose.position.y;
      // origin_alt_ = previous_pose_msg_->pose.position.z;
      got_first_fix_ = true;
      RCLCPP_INFO(this->get_logger(), "Set origin to:  %f, %f, %f", origin_x_, origin_y_, origin_alt_);
    }

    double coord_x = pose_msg->pose.position.x - origin_x_;
    double coord_y = pose_msg->pose.position.y - origin_y_;
    double alt = pose_msg->pose.position.z - origin_alt_;

    boost::qvm::quat<double> q = {
      pose_msg->pose.orientation.w,
      pose_msg->pose.orientation.x,
      pose_msg->pose.orientation.y,
      pose_msg->pose.orientation.z
    };

    // interpolate pose to scan timestamp
    interpolate_pose(scan_msg, pose_msg, coord_x, coord_y, alt, q);

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

    geometry_msgs::msg::PoseStamped sync_pose_msg;
    sync_pose_msg.header.stamp = timestamp;
    sync_pose_msg.header.frame_id = world_link_;
    sync_pose_msg.pose.position.x = coord_x;
    sync_pose_msg.pose.position.y = coord_y;
    sync_pose_msg.pose.position.z = alt;
    sync_pose_msg.pose.orientation.w = q.a[0];
    sync_pose_msg.pose.orientation.x = q.a[1];
    sync_pose_msg.pose.orientation.y = q.a[2];
    sync_pose_msg.pose.orientation.z = q.a[3];

    sync_pose_pub_->publish(sync_pose_msg);
    sync_scan_pub_->publish(*scan_msg);
  }


  void interpolate_pose(const sensor_msgs::msg::LaserScan::ConstSharedPtr& scan_msg,
                        const geometry_msgs::msg::PoseStamped::ConstSharedPtr& pose_msg,
                        double& coord_x, double& coord_y, double& alt,
                        boost::qvm::quat<double>& q)
  {
    double scan_timestamp = static_cast<double>(scan_msg->header.stamp.sec) +
                              static_cast<double>(scan_msg->header.stamp.nanosec) * 1e-9;

    double prev_timestamp = static_cast<double>(previous_pose_msg_->header.stamp.sec) +
                              static_cast<double>(previous_pose_msg_->header.stamp.nanosec) * 1e-9;
    double curr_timestamp = static_cast<double>(pose_msg->header.stamp.sec) +
                              static_cast<double>(pose_msg->header.stamp.nanosec) * 1e-9;

    // linear interpolation for orientation
    boost::qvm::quat<double> q_prev = {
      previous_pose_msg_->pose.orientation.w,
      previous_pose_msg_->pose.orientation.x,
      previous_pose_msg_->pose.orientation.y,
      previous_pose_msg_->pose.orientation.z
    };

    boost::qvm::quat<double> q_curr = {
      pose_msg->pose.orientation.w,
      pose_msg->pose.orientation.x,
      pose_msg->pose.orientation.y,
      pose_msg->pose.orientation.z
    };

    q = q_curr;
    coord_x = pose_msg->pose.position.x - origin_x_;
    coord_y = pose_msg->pose.position.y - origin_y_;
    alt = pose_msg->pose.position.z - origin_alt_;

    if (curr_timestamp > prev_timestamp &&
        curr_timestamp - prev_timestamp < interpolation_timestamp_threshold_)
    {
      double dt = (curr_timestamp - prev_timestamp);

      // q_curr = q_delta * q_prev
      boost::qvm::quat<double> q_delta = q_curr * boost::qvm::inverse(q_prev);

      // use the shortest path
      if (q_delta.a[0] < 0.0)
        q_delta = -q_delta;

      // get angle from q_delta
      double w = q_delta.a[0];
      boost::qvm::vec<double, 3> v_delta = boost::qvm::V(q_delta);
      double v_delta_norm = boost::qvm::mag(v_delta);
      double theta = 2.0 * std::atan2(v_delta_norm, w);

      // get angular velocity
      boost::qvm::vec<double, 3> omega;
      if (v_delta_norm < 1e-6)
      {
        // approximate for small angles
        omega = v_delta * (2.0 / dt);
      }
      else
      {
        omega = v_delta * (theta / (v_delta_norm * dt));
      }

      // interpolate angle
      double omega_norm = boost::qvm::mag(omega);
      double dt_interp = scan_timestamp - prev_timestamp;
      double theta_interp = omega_norm * dt_interp;

      // compute delta quaternion for interpolated angle
      boost::qvm::quat<double> q_delta_interp;
      if (omega_norm < 1e-6)
      {
        // no rotation
        q_delta_interp = boost::qvm::identity_quat<double>();
      }
      else
      {
        boost::qvm::vec<double, 3> axis = omega / omega_norm;
        q_delta_interp = boost::qvm::rot_quat(axis, theta_interp);
      }
      q = q_delta_interp * q_prev;
      boost::qvm::normalize(q);

      // linear interpolation for GPS
      double prev_coord_x = previous_pose_msg_->pose.position.x - origin_x_;
      double prev_coord_y = previous_pose_msg_->pose.position.y - origin_y_;
      double prev_alt = previous_pose_msg_->pose.position.z - origin_alt_;

      double t = (scan_timestamp - prev_timestamp) /
                 (curr_timestamp - prev_timestamp);

      coord_x = prev_coord_x + t * (coord_x - prev_coord_x);
      coord_y = prev_coord_y + t * (coord_y - prev_coord_y);
      alt = prev_alt + t * (alt - prev_alt);
    }
    else if (curr_timestamp == prev_timestamp)
    {
      RCLCPP_WARN(this->get_logger(), "Timestamps are identical, cannot interpolate.");
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Timestamps too far apart for interpolation.");
    }
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
