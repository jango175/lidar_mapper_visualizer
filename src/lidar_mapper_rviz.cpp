#include <memory>
#include <string>
#include <rclcpp/node.hpp>
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
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>


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

    double timestamp_diff_threshold = this->get_parameter("timestamp_diff_threshold").as_double();
    RCLCPP_INFO(this->get_logger(), "Using timestamp difference threshold: %f seconds", timestamp_diff_threshold);

    auto qos = rclcpp::SensorDataQoS();

    // tf
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
      this->get_node_base_interface(),
      this->get_node_timers_interface()
    );
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_buffer_->setCreateTimerInterface(timer_interface);
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

    // publishers
    sync_scan_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>(sync_scan_topic_, qos);
    point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(point_cloud_topic_, qos);
    sync_point_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(sync_point_cloud_topic_, qos);

    // subscribers
    mf_scan_sub_.subscribe(this, scan_topic_, qos.get_rmw_qos_profile());

    mf_tf2_ = std::make_shared<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>>(
      mf_scan_sub_, *tf_buffer_, world_link_, 10,
      this->get_node_logging_interface(),
      this->get_node_clock_interface(),
      tf2::durationFromSec(timestamp_diff_threshold)
    );
    mf_tf2_->registerCallback(&LidarMapperVisualiser::sync_scan_callback, this);

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


  ~LidarMapperVisualiser()
  {

  }


private:
  std::string scan_topic_ = "/ldlidar_node/scan";
  std::string orientation_topic_ = "/mavros/imu/data";
  std::string gps_topic_ = "/mavros/global_position/global";
  std::string pose_topic_ = "/mavros/local_position/pose";

  std::string sync_scan_topic_ = "/drone/sync_scan";
  std::string point_cloud_topic_ = "/drone/point_cloud";
  std::string sync_point_cloud_topic_ = "/drone/sync_point_cloud";

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::string world_link_ = "map";
  std::string drone_base_link_ = "drone_base_link";

  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr sync_scan_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr point_cloud_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr sync_point_cloud_pub_;

  message_filters::Subscriber<sensor_msgs::msg::LaserScan> mf_scan_sub_;
  std::shared_ptr<tf2_ros::MessageFilter<sensor_msgs::msg::LaserScan>> mf_tf2_;

  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr orientation_sub_;
  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr gps_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;

  laser_geometry::LaserProjection laser_projector_;

  sensor_msgs::msg::Imu::ConstSharedPtr latest_orientation_msg_;
  sensor_msgs::msg::Imu::ConstSharedPtr previous_orientation_msg_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr latest_gps_msg_;
  sensor_msgs::msg::NavSatFix::ConstSharedPtr previous_gps_msg_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr latest_scan_msg_;
  sensor_msgs::msg::LaserScan::ConstSharedPtr previous_scan_msg_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr latest_pose_msg_;
  geometry_msgs::msg::PoseStamped::ConstSharedPtr previous_pose_msg_;

  double interpolation_timestamp_threshold_ = 0.25;


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
    world_drone_tf.child_frame_id = drone_base_link_;
    world_drone_tf.transform.translation.x = pose_msg->pose.position.x;
    world_drone_tf.transform.translation.y = pose_msg->pose.position.y;
    world_drone_tf.transform.translation.z = pose_msg->pose.position.z;
    world_drone_tf.transform.rotation.w = pose_msg->pose.orientation.w;
    world_drone_tf.transform.rotation.x = pose_msg->pose.orientation.x;
    world_drone_tf.transform.rotation.y = pose_msg->pose.orientation.y;
    world_drone_tf.transform.rotation.z = pose_msg->pose.orientation.z;

    tf_broadcaster_->sendTransform(world_drone_tf);
  }


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

    sensor_msgs::msg::PointCloud2 point_cloud_msg;
    laser_projector_.projectLaser(*scan_msg, point_cloud_msg);
    point_cloud_pub_->publish(point_cloud_msg);
  }


  void sync_scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr scan_msg)
  {
    rclcpp::Duration scan_duration = rclcpp::Duration::from_seconds(scan_msg->ranges.size() * scan_msg->time_increment);
    rclcpp::Time end_of_scan = rclcpp::Time(scan_msg->header.stamp) + scan_duration;
    if (!tf_buffer_->canTransform(world_link_,
                                  scan_msg->header.frame_id,
                                  end_of_scan,
                                  rclcpp::Duration::from_seconds(interpolation_timestamp_threshold_)))
    {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "Waiting for TF data to cover the entire scan duration...");
      return;
    }

    sensor_msgs::msg::PointCloud2 sync_point_cloud_msg;
    laser_projector_.transformLaserScanToPointCloud(world_link_, *scan_msg, sync_point_cloud_msg, *tf_buffer_);

    sync_point_cloud_pub_->publish(sync_point_cloud_msg);
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
