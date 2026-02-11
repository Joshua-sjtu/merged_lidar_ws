#include <rclcpp/rclcpp.hpp>
#include <livox_ros_driver2/msg/custom_msg.hpp>
#include <livox_ros_driver2/msg/custom_point.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <cstdint>  // uint64_t, uint8_t
#include <vector>
#include <string>

using CustomMsg = livox_ros_driver2::msg::CustomMsg;
using CustomPoint = livox_ros_driver2::msg::CustomPoint;

using SyncPolicy = message_filters::sync_policies::ApproximateTime<CustomMsg, CustomMsg>;

class MergeCustomMsgNode : public rclcpp::Node {
public:
  MergeCustomMsgNode() : Node("merge_point_node"),
                         tf_buffer_(this->get_clock()),
                         tf_listener_(tf_buffer_) {
    // QoS 配置
    rclcpp::QoS qos = rclcpp::SensorDataQoS();

    rmw_qos_profile_t rmw_qos = qos.get_rmw_qos_profile();

    // subscriber 用 rmw_qos
    sub1_.subscribe(this, "/livox/lidar_192_168_1_187", rmw_qos);
    sub2_.subscribe(this, "/livox/lidar_192_168_1_198", rmw_qos);
    
    sync_ = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(SyncPolicy(10), sub1_, sub2_);
    sync_->setMaxIntervalDuration(rclcpp::Duration::from_seconds(0.03));
    sync_->registerCallback(std::bind(&MergeCustomMsgNode::callback, this, std::placeholders::_1, std::placeholders::_2));

    // publisher 直接用 qos（rclcpp::QoS 类型）
    // pub_merged_ = this->create_publisher<CustomMsg>("/merged_cloud", qos);
    pub_merged_ = this->create_publisher<CustomMsg>("/merged_cloud", 20); // 为了配合 fast_lio 的 qos 设置

    // ──────────────── 新增：过滤参数声明 ────────────────
    this->declare_parameter<bool>("enable_box_filter", true);
    this->declare_parameter<std::string>("filter_target_frame", "base_link");//这对吗？是叫这个名字吗？
    this->declare_parameter<double>("filter_min_x", -1.0);//单位：米
    this->declare_parameter<double>("filter_max_x",  2.0);
    this->declare_parameter<double>("filter_min_y", -1.0);
    this->declare_parameter<double>("filter_max_y",  1.0);
    this->declare_parameter<double>("filter_min_z", -0.5);
    this->declare_parameter<double>("filter_max_z",  1.5);
  }

private:
  void callback(const CustomMsg::ConstSharedPtr& msg1, const CustomMsg::ConstSharedPtr& msg2) {
    if (!msg1 || !msg2 || msg1->points.empty() || msg2->points.empty()) {
      return;
    }

    // 获取 TF (lidar2 -> lidar1)
    geometry_msgs::msg::TransformStamped tf_stamped;
    try {
      tf_stamped = tf_buffer_.lookupTransform(
        "livox_frame_192_168_1_187",           // target = lidar1 frame
        "livox_frame_192_168_1_198",           // source = lidar2 frame
        rclcpp::Time(0),
        rclcpp::Duration(1, 0));
    } catch (const tf2::TransformException& ex) {
      RCLCPP_WARN(this->get_logger(), "TF failed: %s", ex.what());
      return;
    }



    // 计算时间差 (ns)，统一到 较早时间基准
    uint64_t dt_ns = 0;
    uint8_t earlier_lidar_id;
    if (msg2->timebase > msg1->timebase) {
      earlier_lidar_id = 1;
      dt_ns = msg2->timebase - msg1->timebase;
    } else {
      earlier_lidar_id = 2;
      dt_ns = msg1->timebase - msg2->timebase;
    }

    CustomMsg merged;
    merged.lidar_id = 0;  // 融合云设为 0

    // 处理 msg1 的点（不做坐标变换）
    for (const auto& pt1 : msg1->points) 
    {
      CustomPoint new_pt = pt1;
      new_pt.offset_time = static_cast<uint32_t>(new_pt.offset_time + (earlier_lidar_id == 1 ? 0 : dt_ns / 1000));
      merged.points.push_back(new_pt);
    }

    // 处理 msg2 的点（变换到 lidar1 坐标系）
    for (const auto& pt2 : msg2->points)
    {
      CustomPoint new_pt = pt2;

      // 变换坐标 (x,y,z)
      geometry_msgs::msg::PointStamped pt_in, pt_out;
      pt_in.header = msg2->header;
      pt_in.point.x = pt2.x; pt_in.point.y = pt2.y; pt_in.point.z = pt2.z;
      tf2::doTransform(pt_in, pt_out, tf_stamped);
      new_pt.x = static_cast<float>(pt_out.point.x);
      new_pt.y = static_cast<float>(pt_out.point.y);
      new_pt.z = static_cast<float>(pt_out.point.z);

      new_pt.offset_time = static_cast<uint32_t>(new_pt.offset_time + (earlier_lidar_id == 2 ? 0 : dt_ns / 1000));
      merged.points.push_back(new_pt);
    }

    // ──────────────── 新增：长方体过滤（在指定坐标系下） ────────────────
    bool enable_box_filter = this->get_parameter("enable_box_filter").as_bool();
    if (enable_box_filter) {
      std::string target_frame = this->get_parameter("filter_target_frame").as_string();

      double min_x = this->get_parameter("filter_min_x").as_double();
      double max_x = this->get_parameter("filter_max_x").as_double();
      double min_y = this->get_parameter("filter_min_y").as_double();
      double max_y = this->get_parameter("filter_max_y").as_double();
      double min_z = this->get_parameter("filter_min_z").as_double();
      double max_z = this->get_parameter("filter_max_z").as_double();

      // 获取从 lidar1 到目标坐标系的变换
      geometry_msgs::msg::TransformStamped tf_to_target;
      bool has_tf = true;
      try {
        tf_to_target = tf_buffer_.lookupTransform(
          target_frame,
          "livox_frame_192_168_1_187",
          rclcpp::Time(0),
          rclcpp::Duration(0.2, 0));
      } catch (const tf2::TransformException& ex) {
        RCLCPP_WARN(this->get_logger(), "Cannot get TF to %s: %s", target_frame.c_str(), ex.what());
        has_tf = false;
      }

      if (has_tf) {
        std::vector<CustomPoint> filtered_points;
        filtered_points.reserve(merged.points.size());

        size_t removed = 0;

        for (const auto& pt : merged.points) 
        {
          geometry_msgs::msg::PointStamped pin, pout;
          pin.header.frame_id = "livox_frame_192_168_1_187";
          pin.header.stamp = merged.header.stamp;
          pin.point.x = pt.x;
          pin.point.y = pt.y;
          pin.point.z = pt.z;

          bool keep = true;

          try {
            tf2::doTransform(pin, pout, tf_to_target);
            double px = pout.point.x;
            double py = pout.point.y;
            double pz = pout.point.z;

            if (px >= min_x && px <= max_x &&
                py >= min_y && py <= max_y &&
                pz >= min_z && pz <= max_z) {
              keep = false;
              removed++;
            }
          } catch (...) {
            // 变换失败 → 保留该点
            keep = true;
          }

          if (keep) {
            filtered_points.push_back(pt);
          }
        }

        if (removed > 0) {
          RCLCPP_INFO(this->get_logger(), "Removed %zu points in box (frame: %s)", removed, target_frame.c_str());
        }

        merged.points = std::move(filtered_points);
      }
    }

    // 更新数据头等信息
    merged.point_num = static_cast<uint32_t>(merged.points.size());
    merged.header.stamp = (earlier_lidar_id == 1) ? msg1->header.stamp : msg2->header.stamp;
    merged.header.frame_id = "livox_frame_192_168_1_187";  // 统一到 lidar1
    merged.timebase = earlier_lidar_id == 1 ? msg1->timebase : msg2->timebase;

    pub_merged_->publish(merged);

    RCLCPP_INFO(this->get_logger(), "Merged: %zu + %zu = %zu points", msg1->points.size(), msg2->points.size(), merged.points.size());
  }

  message_filters::Subscriber<CustomMsg> sub1_, sub2_;
  std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> sync_;
  rclcpp::Publisher<CustomMsg>::SharedPtr pub_merged_;

  tf2_ros::Buffer tf_buffer_;
  tf2_ros::TransformListener tf_listener_;
};

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MergeCustomMsgNode>());
  rclcpp::shutdown();
  return 0;
}