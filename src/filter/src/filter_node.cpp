#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.hpp>
#include <vector>

class PointCloudProcessor : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    // 存储车体边界框参数
    double min_x_, max_x_, min_y_, max_y_, min_z_, max_z_;

    void process_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg);

public:
    explicit PointCloudProcessor(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
};

PointCloudProcessor::PointCloudProcessor(const rclcpp::NodeOptions & options)
: Node("pointcloud_processor", options) {
    // 1. 声明并获取动态参数
    this->declare_parameter("min_x", -0.35);
    this->declare_parameter("max_x", 0.35);
    this->declare_parameter("min_y", -0.25);
    this->declare_parameter("max_y", 0.25);
    this->declare_parameter("min_z", -0.15);
    this->declare_parameter("max_z", 0.50);

    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();

    // 2. 初始化 TF 监听器与发布/订阅
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::process_pointcloud, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_filtered", rclcpp::SensorDataQoS()
    );

    // RCLCPP_INFO(this->get_logger(), "点云滤波节点启动. 拦截框: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
    //             min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
}

void PointCloudProcessor::process_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    sensor_msgs::msg::PointCloud2 cloud_base;
    try {
        auto transform = tf_buffer_->lookupTransform("base_link", msg->header.frame_id, tf2::TimePointZero);
        tf2::doTransform(*msg, cloud_base, transform);
    } catch (const tf2::TransformException & ex) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "等待 TF 转换: %s", ex.what());
        return;
    }

    sensor_msgs::msg::PointCloud2 filtered_cloud;
    filtered_cloud.header = cloud_base.header;
    filtered_cloud.fields = cloud_base.fields;

    size_t point_step = cloud_base.point_step;
    size_t data_size = cloud_base.data.size();
    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(data_size);

    for (size_t i = 0; i < data_size; i += point_step) {
        float x = *reinterpret_cast<const float*>(cloud_base.data.data() + i);
        float y = *reinterpret_cast<const float*>(cloud_base.data.data() + i + 4);
        float z = *reinterpret_cast<const float*>(cloud_base.data.data() + i + 8);

        if (x >= min_x_ && x <= max_x_ && y >= min_y_ && y <= max_y_ && z >= min_z_ && z <= max_z_) {
            continue; 
        }    
        filtered_data.insert(filtered_data.end(), cloud_base.data.begin() + i, cloud_base.data.begin() + i + point_step);
    }

    filtered_cloud.width = filtered_data.size() / point_step;
    filtered_cloud.height = 1;
    filtered_cloud.row_step = filtered_cloud.width * point_step;
    filtered_cloud.point_step = point_step;
    filtered_cloud.is_bigendian = cloud_base.is_bigendian;
    filtered_cloud.is_dense = cloud_base.is_dense;
    filtered_cloud.data = std::move(filtered_data);

    if (filtered_cloud.width > 0) {
        publisher_->publish(filtered_cloud);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudProcessor)