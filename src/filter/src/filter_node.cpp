#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vector>

class PointCloudProcessor : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // 存储车体边界框参数 (注意：现在是相对于雷达/IMU中心的坐标)
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
    this->declare_parameter("min_z", -0.50); // 注意Z轴的调整
    this->declare_parameter("max_z", 0.10);

    min_x_ = this->get_parameter("min_x").as_double();
    max_x_ = this->get_parameter("max_x").as_double();
    min_y_ = this->get_parameter("min_y").as_double();
    max_y_ = this->get_parameter("max_y").as_double();
    min_z_ = this->get_parameter("min_z").as_double();
    max_z_ = this->get_parameter("max_z").as_double();

    // 2. 初始化发布/订阅 (去除了所有的 TF 监听器)
    // 【关键修改】：订阅 /cloud_registered_body
    subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/cloud_registered_body", rclcpp::SensorDataQoS(), std::bind(&PointCloudProcessor::process_pointcloud, this, std::placeholders::_1)
    );

    publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
        "/cloud_filtered", rclcpp::SensorDataQoS()
    );

    RCLCPP_INFO(this->get_logger(), "极致点云滤波节点启动. 拦截框: X[%.2f, %.2f], Y[%.2f, %.2f], Z[%.2f, %.2f]",
                min_x_, max_x_, min_y_, max_y_, min_z_, max_z_);
}

void PointCloudProcessor::process_pointcloud(sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // 直接复用原始消息的 header 和 fields
    sensor_msgs::msg::PointCloud2 filtered_cloud;
    filtered_cloud.header = msg->header;  // 此时的 frame_id 是车体系 (如 body 或 lidar)
    filtered_cloud.fields = msg->fields;

    size_t point_step = msg->point_step;
    size_t data_size = msg->data.size();
    std::vector<uint8_t> filtered_data;
    filtered_data.reserve(data_size);

    // 直接在原始的字节流上进行极速遍历
    for (size_t i = 0; i < data_size; i += point_step) {
        float x = *reinterpret_cast<const float*>(msg->data.data() + i);
        float y = *reinterpret_cast<const float*>(msg->data.data() + i + 4);
        float z = *reinterpret_cast<const float*>(msg->data.data() + i + 8);

        // 如果点落在车身范围内，直接丢弃 (continue)
        if (x >= min_x_ && x <= max_x_ && y >= min_y_ && y <= max_y_ && z >= min_z_ && z <= max_z_) {
            continue; 
        }
        
        // 保留有效点
        filtered_data.insert(filtered_data.end(), msg->data.begin() + i, msg->data.begin() + i + point_step);
    }

    filtered_cloud.width = filtered_data.size() / point_step;
    filtered_cloud.height = 1;
    filtered_cloud.row_step = filtered_cloud.width * point_step;
    filtered_cloud.point_step = point_step;
    filtered_cloud.is_bigendian = msg->is_bigendian;
    filtered_cloud.is_dense = msg->is_dense;
    filtered_cloud.data = std::move(filtered_data);

    if (filtered_cloud.width > 0) {
        publisher_->publish(filtered_cloud);
    }
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(PointCloudProcessor)