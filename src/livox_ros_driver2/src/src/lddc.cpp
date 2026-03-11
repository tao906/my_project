//
// The MIT License (MIT)
//
// Copyright (c) 2022 Livox. All rights reserved.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in
// all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//

#include "lddc.h"
#include "comm/comm.h"
#include "comm/ldq.h"

#include <cstdint>
#include <inttypes.h>
#include <iomanip>
#include <iostream>
#include <math.h>
#include <stdint.h>

#include "include/ros_headers.h"

#include "driver_node.h"
#include "lds_lidar.h"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>

namespace livox_ros {

/** Lidar Data Distribute Control--------------------------------------------*/
Lddc::Lddc(int format, int multi_topic, int data_src, int output_type,
           double frq, std::string &frame_id)
    : transfer_format_(format), use_multi_topic_(multi_topic),
      data_src_(data_src), output_type_(output_type), publish_frq_(frq),
      frame_id_(frame_id) {
  publish_period_ns_ = kNsPerSecond / publish_frq_;
  lds_ = nullptr;
}

Lddc::~Lddc() {

  PrepareExit();

  std::cout << "lddc destory!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            << std::endl;
}

int Lddc::RegisterLds(Lds *lds) {
  if (lds_ == nullptr) {
    lds_ = lds;
    return 0;
  } else {
    return -1;
  }
}

void Lddc::DistributePointCloudData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributePointCloudData is RequestExit" << std::endl;
    return;
  }

  lds_->pcd_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarDataQueue *p_queue = &lidar->data;
    if ((kConnectStateSampling != lidar->connect_state) ||
        (p_queue == nullptr)) {
      continue;
    }
    PollingLidarPointCloudData(lidar_id, lidar);
  }
}

void Lddc::DistributeImuData(void) {
  if (!lds_) {
    std::cout << "lds is not registered" << std::endl;
    return;
  }
  if (lds_->IsRequestExit()) {
    std::cout << "DistributeImuData is RequestExit" << std::endl;
    return;
  }

  lds_->imu_semaphore_.Wait();
  for (uint32_t i = 0; i < lds_->lidar_count_; i++) {
    uint32_t lidar_id = i;
    LidarDevice *lidar = &lds_->lidars_[lidar_id];
    LidarImuDataQueue *p_queue = &lidar->imu_data;
    if ((kConnectStateSampling != lidar->connect_state) ||
        (p_queue == nullptr)) {
      continue;
    }
    PollingLidarImuData(lidar_id, lidar);
  }
}

void Lddc::PollingLidarPointCloudData(uint8_t index, LidarDevice *lidar) {
  LidarDataQueue *p_queue = &lidar->data;
  if (p_queue == nullptr || p_queue->storage_packet == nullptr) {
    return;
  }

  while (!lds_->IsRequestExit() && !QueueIsEmpty(p_queue)) {
    if (kPointCloud2Msg == transfer_format_) {
      PublishPointcloud2(p_queue, index);
    } else if (kLivoxCustomMsg == transfer_format_) {
      // PublishPointcloud2AndCustomMsg(p_queue, index);
      PublishCustomPointcloud(p_queue, index);
    } else if (kPclPxyziMsg == transfer_format_) {
      PublishPclMsg(p_queue, index);
    } else if (kAllMsg == transfer_format_) {
      PublishPointcloud2AndCustomMsg(p_queue, index);
    }
  }
}

void Lddc::PollingLidarImuData(uint8_t index, LidarDevice *lidar) {
  LidarImuDataQueue &p_queue = lidar->imu_data;
  while (!lds_->IsRequestExit() && !p_queue.Empty()) {
    PublishImuData(p_queue, index);
  }
}

void Lddc::PrepareExit(void) {
  if (lds_) {
    lds_->PrepareExit();
    lds_ = nullptr;
  }
}

void Lddc::PublishPointcloud2(LidarDataQueue *queue, uint8_t index) {
  while (!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud2 failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud2 cloud;
    uint64_t timestamp = 0;
    InitPointcloud2Msg(pkg, cloud, timestamp);
    PublishPointcloud2Data(index, timestamp, cloud);
  }
}

void Lddc::PublishCustomPointcloud(LidarDataQueue *queue, uint8_t index) {
  while (!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish custom point cloud failed, the pkg points is empty.\n");
      continue;
    }

    CustomMsg livox_msg;
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);
    PublishCustomPointData(livox_msg, index);
  }
}

void Lddc::PublishPointcloud2AndCustomMsg(LidarDataQueue *queue,
                                          uint8_t index) {
  // printf("12312312321\n");
  while (!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud2 cloud;
    uint64_t timestamp = 0;
    InitPointcloud2Msg(pkg, cloud, timestamp);
    PublishPointcloud2Data(index, timestamp, cloud);

    CustomMsg livox_msg;
    InitCustomMsg(livox_msg, pkg, index);
    FillPointsToCustomMsg(livox_msg, pkg);
    PublishCustomPointData(livox_msg, index);
  }
}

/* for pcl::pxyzi */
void Lddc::PublishPclMsg(LidarDataQueue *queue, uint8_t index) {
#ifdef BUILDING_ROS2
  static bool first_log = true;
  if (first_log) {
    std::cout
        << "error: message type 'pcl::PointCloud' is NOT supported in ROS2, "
        << "please modify the 'xfer_format' field in the launch file"
        << std::endl;
  }
  first_log = false;
  return;
#endif
  while (!QueueIsEmpty(queue)) {
    StoragePacket pkg;
    QueuePop(queue, &pkg);
    if (pkg.points.empty()) {
      printf("Publish point cloud failed, the pkg points is empty.\n");
      continue;
    }

    PointCloud cloud;
    uint64_t timestamp = 0;
    InitPclMsg(pkg, cloud, timestamp);
    FillPointsToPclMsg(pkg, cloud);
    PublishPclData(index, timestamp, cloud);
  }
  return;
}

void Lddc::InitPointcloud2MsgHeader(PointCloud2 &cloud) {
  cloud.header.frame_id.assign(frame_id_);
  cloud.height = 1;
  cloud.width = 0;
  cloud.fields.resize(7);
  cloud.fields[0].offset = 0;
  cloud.fields[0].name = "x";
  cloud.fields[0].count = 1;
  cloud.fields[0].datatype = PointField::FLOAT32;
  cloud.fields[1].offset = 4;
  cloud.fields[1].name = "y";
  cloud.fields[1].count = 1;
  cloud.fields[1].datatype = PointField::FLOAT32;
  cloud.fields[2].offset = 8;
  cloud.fields[2].name = "z";
  cloud.fields[2].count = 1;
  cloud.fields[2].datatype = PointField::FLOAT32;
  cloud.fields[3].offset = 12;
  cloud.fields[3].name = "intensity";
  cloud.fields[3].count = 1;
  cloud.fields[3].datatype = PointField::FLOAT32;
  cloud.fields[4].offset = 16;
  cloud.fields[4].name = "tag";
  cloud.fields[4].count = 1;
  cloud.fields[4].datatype = PointField::UINT8;
  cloud.fields[5].offset = 17;
  cloud.fields[5].name = "line";
  cloud.fields[5].count = 1;
  cloud.fields[5].datatype = PointField::UINT8;
  cloud.fields[6].offset = 18;
  cloud.fields[6].name = "timestamp";
  cloud.fields[6].count = 1;
  cloud.fields[6].datatype = PointField::FLOAT64;
  cloud.point_step = sizeof(LivoxPointXyzrtlt);
}

void Lddc::InitPointcloud2Msg(const StoragePacket &pkg, PointCloud2 &cloud,
                              uint64_t &timestamp) {
  InitPointcloud2MsgHeader(cloud);

  cloud.point_step = sizeof(LivoxPointXyzrtlt);

  cloud.width = pkg.points_num;
  cloud.row_step = cloud.width * cloud.point_step;

  cloud.is_bigendian = false;
  cloud.is_dense = true;

  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }

  cloud.header.stamp = rclcpp::Time(timestamp);

  std::vector<LivoxPointXyzrtlt> points;
  for (size_t i = 0; i < pkg.points_num; ++i) {
    LivoxPointXyzrtlt point;
    point.x = pkg.points[i].x;
    point.y = pkg.points[i].y;
    point.z = pkg.points[i].z;
    point.reflectivity = pkg.points[i].intensity;
    point.tag = pkg.points[i].tag;
    point.line = pkg.points[i].line;
    point.timestamp = static_cast<double>(pkg.points[i].offset_time);
    points.push_back(std::move(point));
  }
  cloud.data.resize(pkg.points_num * sizeof(LivoxPointXyzrtlt));
  memcpy(cloud.data.data(), points.data(),
         pkg.points_num * sizeof(LivoxPointXyzrtlt));
}

// void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp,
//                                   const PointCloud2 &cloud) {
//   Publisher<PointCloud2>::SharedPtr publisher_ptr;
//   if (kAllMsg == transfer_format_) {
//     publisher_ptr = std::dynamic_pointer_cast<Publisher<PointCloud2>>(
//         GetCurrentPublisher2(index));
//   } else {
//     publisher_ptr = std::dynamic_pointer_cast<Publisher<PointCloud2>>(
//         GetCurrentPublisher(index));
//   }

//   // 创建一个新的 PointCloud2 消息用于发布过滤后的点云
//   PointCloud2 filtered_cloud;
//   filtered_cloud.header = cloud.header;
//   filtered_cloud.height = cloud.height;
//   filtered_cloud.width = 0; // 初始化宽度为 0，稍后根据过滤后的点云数量更新
//   filtered_cloud.fields = cloud.fields;
//   filtered_cloud.is_bigendian = cloud.is_bigendian;
//   filtered_cloud.is_dense = cloud.is_dense;
//   filtered_cloud.point_step = cloud.point_step;

//   // 过滤点云
//   std::vector<uint8_t> filtered_data;
//   for (size_t i = 0; i < cloud.data.size(); i += cloud.point_step) {
//     float x = *reinterpret_cast<const float*>(cloud.data.data() + i);
//     float y = *reinterpret_cast<const float*>(cloud.data.data() + i + 4);
//     float z = *reinterpret_cast<const float*>(cloud.data.data() + i + 8);

//     // if (x >= -0.2 && x <= 0.6 && y >= -0.3 && y <= 0.3 && z >= -0.3 && z <= 0.3) {
//     if (x >= -1 && x <= 1 && y >= -1 && y <= 1 && z >= -1 && z <= 1) {
//       continue;
//     }

//     // 将过滤后的点云数据添加到 filtered_data
//     filtered_data.insert(filtered_data.end(), cloud.data.begin() + i,
//                          cloud.data.begin() + i + cloud.point_step);
//   }

//   // 更新过滤后的点云宽度和行步长
//   filtered_cloud.width = filtered_data.size() / cloud.point_step;
//   filtered_cloud.row_step = filtered_cloud.width * cloud.point_step;
//   filtered_cloud.data = filtered_data;

//   // 发布过滤后的点云
//   if (kOutputToRos == output_type_ && filtered_cloud.width > 0) {
//     publisher_ptr->publish(filtered_cloud);
//   }
// }

// void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp,
//                                   const PointCloud2 &cloud) {
//   Publisher<PointCloud2>::SharedPtr publisher_ptr;
//   if (kAllMsg == transfer_format_) {
//     publisher_ptr = std::dynamic_pointer_cast<Publisher<PointCloud2>>(
//         GetCurrentPublisher2(index));
//   } else {
//     publisher_ptr = std::dynamic_pointer_cast<Publisher<PointCloud2>>(
//         GetCurrentPublisher(index));
//   }

//   // 将 PointCloud2 消息转换为 PCL 的点云格式
//   pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   pcl::fromROSMsg(cloud, *pcl_cloud);

//   // 创建并配置 CropBox 过滤器
//   pcl::CropBox<pcl::PointXYZ> boxFilter;
//   Eigen::Vector4f min_pt(-0.36, -0.36, 0.0, 1.0);  // 最小点
//   Eigen::Vector4f max_pt(0.36, 0.36, 0.64, 1.0);    // 最大点
//   boxFilter.setMin(min_pt);
//   boxFilter.setMax(max_pt);

//   // 设置为反转行为，过滤掉区域内的点，保留区域外的点
//   boxFilter.setNegative(true);

//   boxFilter.setInputCloud(pcl_cloud);

//   // 执行过滤操作
//   pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZ>);
//   boxFilter.filter(*filtered_cloud);

//   // 将过滤后的 PCL 点云转换回 PointCloud2 消息
//   PointCloud2 filtered_cloud_msg;
//   pcl::toROSMsg(*filtered_cloud, filtered_cloud_msg);
//   filtered_cloud_msg.header = cloud.header;

//   // 发布过滤后的点云
//   if (kOutputToRos == output_type_ && filtered_cloud->size() > 0) {
//     publisher_ptr->publish(filtered_cloud_msg);
//   }else{}
// }


void Lddc::PublishPointcloud2Data(const uint8_t index, const uint64_t timestamp,
                                  const PointCloud2 &cloud) {
  Publisher<PointCloud2>::SharedPtr publisher_ptr;
  if (kAllMsg == transfer_format_) {
    publisher_ptr = std::dynamic_pointer_cast<Publisher<PointCloud2>>(
        GetCurrentPublisher2(index));
  } else {
    publisher_ptr = std::dynamic_pointer_cast<Publisher<PointCloud2>>(
        GetCurrentPublisher(index));
  }
  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(cloud);
  } else {
  }
}

void Lddc::InitCustomMsg(CustomMsg &livox_msg, const StoragePacket &pkg,
                         uint8_t index) {
  livox_msg.header.frame_id.assign(frame_id_);

  uint64_t timestamp = 0;
  if (!pkg.points.empty()) {
    timestamp = pkg.base_time;
  }
  livox_msg.timebase = timestamp;

  livox_msg.header.stamp = rclcpp::Time(timestamp);

  livox_msg.point_num = pkg.points_num;
  if (lds_->lidars_[index].lidar_type == kLivoxLidarType) {
    livox_msg.lidar_id = lds_->lidars_[index].handle;
  } else {
    printf("Init custom msg lidar id failed, the index:%u.\n", index);
    livox_msg.lidar_id = 0;
  }
}

void Lddc::FillPointsToCustomMsg(CustomMsg &livox_msg,
                                 const StoragePacket &pkg) {
  uint32_t points_num = pkg.points_num;
  const std::vector<PointXyzlt> &points = pkg.points;
  for (uint32_t i = 0; i < points_num; ++i) {
    CustomPoint point;
    point.x = points[i].x;
    point.y = points[i].y;
    point.z = points[i].z;
    point.reflectivity = points[i].intensity;
    point.tag = points[i].tag;
    point.line = points[i].line;
    point.offset_time =
        static_cast<uint32_t>(points[i].offset_time - pkg.base_time);

    livox_msg.points.push_back(std::move(point));
  }
}

void Lddc::PublishCustomPointData(const CustomMsg &livox_msg,
                                  const uint8_t index) {
  Publisher<CustomMsg>::SharedPtr publisher_ptr =
      std::dynamic_pointer_cast<Publisher<CustomMsg>>(
          GetCurrentPublisher(index));

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(livox_msg);
  } else {
  }
}

void Lddc::InitPclMsg(const StoragePacket &pkg, PointCloud &cloud,
                      uint64_t &timestamp) {
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" << std::endl;
  return;
}

void Lddc::FillPointsToPclMsg(const StoragePacket &pkg, PointCloud &pcl_msg) {
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" << std::endl;
  return;
}

void Lddc::PublishPclData(const uint8_t index, const uint64_t timestamp,
                          const PointCloud &cloud) {
  std::cout << "warning: pcl::PointCloud is not supported in ROS2, "
            << "please check code logic" << std::endl;
  return;
}

void Lddc::InitImuMsg(const ImuData &imu_data, ImuMsg &imu_msg,
                      uint64_t &timestamp) {
  imu_msg.header.frame_id = "livox_frame";

  timestamp = imu_data.time_stamp;
  imu_msg.header.stamp = rclcpp::Time(timestamp); // to ros time stamp

  imu_msg.angular_velocity.x = imu_data.gyro_x;
  imu_msg.angular_velocity.y = imu_data.gyro_y;
  imu_msg.angular_velocity.z = imu_data.gyro_z;
  imu_msg.linear_acceleration.x = imu_data.acc_x;
  imu_msg.linear_acceleration.y = imu_data.acc_y;
  imu_msg.linear_acceleration.z = imu_data.acc_z;
}

void Lddc::PublishImuData(LidarImuDataQueue &imu_data_queue,
                          const uint8_t index) {
  ImuData imu_data;
  if (!imu_data_queue.Pop(imu_data)) {
    // printf("Publish imu data failed, imu data queue pop failed.\n");
    return;
  }

  ImuMsg imu_msg;
  uint64_t timestamp;
  InitImuMsg(imu_data, imu_msg, timestamp);

  Publisher<ImuMsg>::SharedPtr publisher_ptr =
      std::dynamic_pointer_cast<Publisher<ImuMsg>>(
          GetCurrentImuPublisher(index));

  if (kOutputToRos == output_type_) {
    publisher_ptr->publish(imu_msg);
  } else {
  }
}

std::shared_ptr<rclcpp::PublisherBase>
Lddc::CreatePublisher(uint8_t msg_type, std::string &topic_name,
                      uint32_t queue_size) {
  if (kPointCloud2Msg == msg_type) {
    DRIVER_INFO(*cur_node_, "%s publish use PointCloud2 format",
                topic_name.c_str());
    return cur_node_->create_publisher<PointCloud2>(topic_name, queue_size);
  } else if (kLivoxCustomMsg == msg_type) {
    DRIVER_INFO(*cur_node_, "%s publish use livox custom format",
                topic_name.c_str());
    return cur_node_->create_publisher<CustomMsg>(topic_name, queue_size);
  } else if (kLivoxImuMsg == msg_type) {
    DRIVER_INFO(*cur_node_, "%s publish use imu format", topic_name.c_str());
    return cur_node_->create_publisher<ImuMsg>(topic_name, queue_size);
  } else {
    PublisherPtr null_publisher(nullptr);
    return null_publisher;
  }
}

std::shared_ptr<rclcpp::PublisherBase>
Lddc::GetCurrentPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));

      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_pub_[handle] =
          CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      if (kAllMsg == transfer_format_) {
        global_pub_ =
            cur_node_->create_publisher<CustomMsg>(topic_name, queue_size);
        global_pub_2_ = cur_node_->create_publisher<PointCloud2>(
            "livox/lidar/pointcloud", queue_size);
        DRIVER_INFO(*cur_node_, "%s publish use pointcloud2 and custom format",
                    topic_name.c_str());
      } else {
        global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
      }
    }
    return global_pub_;
  }
}

std::shared_ptr<rclcpp::PublisherBase>
Lddc::GetCurrentPublisher2(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));

      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/lidar_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_pub_[handle] =
          CreatePublisher(transfer_format_, topic_name, queue_size);
    }
    return private_pub_[handle];
  } else {
    if (!global_pub_) {
      std::string topic_name("livox/lidar");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      if (kAllMsg == transfer_format_) {
        global_pub_ =
            cur_node_->create_publisher<CustomMsg>(topic_name, queue_size);
        global_pub_2_ = cur_node_->create_publisher<PointCloud2>(
            "livox/lidar/pointcloud", queue_size);
        DRIVER_INFO(*cur_node_, "%s publish use pointcloud2 and custom format",
                    topic_name.c_str());
      } else {
        global_pub_ = CreatePublisher(transfer_format_, topic_name, queue_size);
      }
    }
    return global_pub_2_;
  }
}

std::shared_ptr<rclcpp::PublisherBase>
Lddc::GetCurrentImuPublisher(uint8_t handle) {
  uint32_t queue_size = kMinEthPacketQueueSize;
  if (use_multi_topic_) {
    if (!private_imu_pub_[handle]) {
      char name_str[48];
      memset(name_str, 0, sizeof(name_str));
      std::string ip_string = IpNumToString(lds_->lidars_[handle].handle);
      snprintf(name_str, sizeof(name_str), "livox/imu_%s",
               ReplacePeriodByUnderline(ip_string).c_str());
      std::string topic_name(name_str);
      queue_size = queue_size * 2; // queue size is 64 for only one lidar
      private_imu_pub_[handle] =
          CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return private_imu_pub_[handle];
  } else {
    if (!global_imu_pub_) {
      std::string topic_name("livox/imu");
      queue_size = queue_size * 8; // shared queue size is 256, for all lidars
      global_imu_pub_ = CreatePublisher(kLivoxImuMsg, topic_name, queue_size);
    }
    return global_imu_pub_;
  }
}

void Lddc::CreateBagFile(const std::string &file_name) {}

} // namespace livox_ros
