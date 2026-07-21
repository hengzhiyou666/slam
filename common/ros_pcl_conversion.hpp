#pragma once

#include <cstdint>
#include <vector>

#include <pcl/PCLHeader.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/PCLPointField.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/register_point_struct.h>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <std_msgs/msg/header.hpp>

// dog3 的精简 ROS 2 中没有 pcl_conversions/pcl_ros。这里提供本工程实际
// 使用到的 PointCloud2 与 PCL 点云互转功能，接口名称与官方包保持一致。
namespace pcl_conversions
{
inline void fromPCL(const pcl::PCLHeader & pcl_header, std_msgs::msg::Header & header)
{
  header.stamp.sec = static_cast<std::int32_t>(pcl_header.stamp / 1000000ULL);
  header.stamp.nanosec =
    static_cast<std::uint32_t>((pcl_header.stamp % 1000000ULL) * 1000ULL);
  header.frame_id = pcl_header.frame_id;
}

inline void toPCL(const std_msgs::msg::Header & header, pcl::PCLHeader & pcl_header)
{
  pcl_header.seq = 0;
  pcl_header.stamp = static_cast<std::uint64_t>(header.stamp.sec) * 1000000ULL +
    static_cast<std::uint64_t>(header.stamp.nanosec) / 1000ULL;
  pcl_header.frame_id = header.frame_id;
}

inline void fromPCL(
  const pcl::PCLPointField & pcl_field, sensor_msgs::msg::PointField & field)
{
  field.name = pcl_field.name;
  field.offset = pcl_field.offset;
  field.datatype = pcl_field.datatype;
  field.count = pcl_field.count;
}

inline void fromPCL(
  const std::vector<pcl::PCLPointField> & pcl_fields,
  std::vector<sensor_msgs::msg::PointField> & fields)
{
  fields.resize(pcl_fields.size());
  for (std::size_t index = 0; index < pcl_fields.size(); ++index) {
    fromPCL(pcl_fields[index], fields[index]);
  }
}

inline void toPCL(
  const sensor_msgs::msg::PointField & field, pcl::PCLPointField & pcl_field)
{
  pcl_field.name = field.name;
  pcl_field.offset = field.offset;
  pcl_field.datatype = field.datatype;
  pcl_field.count = field.count;
}

inline void toPCL(
  const std::vector<sensor_msgs::msg::PointField> & fields,
  std::vector<pcl::PCLPointField> & pcl_fields)
{
  pcl_fields.resize(fields.size());
  for (std::size_t index = 0; index < fields.size(); ++index) {
    toPCL(fields[index], pcl_fields[index]);
  }
}

inline void fromPCL(
  const pcl::PCLPointCloud2 & pcl_cloud, sensor_msgs::msg::PointCloud2 & cloud)
{
  fromPCL(pcl_cloud.header, cloud.header);
  cloud.height = pcl_cloud.height;
  cloud.width = pcl_cloud.width;
  fromPCL(pcl_cloud.fields, cloud.fields);
  cloud.is_bigendian = pcl_cloud.is_bigendian;
  cloud.point_step = pcl_cloud.point_step;
  cloud.row_step = pcl_cloud.row_step;
  cloud.data = pcl_cloud.data;
  cloud.is_dense = pcl_cloud.is_dense;
}

inline void moveFromPCL(
  pcl::PCLPointCloud2 & pcl_cloud, sensor_msgs::msg::PointCloud2 & cloud)
{
  fromPCL(pcl_cloud.header, cloud.header);
  cloud.height = pcl_cloud.height;
  cloud.width = pcl_cloud.width;
  fromPCL(pcl_cloud.fields, cloud.fields);
  cloud.is_bigendian = pcl_cloud.is_bigendian;
  cloud.point_step = pcl_cloud.point_step;
  cloud.row_step = pcl_cloud.row_step;
  cloud.data.swap(pcl_cloud.data);
  cloud.is_dense = pcl_cloud.is_dense;
}

inline void toPCL(
  const sensor_msgs::msg::PointCloud2 & cloud, pcl::PCLPointCloud2 & pcl_cloud)
{
  toPCL(cloud.header, pcl_cloud.header);
  pcl_cloud.height = cloud.height;
  pcl_cloud.width = cloud.width;
  toPCL(cloud.fields, pcl_cloud.fields);
  pcl_cloud.is_bigendian = cloud.is_bigendian;
  pcl_cloud.point_step = cloud.point_step;
  pcl_cloud.row_step = cloud.row_step;
  pcl_cloud.data = cloud.data;
  pcl_cloud.is_dense = cloud.is_dense;
}
}  // namespace pcl_conversions

namespace pcl
{
template<typename PointT>
void toROSMsg(
  const pcl::PointCloud<PointT> & pcl_cloud, sensor_msgs::msg::PointCloud2 & cloud)
{
  pcl::PCLPointCloud2 pcl_cloud2;
  pcl::toPCLPointCloud2(pcl_cloud, pcl_cloud2);
  pcl_conversions::moveFromPCL(pcl_cloud2, cloud);
}

template<typename PointT>
void fromROSMsg(
  const sensor_msgs::msg::PointCloud2 & cloud, pcl::PointCloud<PointT> & pcl_cloud)
{
  pcl::PCLPointCloud2 pcl_cloud2;
  pcl_conversions::toPCL(cloud, pcl_cloud2);
  pcl::fromPCLPointCloud2(pcl_cloud2, pcl_cloud);
}
}  // namespace pcl
