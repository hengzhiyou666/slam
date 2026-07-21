#pragma once

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

// dog3 的私有依赖中没有完整的 libpcl_io 运行库。本文件只实现本工程
// 所需的 PCD ASCII/binary 读取，以及 PointXYZINormal binary 保存。
namespace omni_slam::pcd
{
struct Header
{
  std::vector<std::string> fields;
  std::vector<std::size_t> sizes;
  std::vector<char> types;
  std::vector<std::size_t> counts;
  std::size_t width = 0;
  std::size_t height = 1;
  std::size_t points = 0;
  std::string data;
};

inline std::string uppercase(std::string value)
{
  std::transform(
    value.begin(), value.end(), value.begin(),
    [](unsigned char character) {return static_cast<char>(std::toupper(character));});
  return value;
}

template<typename Value>
inline bool parse_values(std::istringstream & stream, std::vector<Value> & values)
{
  values.clear();
  Value value;
  while (stream >> value) {
    values.push_back(value);
  }
  return !values.empty();
}

inline bool read_header(std::istream & stream, Header & header, std::string * error)
{
  std::string line;
  while (std::getline(stream, line)) {
    if (line.empty() || line.front() == '#') {
      continue;
    }

    std::istringstream line_stream(line);
    std::string key;
    line_stream >> key;
    key = uppercase(key);
    if (key == "FIELDS" || key == "FIELD") {
      parse_values(line_stream, header.fields);
    } else if (key == "SIZE") {
      parse_values(line_stream, header.sizes);
    } else if (key == "TYPE") {
      std::vector<std::string> types;
      parse_values(line_stream, types);
      header.types.clear();
      for (const auto & type : types) {
        header.types.push_back(
          type.empty() ? '\0' : static_cast<char>(std::toupper(type.front())));
      }
    } else if (key == "COUNT") {
      parse_values(line_stream, header.counts);
    } else if (key == "WIDTH") {
      line_stream >> header.width;
    } else if (key == "HEIGHT") {
      line_stream >> header.height;
    } else if (key == "POINTS") {
      line_stream >> header.points;
    } else if (key == "DATA") {
      line_stream >> header.data;
      std::transform(
        header.data.begin(), header.data.end(), header.data.begin(),
        [](unsigned char character) {return static_cast<char>(std::tolower(character));});
      break;
    }
  }

  if (header.fields.empty() || header.sizes.size() != header.fields.size() ||
    header.types.size() != header.fields.size() || header.data.empty())
  {
    if (error) {
      *error = "invalid or incomplete PCD header";
    }
    return false;
  }
  if (header.counts.empty()) {
    header.counts.assign(header.fields.size(), 1);
  }
  if (header.counts.size() != header.fields.size()) {
    if (error) {
      *error = "PCD COUNT does not match FIELDS";
    }
    return false;
  }
  if (header.points == 0) {
    header.points = header.width * header.height;
  }
  if (header.width == 0) {
    header.width = header.points;
  }
  if (header.height == 0) {
    header.height = 1;
  }
  return header.points > 0;
}

inline double binary_number(const char * data, std::size_t size, char type)
{
  if (type == 'F' && size == 4) {
    float value;
    std::memcpy(&value, data, sizeof(value));
    return value;
  }
  if (type == 'F' && size == 8) {
    double value;
    std::memcpy(&value, data, sizeof(value));
    return value;
  }
  if (type == 'I') {
    if (size == 1) {
      return *reinterpret_cast<const std::int8_t *>(data);
    }
    if (size == 2) {
      std::int16_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    if (size == 4) {
      std::int32_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    if (size == 8) {
      std::int64_t value;
      std::memcpy(&value, data, sizeof(value));
      return static_cast<double>(value);
    }
  }
  if (type == 'U') {
    if (size == 1) {
      return *reinterpret_cast<const std::uint8_t *>(data);
    }
    if (size == 2) {
      std::uint16_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    if (size == 4) {
      std::uint32_t value;
      std::memcpy(&value, data, sizeof(value));
      return value;
    }
    if (size == 8) {
      std::uint64_t value;
      std::memcpy(&value, data, sizeof(value));
      return static_cast<double>(value);
    }
  }
  return 0.0;
}

template<typename Point, typename Assign>
inline bool load(
  const std::string & path, pcl::PointCloud<Point> & cloud, Assign assign,
  std::string * error = nullptr)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    if (error) {
      *error = "cannot open PCD file";
    }
    return false;
  }

  Header header;
  if (!read_header(stream, header, error)) {
    return false;
  }

  cloud.clear();
  cloud.points.resize(header.points);
  if (header.data == "ascii") {
    for (std::size_t point_index = 0; point_index < header.points; ++point_index) {
      for (std::size_t field_index = 0; field_index < header.fields.size(); ++field_index) {
        for (std::size_t count_index = 0;
          count_index < header.counts[field_index]; ++count_index)
        {
          double value;
          if (!(stream >> value)) {
            if (error) {
              *error = "unexpected end of ASCII PCD data";
            }
            return false;
          }
          if (count_index == 0) {
            assign(cloud.points[point_index], header.fields[field_index], value);
          }
        }
      }
    }
  } else if (header.data == "binary") {
    std::size_t point_step = 0;
    std::vector<std::size_t> offsets;
    offsets.reserve(header.fields.size());
    for (std::size_t field_index = 0; field_index < header.fields.size(); ++field_index) {
      offsets.push_back(point_step);
      point_step += header.sizes[field_index] * header.counts[field_index];
    }
    std::vector<char> buffer(point_step * header.points);
    if (!stream.read(buffer.data(), static_cast<std::streamsize>(buffer.size()))) {
      if (error) {
        *error = "unexpected end of binary PCD data";
      }
      return false;
    }
    for (std::size_t point_index = 0; point_index < header.points; ++point_index) {
      const char * point_data = buffer.data() + point_index * point_step;
      for (std::size_t field_index = 0; field_index < header.fields.size(); ++field_index) {
        assign(
          cloud.points[point_index], header.fields[field_index],
          binary_number(
            point_data + offsets[field_index], header.sizes[field_index],
            header.types[field_index]));
      }
    }
  } else {
    if (error) {
      *error = "binary_compressed PCD is not supported";
    }
    return false;
  }

  cloud.width = static_cast<std::uint32_t>(header.width);
  cloud.height = static_cast<std::uint32_t>(header.height);
  cloud.is_dense = false;
  return true;
}

inline bool load_xyz(
  const std::string & path, pcl::PointCloud<pcl::PointXYZ> & cloud,
  std::string * error = nullptr)
{
  return load(
    path, cloud,
    [](pcl::PointXYZ & point, const std::string & field, double value) {
      if (field == "x") {
        point.x = static_cast<float>(value);
      } else if (field == "y") {
        point.y = static_cast<float>(value);
      } else if (field == "z") {
        point.z = static_cast<float>(value);
      }
    }, error);
}

inline bool load_xyzinormal(
  const std::string & path, pcl::PointCloud<pcl::PointXYZINormal> & cloud,
  std::string * error = nullptr)
{
  return load(
    path, cloud,
    [](pcl::PointXYZINormal & point, const std::string & field, double value) {
      if (field == "x") {
        point.x = static_cast<float>(value);
      } else if (field == "y") {
        point.y = static_cast<float>(value);
      } else if (field == "z") {
        point.z = static_cast<float>(value);
      } else if (field == "intensity") {
        point.intensity = static_cast<float>(value);
      } else if (field == "normal_x") {
        point.normal_x = static_cast<float>(value);
      } else if (field == "normal_y") {
        point.normal_y = static_cast<float>(value);
      } else if (field == "normal_z") {
        point.normal_z = static_cast<float>(value);
      } else if (field == "curvature") {
        point.curvature = static_cast<float>(value);
      }
    }, error);
}

inline bool save_xyzinormal_binary(
  const std::string & path, const pcl::PointCloud<pcl::PointXYZINormal> & cloud,
  std::string * error = nullptr)
{
  std::ofstream stream(path, std::ios::binary | std::ios::trunc);
  if (!stream) {
    if (error) {
      *error = "cannot create PCD file";
    }
    return false;
  }

  stream << "# .PCD v0.7 - Point Cloud Data file format\n"
         << "VERSION 0.7\n"
         << "FIELDS x y z intensity normal_x normal_y normal_z curvature\n"
         << "SIZE 4 4 4 4 4 4 4 4\n"
         << "TYPE F F F F F F F F\n"
         << "COUNT 1 1 1 1 1 1 1 1\n"
         << "WIDTH " << cloud.size() << "\n"
         << "HEIGHT 1\n"
         << "VIEWPOINT 0 0 0 1 0 0 0\n"
         << "POINTS " << cloud.size() << "\n"
         << "DATA binary\n";
  for (const auto & point : cloud.points) {
    const float values[] = {
      point.x, point.y, point.z, point.intensity,
      point.normal_x, point.normal_y, point.normal_z, point.curvature};
    stream.write(reinterpret_cast<const char *>(values), sizeof(values));
  }
  if (!stream) {
    if (error) {
      *error = "failed while writing PCD data";
    }
    return false;
  }
  return true;
}
}  // namespace omni_slam::pcd
