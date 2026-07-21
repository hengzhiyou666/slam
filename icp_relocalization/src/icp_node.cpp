#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include "ros_pcl_conversion.hpp"
#include "simple_pcd_io.hpp"
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/voxel_grid.h>
#include <Eigen/Geometry>
#include <cmath>
#include <stdexcept>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2/transform_datatypes.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#ifdef USE_LIVOX
#include <livox_ros_driver2/msg/custom_msg.hpp>
#endif

class ICPNode : public rclcpp::Node
{
public:
    ICPNode()
        : Node("icp_node")
    {
        this->declare_parameter("initial_x", 0.0);
        this->declare_parameter("initial_y", 0.0);
        this->declare_parameter("initial_z", 0.0);
        this->declare_parameter("initial_a", 0.0);
        this->declare_parameter("solver_max_iter", 75);
        this->declare_parameter("max_correspondence_distance", 0.1);
        this->declare_parameter("RANSAC_outlier_rejection_threshold", 1.0);
        this->declare_parameter("map_path", "");
        this->declare_parameter("map_frame_id", "map");
        this->declare_parameter("fitness_score_thre", 0.0);
        this->declare_parameter("map_voxel_leaf_size", 0.1);
        this->declare_parameter("cloud_voxel_leaf_size", 0.1);
        this->declare_parameter("converged_count_thre", 20);
        this->declare_parameter("pcl_type","livox");
        this->declare_parameter("livox_topic", "/livox/lidar");
        this->declare_parameter("pointcloud_topic", "/pointcloud2");

        this->get_parameter("initial_x", initial_x);
        this->get_parameter("initial_y", initial_y);
        this->get_parameter("initial_z", initial_z);
        this->get_parameter("initial_a", initial_a);
        this->get_parameter("solver_max_iter", solver_max_iter);
        this->get_parameter("max_correspondence_distance", max_correspondence_distance);
        this->get_parameter("RANSAC_outlier_rejection_threshold", RANSAC_outlier_rejection_threshold);
        this->get_parameter("map_path", map_path);
        this->get_parameter("map_frame_id", map_frame);
        this->get_parameter("fitness_score_thre", fitness_score_thre);
        this->get_parameter("map_voxel_leaf_size", map_voxel_leaf_size);
        this->get_parameter("cloud_voxel_leaf_size", cloud_voxel_leaf_size);
        this->get_parameter("converged_count_thre", converged_count_thre);
        this->get_parameter("pcl_type", pcl_type);
        this->get_parameter("livox_topic", livox_topic);
        this->get_parameter("pointcloud_topic", pointcloud_topic);

        publisher_ = this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("icp_result", 10);
#ifdef USE_LIVOX
        if(pcl_type == "livox")
        {
            lvx_cloud_sub_ = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                livox_topic, 10, std::bind(&ICPNode::lvx_cloud_callback, this, std::placeholders::_1));
        }
        else
        {
            cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
        }
#else
        cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_topic, 10, std::bind(&ICPNode::cloud_callback, this, std::placeholders::_1));
#endif
        pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "initialpose", 10, std::bind(&ICPNode::pose_callback, this, std::placeholders::_1));
        map_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("prior_map", 10);
        transformed_cloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("transformed_cloud", 10);
        
        // init guess
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = initial_x;
        initGuess(1, 3) = initial_y;
        initGuess(2, 3) = initial_z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        q.setRPY(0, 0, initial_a);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", initial_x, initial_y, initial_z, initial_a);
        // Load the target point cloud from a PCD file
        std::string pcd_error;
        if (!omni_slam::pcd::load_xyz(map_path, *target_cloud_, &pcd_error))
        {
            RCLCPP_FATAL(
                this->get_logger(), "Couldn't read map file %s: %s",
                map_path.c_str(), pcd_error.c_str());
            throw std::runtime_error("failed to load ICP map");
        }
        RCLCPP_INFO(this->get_logger(), "Loaded %d data points from target.pcd", target_cloud_->width * target_cloud_->height);

        // downsample the target cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_map;
        sor_map.setInputCloud(target_cloud_);
        sor_map.setLeafSize(map_voxel_leaf_size, map_voxel_leaf_size, map_voxel_leaf_size);
        sor_map.filter(*target_cloud_);
        RCLCPP_INFO(this->get_logger(), "Downsampled target cloud to %d data points", target_cloud_->width * target_cloud_->height);
        // Publish the downsampled target cloud

        pcl::toROSMsg(*target_cloud_, target_cloud_msg);
        target_cloud_msg.header.stamp = this->now();
        target_cloud_msg.header.frame_id = map_frame;
        map_pub_->publish(target_cloud_msg);
    }

private:
    void filter_input_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr & cloud)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>);
        filtered->reserve(cloud->size());

        constexpr float min_range_squared = 0.3f * 0.3f;
        constexpr float max_range_squared = 30.0f * 30.0f;
        for (const auto & point : cloud->points)
        {
            if (!std::isfinite(point.x) || !std::isfinite(point.y) || !std::isfinite(point.z))
            {
                continue;
            }
            const float range_squared =
                point.x * point.x + point.y * point.y + point.z * point.z;
            if (range_squared < min_range_squared || range_squared > max_range_squared)
            {
                continue;
            }
            filtered->push_back(point);
        }

        filtered->width = filtered->size();
        filtered->height = 1;
        filtered->is_dense = true;
        if (filtered->size() != cloud->size())
        {
            RCLCPP_DEBUG(
                this->get_logger(), "Filtered %zu invalid/out-of-range points from ICP input",
                cloud->size() - filtered->size());
        }
        cloud.swap(filtered);
    }

    void cloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *input_cloud);
        filter_input_cloud(input_cloud);

        if (input_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "ICP input cloud is empty after filtering");
            return;
        }

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        // Rotate pcl alone x axis for 180 degree
        // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        // rotation(1, 1) = -1;
        // rotation(2, 2) = -1;
        // pcl::transformPointCloud(*input_cloud, *input_cloud, rotation);

        // Perform ICP alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        // icp.setTransformationEpsilon(1e-8);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);
        // icp.setRANSACIterations(100);
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, initGuess);

        // Get fitness score
        double fitness_score = icp.getFitnessScore(max_correspondence_distance);
        RCLCPP_INFO(this->get_logger(), "ICP fitness score: %f", fitness_score);

        if (fitness_score < fitness_score_thre && icp.hasConverged())
        {
            converged_count++;
            const Eigen::Matrix4f transformation_result = icp.getFinalTransformation();
            const double yaw = std::atan2(
                transformation_result(1, 0), transformation_result(0, 0));
            RCLCPP_INFO(
                this->get_logger(),
                "Relocalization candidate [%d/%d]: X=%.3f Y=%.3f Z=%.3f YAW=%.3f rad (%.1f deg), fitness=%.6f",
                converged_count, converged_count_thre,
                transformation_result(0, 3), transformation_result(1, 3),
                transformation_result(2, 3), yaw, yaw * 180.0 / M_PI, fitness_score);
            if(converged_count < converged_count_thre)
            {
                RCLCPP_INFO(this->get_logger(), "ICP converged, but not enough count, no pose is published");
                return;
            }
            // Convert the transformation_result result to a PoseWithCovarianceStamped message and publish it
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = map_frame;
            pose_msg.pose.pose.position.x = transformation_result(0, 3);
            pose_msg.pose.pose.position.y = transformation_result(1, 3);
            pose_msg.pose.pose.position.z = transformation_result(2, 3);
            // set orientation
            Eigen::Matrix3f rotation = transformation_result.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rotation);
            pose_msg.pose.pose.orientation.x = q.x();
            pose_msg.pose.pose.orientation.y = q.y();
            pose_msg.pose.pose.orientation.z = q.z();
            pose_msg.pose.pose.orientation.w = q.w();
            publisher_->publish(pose_msg);
            RCLCPP_INFO(
                this->get_logger(),
                "RELOCALIZATION SUCCEEDED: published /icp_result, X=%.3f Y=%.3f Z=%.3f YAW=%.3f rad (%.1f deg), fitness=%.6f",
                transformation_result(0, 3), transformation_result(1, 3),
                transformation_result(2, 3), yaw, yaw * 180.0 / M_PI, fitness_score);

            // Transform the input cloud using the ICP result
            pcl::transformPointCloud(*input_cloud, *input_cloud, transformation_result);
            // Publish the transformed input cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
            transformed_cloud_msg.header.stamp = this->now();
            transformed_cloud_msg.header.frame_id = map_frame;
            transformed_cloud_pub_->publish(transformed_cloud_msg);
            rclcpp::shutdown();
        }
        else
        {
            converged_count = 0;
            Eigen::Matrix4f transformation_result = initGuess;
            pcl::transformPointCloud(*input_cloud, *input_cloud, transformation_result);
            // Publish the transformed input cloud
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
            transformed_cloud_msg.header.stamp = this->now();
            transformed_cloud_msg.header.frame_id = map_frame;
            transformed_cloud_pub_->publish(transformed_cloud_msg);
            RCLCPP_INFO(this->get_logger(), "ICP fitness score is higher than the threshold, no pose is published");
        }
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }

#ifdef USE_LIVOX
    void lvx_cloud_callback(const livox_ros_driver2::msg::CustomMsg::SharedPtr msg)
    {
        // Convert the incoming point cloud to PCL format
        pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        for (int i = 0; i < msg->point_num; i++)
        {
            pcl::PointXYZ point;
            point.x = msg->points[i].x;
            point.y = msg->points[i].y;
            point.z = msg->points[i].z;
            input_cloud->push_back(point);
        }
        input_cloud->width = input_cloud->size();
        input_cloud->height = 1;
        filter_input_cloud(input_cloud);

        if (input_cloud->empty())
        {
            RCLCPP_WARN(this->get_logger(), "ICP input cloud is empty after filtering");
            return;
        }

        // Downsample the input cloud
        pcl::VoxelGrid<pcl::PointXYZ> sor_scan;
        sor_scan.setInputCloud(input_cloud);
        sor_scan.setLeafSize(cloud_voxel_leaf_size, cloud_voxel_leaf_size, cloud_voxel_leaf_size);
        sor_scan.filter(*input_cloud);
        RCLCPP_INFO(this->get_logger(), "Downsampled input cloud to %d data points", input_cloud->width * input_cloud->height);

        // Rotate pcl alone x axis for 180 degree
        // Eigen::Matrix4f rotation = Eigen::Matrix4f::Identity();
        // rotation(1, 1) = -1;
        // rotation(2, 2) = -1;
        // pcl::transformPointCloud(*input_cloud, *input_cloud, rotation);

        // Perform ICP alignment
        pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;
        icp.setInputSource(input_cloud);
        icp.setInputTarget(target_cloud_);
        icp.setMaximumIterations(solver_max_iter);
        // icp.setTransformationEpsilon(1e-8);
        icp.setMaxCorrespondenceDistance(max_correspondence_distance);
        icp.setRANSACOutlierRejectionThreshold(RANSAC_outlier_rejection_threshold);
        // icp.setRANSACIterations(100);
        pcl::PointCloud<pcl::PointXYZ> final_cloud;
        icp.align(final_cloud, initGuess);

        // Get fitness score
        double fitness_score = icp.getFitnessScore(max_correspondence_distance);
        RCLCPP_INFO(this->get_logger(), "ICP fitness score: %f", fitness_score);

        if (icp.hasConverged() && fitness_score < fitness_score_thre && converged_count > converged_count_thre)
        {
            RCLCPP_INFO(this->get_logger(), "ICP converged!!!");
            Eigen::Matrix4f transformation_result = icp.getFinalTransformation();
            // Convert the transformation_result result to a PoseWithCovarianceStamped message and publish it
            geometry_msgs::msg::PoseWithCovarianceStamped pose_msg;
            pose_msg.header.stamp = this->now();
            pose_msg.header.frame_id = map_frame;
            pose_msg.pose.pose.position.x = transformation_result(0, 3);
            pose_msg.pose.pose.position.y = transformation_result(1, 3);
            pose_msg.pose.pose.position.z = transformation_result(2, 3);
            // set orientation
            Eigen::Matrix3f rotation = transformation_result.block<3, 3>(0, 0);
            Eigen::Quaternionf q(rotation);
            pose_msg.pose.pose.orientation.x = q.x();
            pose_msg.pose.pose.orientation.y = q.y();
            pose_msg.pose.pose.orientation.z = q.z();
            pose_msg.pose.pose.orientation.w = q.w();
            publisher_->publish(pose_msg);

            // Transform the input cloud using ICP result
            pcl::transformPointCloud(*input_cloud, *input_cloud, transformation_result);
            // Publish the transformed input cloud for the last time
            sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
            pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
            transformed_cloud_msg.header.stamp = this->now();
            transformed_cloud_msg.header.frame_id = map_frame;
            transformed_cloud_pub_->publish(transformed_cloud_msg);
            rclcpp::shutdown();
        }
        else if(icp.hasConverged() && fitness_score < fitness_score_thre && converged_count <= converged_count_thre)
        {
            converged_count++;
            initGuess = icp.getFinalTransformation(); // update the initial guess with the ICP result
            pcl::transformPointCloud(*input_cloud, *input_cloud, initGuess);
            RCLCPP_INFO(this->get_logger(), "ICP converged with low error, but not enough count, no pose is published");
        }
        else if(icp.hasConverged() && fitness_score >= fitness_score_thre)
        {
            converged_count = 0;
            initGuess = icp.getFinalTransformation(); // update the initial guess with the ICP result
            pcl::transformPointCloud(*input_cloud, *input_cloud, initGuess);
            RCLCPP_INFO(this->get_logger(), "ICP converged with high error, no pose is published");
        }
        else // if ICP doesn't converge
        {
            converged_count = 0;
            pcl::transformPointCloud(*input_cloud, *input_cloud, initGuess);
            RCLCPP_INFO(this->get_logger(), "ICP doesn't converge!!!");
        }

        // Publish the transformed input cloud
        sensor_msgs::msg::PointCloud2 transformed_cloud_msg;
        pcl::toROSMsg(*input_cloud, transformed_cloud_msg);
        transformed_cloud_msg.header.stamp = this->now();
        transformed_cloud_msg.header.frame_id = map_frame;
        transformed_cloud_pub_->publish(transformed_cloud_msg);
        RCLCPP_INFO(this->get_logger(), "ICP fitness score is higher than the threshold, no pose is published");
        target_cloud_msg.header.stamp = this->now();
        map_pub_->publish(target_cloud_msg);
    }
#endif

    void pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        // Convert the incoming pose to an Eigen matrix
        initGuess = Eigen::Matrix4f::Identity();
        initGuess(0, 3) = msg->pose.pose.position.x;
        initGuess(1, 3) = msg->pose.pose.position.y;
        initGuess(2, 3) = msg->pose.pose.position.z;
        // You need to convert the quaternion to a rotation matrix and set it to the upper-left 3x3 part of the matrix
        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        tf2::Matrix3x3 rot_mat(q);
        for (int i = 0; i < 3; i++)
        {
            for (int j = 0; j < 3; j++)
            {
                initGuess(i, j) = rot_mat[i][j];
            }
        }
        double r,p,yaw;
        rot_mat.getRPY(r, p, yaw);
        RCLCPP_INFO(this->get_logger(), "Initial guess: \n x: %f, y: %f, z: %f, a: %f", msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z, yaw);
    }

    rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
#ifdef USE_LIVOX
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr lvx_cloud_sub_;
#endif
    rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr pose_sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr map_pub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr transformed_cloud_pub_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr target_cloud_{new pcl::PointCloud<pcl::PointXYZ>};

    Eigen::Matrix4f initGuess;
    double initial_x, initial_y, initial_z, initial_a;
    int solver_max_iter;
    double max_correspondence_distance, RANSAC_outlier_rejection_threshold;
    std::string map_path, map_frame;
    double fitness_score_thre;
    double map_voxel_leaf_size, cloud_voxel_leaf_size;
    sensor_msgs::msg::PointCloud2 target_cloud_msg;
    int converged_count = 0;
    int converged_count_thre;
    std::string pcl_type;
    std::string livox_topic;
    std::string pointcloud_topic;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ICPNode>());
    rclcpp::shutdown();
    return 0;
}
