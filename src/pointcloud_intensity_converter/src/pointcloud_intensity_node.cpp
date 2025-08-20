#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"
#include <sstream>
// 添加TF2相关头文件
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "tf2_eigen/tf2_eigen.hpp"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"

// debug 模式:   ros2 run pointcloud_intensity_converter pointcloud_intensity_node --ros-args -p debug:=true
// ! debug 模式: ros2 run pointcloud_intensity_converter pointcloud_intensity_node
class RGBToIntensityNode : public rclcpp::Node
{
public:
    RGBToIntensityNode()
    : Node("rgb_to_intensity_node")
    {
        using std::placeholders::_1;

        // 添加debug参数，默认为false
        this->declare_parameter("debug", false);
        debug_ = this->get_parameter("debug").as_bool();

        // 初始化TF2
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/depth_registered/points", 10,
            std::bind(&RGBToIntensityNode::pointCloudCallback, this, _1));

        pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
            "/registered_scan", 10);

        RCLCPP_INFO(this->get_logger(), "✅ RGB to Intensity Node started");
        RCLCPP_INFO(this->get_logger(), "📥 Subscribing to: /camera/depth_registered/points");
        RCLCPP_INFO(this->get_logger(), "📤 Publishing to: /registered_scan");
        RCLCPP_INFO(this->get_logger(), "🔍 Debug mode: %s", debug_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "🌐 Converting point cloud to map frame");
    }

private:
    template<typename... Args>
    void debug_print(const char* format, Args... args)
    {
        if (debug_) {
            RCLCPP_INFO(this->get_logger(), format, args...);
        }
    }

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        try {
            // 记录处理开始时间
            auto start_time = this->now();
            
            // 输入点云信息日志
            debug_print("接收到点云数据 - 帧ID: %s, 大小: %dx%d, 点数: %d, 字段数: %zu",
                      msg->header.frame_id.c_str(), msg->height, msg->width, 
                      msg->width * msg->height, msg->fields.size());
            
            // 打印输入点云字段信息
            if (debug_) {
                std::stringstream fields_info;
                fields_info << "输入点云字段: ";
                for (const auto& field : msg->fields) {
                    fields_info << field.name << "(offset:" << field.offset << ", datatype:" << field.datatype << ") ";
                }
                RCLCPP_INFO(this->get_logger(), "%s", fields_info.str().c_str());
                RCLCPP_INFO(this->get_logger(), "point_step: %d, row_step: %d", msg->point_step, msg->row_step);
            }

            // 检查是否有rgb字段
            bool has_rgb = false;
            for (const auto& field : msg->fields) {
                if (field.name == "rgb") {
                    has_rgb = true;
                    break;
                }
            }

            if (!has_rgb) {
                RCLCPP_ERROR(this->get_logger(), "输入点云没有rgb字段，无法处理！");
                return;
            }

            sensor_msgs::msg::PointCloud2 intensity_cloud;
            intensity_cloud.header = msg->header;
            intensity_cloud.height = msg->height;
            intensity_cloud.width = msg->width;
            intensity_cloud.is_dense = msg->is_dense;
            intensity_cloud.is_bigendian = msg->is_bigendian;
            intensity_cloud.point_step = 16; // 4 float fields: x, y, z, intensity
            intensity_cloud.row_step = intensity_cloud.point_step * intensity_cloud.width;

            intensity_cloud.fields.resize(4);
            intensity_cloud.fields[0].name = "x";
            intensity_cloud.fields[0].offset = 0;
            intensity_cloud.fields[0].datatype = sensor_msgs::msg::PointField::FLOAT32;
            intensity_cloud.fields[0].count = 1;

            intensity_cloud.fields[1].name = "y";
            intensity_cloud.fields[1].offset = 4;
            intensity_cloud.fields[1].datatype = sensor_msgs::msg::PointField::FLOAT32;
            intensity_cloud.fields[1].count = 1;

            intensity_cloud.fields[2].name = "z";
            intensity_cloud.fields[2].offset = 8;
            intensity_cloud.fields[2].datatype = sensor_msgs::msg::PointField::FLOAT32;
            intensity_cloud.fields[2].count = 1;

            intensity_cloud.fields[3].name = "intensity";
            intensity_cloud.fields[3].offset = 12;
            intensity_cloud.fields[3].datatype = sensor_msgs::msg::PointField::FLOAT32;
            intensity_cloud.fields[3].count = 1;

            intensity_cloud.data.resize(intensity_cloud.row_step * intensity_cloud.height);

            sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");
            sensor_msgs::PointCloud2ConstIterator<float> iter_rgb(*msg, "rgb");

            sensor_msgs::PointCloud2Iterator<float> out_x(intensity_cloud, "x");
            sensor_msgs::PointCloud2Iterator<float> out_y(intensity_cloud, "y");
            sensor_msgs::PointCloud2Iterator<float> out_z(intensity_cloud, "z");
            sensor_msgs::PointCloud2Iterator<float> out_intensity(intensity_cloud, "intensity");

            // 样本数据存储
            std::vector<std::string> sample_points;
            
            // 只在debug模式下收集样本点
            bool collect_samples = debug_;
            uint32_t total_points = msg->width * msg->height;
            uint32_t max_samples = 5;
            uint32_t sample_count = (total_points < max_samples) ? total_points : max_samples;
            size_t sample_interval = (total_points > 0) ? (total_points / sample_count) : 1;
            if (sample_interval == 0) sample_interval = 1;

            for (size_t i = 0; i < msg->width * msg->height; ++i)
            {
                *out_x = *iter_x;
                *out_y = *iter_y;
                *out_z = *iter_z;

                // 从rgb字段中提取r, g, b值
                // rgb字段通常是一个浮点数，其内存表示为RGBA四个字节
                uint32_t rgb_val = *reinterpret_cast<const uint32_t*>(&(*iter_rgb));
                uint8_t r = (rgb_val >> 16) & 0xFF;
                uint8_t g = (rgb_val >> 8) & 0xFF;
                uint8_t b = rgb_val & 0xFF;

                // 亮度计算公式
                *out_intensity = 0.299f * r + 0.587f * g + 0.114f * b;

                // 采样几个点用于日志输出，仅在debug模式下
                if (collect_samples && i % sample_interval == 0 && sample_points.size() < sample_count) {
                    std::stringstream ss;
                    ss << "点 " << i << ": x=" << *iter_x << ", y=" << *iter_y << ", z=" << *iter_z 
                       << ", RGB=(" << (int)r << "," << (int)g << "," << (int)b << ")"
                       << " -> intensity=" << *out_intensity;
                    sample_points.push_back(ss.str());
                }

                ++iter_x; ++iter_y; ++iter_z; ++iter_rgb;
                ++out_x; ++out_y; ++out_z; ++out_intensity;
            }

            // 执行坐标系转换
            sensor_msgs::msg::PointCloud2 output;
            try {
                // 尝试获取从摄像头坐标系到map坐标系的转换
                geometry_msgs::msg::TransformStamped transform_stamped;
                transform_stamped = tf_buffer_->lookupTransform(
                    "map",                     // 目标坐标系
                    msg->header.frame_id,      // 源坐标系
                    tf2::TimePointZero);       // 获取最新的转换
                
                // 应用转换
                tf2::doTransform(intensity_cloud, output, transform_stamped);
                debug_print("成功将点云从 %s 坐标系转换到 map 坐标系", msg->header.frame_id.c_str());
            } catch (tf2::TransformException &ex) {
                RCLCPP_WARN(this->get_logger(), "无法转换坐标系: %s", ex.what());
                RCLCPP_WARN(this->get_logger(), "使用原始点云，但将frame_id设置为map");
                output = intensity_cloud;
                output.header.frame_id = "map";  // 如果转换失败，仍然将frame_id设置为map
            }

            // 输出点云信息日志
            debug_print("转换后点云 - 帧ID: %s, 大小: %dx%d, 点数: %d, 字段数: %zu",
                      output.header.frame_id.c_str(), output.height, output.width, 
                      output.width * output.height, output.fields.size());
            
            // 打印采样点数据
            if (debug_) {
                for (const auto& point_info : sample_points) {
                    RCLCPP_INFO(this->get_logger(), "%s", point_info.c_str());
                }
            }
            
            // 记录处理时间
            auto processing_time = (this->now() - start_time).seconds() * 1000.0;
            debug_print("点云处理用时: %.2f ms, 点云大小: %.2f MB", 
                      processing_time, output.data.size() / (1024.0 * 1024.0));

            pub_->publish(output);
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "处理点云时发生错误: %s", e.what());
        } catch (...) {
            RCLCPP_ERROR(this->get_logger(), "处理点云时发生未知错误");
        }
    }

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pub_;
    bool debug_; // 是否启用调试输出
    
    // TF2相关成员
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

// main 函数整合在同一文件中
int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RGBToIntensityNode>());
    rclcpp::shutdown();
    return 0;
}
