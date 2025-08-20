#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <vector>

class ImageCompressedNode : public rclcpp::Node
{
public:
  ImageCompressedNode() : Node("image_compressed_node")
  {
    // 创建压缩图像发布者
    left_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/left/image_raw/compressed", 10);
    right_compressed_pub_ = this->create_publisher<sensor_msgs::msg::CompressedImage>("/right/image_raw/compressed", 10);
    
    // 创建图像订阅者
    left_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/left/image_raw", 10,
      std::bind(&ImageCompressedNode::leftImageCallback, this, std::placeholders::_1));
    
    right_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      "/right/image_raw", 10,
      std::bind(&ImageCompressedNode::rightImageCallback, this, std::placeholders::_1));
  }

private:
  void leftImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    compressAndPublish(msg, left_compressed_pub_);
  }

  void rightImageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
  {
    compressAndPublish(msg, right_compressed_pub_);
  }

  void compressAndPublish(const sensor_msgs::msg::Image::SharedPtr image_msg, 
                         rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr pub)
  {
    try {
      // 转换ROS图像消息到OpenCV格式
      cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);
      
      // 缩放图像以减少数据量（可选）
      cv::Mat resized_image;
      cv::resize(cv_ptr->image, resized_image, cv::Size(), 0.25, 0.25); // 缩放到原尺寸的25%（更小）
      // 或者
      // cv::resize(cv_ptr->image, resized_image, cv::Size(), 0.75, 0.75); // 缩放到原尺寸的75%（较大）
      // 或者固定尺寸
      // cv::resize(cv_ptr->image, resized_image, cv::Size(640, 480)); // 固定为640x480
      
      // 创建压缩图像消息
      auto compressed_msg = std::make_shared<sensor_msgs::msg::CompressedImage>();
      compressed_msg->header = image_msg->header;
      compressed_msg->format = "jpeg";

      // 设置JPEG压缩参数 - 降低质量以提高传输速度
      std::vector<int> compression_params;
      compression_params.push_back(cv::IMWRITE_JPEG_QUALITY);
      compression_params.push_back(30); // 压缩质量30 (0-100)
      
      /*
       * JPEG质量压缩原理：
       * 1. 色彩空间转换：RGB -> YUV (人眼对亮度比色彩更敏感)
       * 2. 色度采样：对UV分量进行下采样 (4:2:0)
       * 3. DCT变换：将8x8像素块转换到频域
       * 4. 量化：根据质量参数丢弃高频信息
       *    - 质量100：几乎无量化，文件大
       *    - 质量30：大量量化，丢失细节，文件小
       *    - 质量0：最大量化，严重失真
       * 5. 熵编码：霍夫曼编码进一步压缩
       * 
       * 质量参数影响：
       * - 高质量(80-100)：保留更多细节，文件较大
       * - 中等质量(50-80)：平衡质量和大小
       * - 低质量(10-50)：明显压缩伪影，文件很小
       * - 极低质量(0-10)：严重失真，不推荐
       */

      // 压缩图像
      std::vector<uchar> compressed_data;
      bool success = cv::imencode(".jpg", resized_image, compressed_data, compression_params);
      
      if (success) {
        compressed_msg->data = compressed_data;
        pub->publish(*compressed_msg);
        RCLCPP_DEBUG(this->get_logger(), "Compressed image size: %zu bytes", compressed_data.size());
      } else {
        RCLCPP_ERROR(this->get_logger(), "Failed to compress image");
      }
    } catch (cv_bridge::Exception& e) {
      RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr left_compressed_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CompressedImage>::SharedPtr right_compressed_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr left_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr right_sub_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ImageCompressedNode>());
  rclcpp::shutdown();
  return 0;
}
