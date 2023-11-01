#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

using std::placeholders::_1;

// https://answers.ros.org/question/219876/using-sensor_msgspointcloud2-natively/
// https://github.com/ros2/turtlebot2_demo/blob/master/depthimage_to_pointcloud2/src/depthimage_to_pointcloud2_node.cpp
// http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/CameraInfo.html

class Depth2PC : public rclcpp::Node
{
  public:
    Depth2PC();
    ~Depth2PC();

  private:
    // Данные
    sensor_msgs::msg::Image::SharedPtr img;					      // Карта глубины
    sensor_msgs::msg::CameraInfo::SharedPtr camera_info;	// Параметры камеры
    sensor_msgs::msg::PointCloud2::SharedPtr pc;			    // Облако точек

    // Топики
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr subscriber_ci;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;

    // Параметры топиков
	  std::string depth_topicname,
			camerainfo_topicname,
			publisher_topicname;

    std::string output_frame_id;


    // Коллбэки
    void depth_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void info_callback(const sensor_msgs::msg::CameraInfo::SharedPtr info);

    template<typename T>
    void convert(sensor_msgs::msg::Image::SharedPtr image,
					sensor_msgs::msg::PointCloud2  *pc,
					sensor_msgs::msg::CameraInfo::SharedPtr info);
};
