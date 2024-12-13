#include "image_transport/image_transport.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fmt/core.h>
#include <mutex>

#include "ros.hpp"

using rcl_interfaces::msg::ParameterDescriptor;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : rclcpp::Node("carla_camera_publisher") {
        this->declare_parameter(
            "topic", "/sensor_stack/cameras/zed2/zed_node/left/image_rect_color",
            ParameterDescriptor{}.set__description("Topic where to publish camera images"));
    }
private:
};

std::shared_ptr<CameraPublisher> node;

std::mutex mutex;
image_transport::ImageTransport *img_transport = nullptr;
image_transport::CameraPublisher camera_publisher;

void ros_init(int argc, const char *argv[])
{
    rclcpp::init(argc, argv);
    node = std::make_shared<CameraPublisher>();
    {
        std::lock_guard guard(mutex);
        img_transport = new image_transport::ImageTransport(node);
        auto topic = node->get_parameter("topic").as_string();
        fmt::println("Publishing camera as {}", topic);
        camera_publisher = img_transport->advertiseCamera(topic, 1);
    }
}

void ros_run()
{
    rclcpp::spin(node);
    {
        std::lock_guard guard(mutex);
        delete img_transport;
    }
    rclcpp::shutdown();
}

void ros_publish(double stamp, uint32_t w, uint32_t h, double fov, void *data) {
    sensor_msgs::msg::CameraInfo camera_info;

    camera_info.header.frame_id = "zed_left_camera_optical_frame"; // FIXME
    camera_info.width = w;
    camera_info.height = h;
    camera_info.distortion_model = "plumb_bob";

    double cx = camera_info.width / 2.0;
    double cy = camera_info.height / 2.0;
    double fx = camera_info.width / (2.0 * tan(fov * M_PI / 360.0));
    double fy = fx;

    camera_info.k = {fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0};
    camera_info.d = {0.0, 0.0, 0.0, 0.0, 0.0};
    camera_info.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    camera_info.p = {fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0};

    std_msgs::msg::Header hdr;
    sensor_msgs::msg::Image image;
    image.header.frame_id = "zed_left_camera_optical_frame"; // FIXME
    image.height = h;
    image.width = w;
    image.encoding = "bgra8";
    image.set__step(w * 4);
    image.data.resize(w * h * 4);
    std::memcpy(image.data.data(), data, image.data.size());
    {
        std::lock_guard guard(mutex);
        if (img_transport)
            camera_publisher.publish(image,
                                     camera_info,
                                     rclcpp::Time(stamp * 1e9));
    }

}
