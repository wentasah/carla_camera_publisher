#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fmt/core.h>
#include <mutex>
#include <optional>

#include "ros.hpp"

using rcl_interfaces::msg::ParameterDescriptor;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : rclcpp::Node("carla_camera_publisher") {
        this->declare_parameter("topic", "/carla_camera_publisher/camera", ParameterDescriptor{}.set__description("Topic where to publish camera images"));
        this->declare_parameter("frame_id", "carla_camera_frame", ParameterDescriptor{}.set__description("Coordinate frame of the camera"));

        this->declare_parameter("width", 640, ParameterDescriptor{}.set__description("Image width"));
        this->declare_parameter("height", 360, ParameterDescriptor{}.set__description("Image height"));
        rcl_interfaces::msg::FloatingPointRange range;
        range.set__from_value(0.0).set__to_value(+180.0);
        this->declare_parameter("fov", 110.0, ParameterDescriptor{}.set__description("Field of view [°]").set__floating_point_range({range}));
        range.set__from_value(0.0).set__to_value(100);
        this->declare_parameter("sensor_tick", 1.0/25.0, ParameterDescriptor{}.set__description("Frame rate [1/s]").set__floating_point_range({range}));
        this->declare_parameter("ego_vehicle_role_name", "ego_vehicle", ParameterDescriptor{}.set__description("Ego vehicle role name"));
        range.set__from_value(-100.0).set__to_value(+100);
        this->declare_parameter("position.x", 0.0, ParameterDescriptor{}.set__description("x").set__floating_point_range({range}));
        this->declare_parameter("position.y", 0.0, ParameterDescriptor{}.set__description("y").set__floating_point_range({range}));
        this->declare_parameter("position.z", 2.0, ParameterDescriptor{}.set__description("z").set__floating_point_range({range}));
        range.set__from_value(-180.0).set__to_value(+180.0);
        this->declare_parameter("orientation.pitch", -10.0, ParameterDescriptor{}.set__description("pitch").set__floating_point_range({range}));
        this->declare_parameter("orientation.yaw", 0.0, ParameterDescriptor{}.set__description("yaw").set__floating_point_range({range}));
        this->declare_parameter("orientation.roll", 0.0, ParameterDescriptor{}.set__description("roll").set__floating_point_range({range}));

        update_parameters();

        param_callback_handle_ = this->add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &) {
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                update_parameters();
                return result;
            });
    }

    std::optional<Params> has_new_params() {
        std::lock_guard guard(params_mutex);
        if (params_updated) {
            params_updated = false;
            return std::optional(params);
        } else {
            return std::nullopt;
        }
    }

    std::string get_frame_id() { return frame_id; }
  private:
    std::string topic, frame_id;

    std::mutex params_mutex;
    Params params;
    bool params_updated = false;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    void update_parameters() {
        std::lock_guard guard(params_mutex);
        params_updated = true;
        params.width = this->get_parameter("width").as_int();
        params.height = this->get_parameter("height").as_int();
        params.fov = this->get_parameter("fov").as_double();
        params.sensor_tick = this->get_parameter("sensor_tick").as_double();
        params.ego_vehicle_role_name = this->get_parameter("ego_vehicle_role_name").as_string();
        params.position[0] = static_cast<float>(get_parameter("position.x").as_double());
        params.position[1] = static_cast<float>(get_parameter("position.y").as_double());
        params.position[2] = static_cast<float>(get_parameter("position.z").as_double());
        params.orientation[0] = static_cast<float>(get_parameter("orientation.pitch").as_double());
        params.orientation[1] = static_cast<float>(get_parameter("orientation.yaw").as_double());
        params.orientation[2] = static_cast<float>(get_parameter("orientation.roll").as_double());
    }
};

static std::shared_ptr<CameraPublisher> node;

static std::mutex mutex;
static image_transport::ImageTransport *img_transport = nullptr;
static image_transport::CameraPublisher camera_publisher;

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

    camera_info.header.frame_id = node->get_frame_id();
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
    image.header.frame_id = node->get_frame_id();
    image.height = h;
    image.width = w;
    image.encoding = "bgra8";
    image.set__step(w * 4);
    image.data.resize(w * h * 4);
    std::memcpy(image.data.data(), data, image.data.size());
    {
        std::lock_guard guard(mutex);
        if (img_transport)
            camera_publisher.publish(image, camera_info, rclcpp::Time(stamp * 1e9));
    }

}

std::optional<Params> ros_has_new_params()
{
    return node->has_new_params();
}
