#include "image_transport/image_transport.hpp"
#include "rcl_interfaces/msg/floating_point_range.hpp"
#include "rclcpp/rclcpp.hpp"
#include "tf2/LinearMath/Quaternion.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2_ros/static_transform_broadcaster.h"
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fmt/core.h>
#include <mutex>
#include <optional>

#include "ros.hpp"

using rcl_interfaces::msg::ParameterDescriptor;

// Protects all global variables in this file
static std::mutex mutex;

// For faster reaction to changes in the CARLA thread
static std::condition_variable cv;

class CameraPublisher : public rclcpp::Node
{
public:
    CameraPublisher() : rclcpp::Node("carla_camera_publisher") {
        this->declare_parameter("topic", "/carla_camera_publisher/camera", ParameterDescriptor{}.set__description("Topic where to publish camera images"));
        this->declare_parameter("frame_id", "carla_camera", ParameterDescriptor{}.set__description("Coordinate frame of the camera"));

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

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        update_parameters();

        param_callback_handle_ = this->add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> &) {
              // Called by ROS - we have to synchronize with the CARLA thread
                rcl_interfaces::msg::SetParametersResult result;
                result.successful = true;
                update_parameters();
                cv.notify_one();
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

    void update_tf(rclcpp::Time stamp, const Params &params_for_tf) {
        // Called from CARLA thread
        geometry_msgs::msg::TransformStamped t;

        if (params_for_tf.position == last_position &&
            params_for_tf.orientation == last_orientation)
          return;
        last_position = params_for_tf.position;
        last_orientation = params_for_tf.orientation;

        t.header.stamp = stamp;
        t.header.frame_id = params.ego_vehicle_role_name;
        t.child_frame_id = frame_id;

        t.transform.translation.x = params_for_tf.position[0];
        t.transform.translation.y = params_for_tf.position[1];
        t.transform.translation.z = params_for_tf.position[2];
        tf2::Quaternion quat, quat_swap;
        quat.setRPY(params_for_tf.orientation[2] / 180.0 * M_PI,
                    -params_for_tf.orientation[0] / 180.0 * M_PI,
                    -params_for_tf.orientation[1] / 180.0 * M_PI);
        tf2::Matrix3x3(0, 0, +1, //
                       -1, 0, 0, //
                       0, -1, 0).getRotation(quat_swap);
        quat *= quat_swap;
        t.transform.rotation.x = quat.x();
        t.transform.rotation.y = quat.y();
        t.transform.rotation.z = quat.z();
        t.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(t);
    }

    std::string get_frame_id() { return frame_id; }
  private:
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;

    std::string topic, frame_id;

    std::mutex params_mutex;
    Params params;
    bool params_updated = false;

    std::array<float, 3> last_position = {123456};
    std::array<float, 3> last_orientation;

    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_callback_handle_;

    void update_parameters() {
        std::lock_guard guard(params_mutex);
        params_updated = true;

        frame_id = this->get_parameter("frame_id").as_string();

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
static image_transport::ImageTransport *img_transport = nullptr;
static image_transport::CameraPublisher camera_publisher;

void ros_init(int argc, const char *argv[])
{
    rclcpp::init(argc, argv);
    {
        std::lock_guard guard(mutex);
        node = std::make_shared<CameraPublisher>();
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
        camera_publisher.shutdown();
        delete img_transport;
        img_transport = nullptr;
        node = nullptr;
    }
    rclcpp::shutdown();
    cv.notify_one();
}

void ros_publish(double stamp, uint32_t w, uint32_t h, double fov, void *data, const Params &params) {
    auto time = rclcpp::Time(stamp * 1e9);
    sensor_msgs::msg::CameraInfo camera_info;

    std::lock_guard guard(mutex);
    if (!node)
        return;

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

    sensor_msgs::msg::Image image;
    image.header.frame_id = node->get_frame_id();
    image.height = h;
    image.width = w;
    image.encoding = "bgra8";
    image.set__step(w * 4);
    image.data.resize(w * h * 4);
    std::memcpy(image.data.data(), data, image.data.size());
    node->update_tf(time, params);
    if (img_transport)
        camera_publisher.publish(image, camera_info, time);
}

std::optional<Params> ros_wait_new_params(unsigned milliseconds)
{
    std::unique_lock lk(mutex);
    if (!node)
        return std::nullopt;
    if (milliseconds > 0) {
        // TODO: Use proper conditions to not wait if notification
        // happens outside of wait_for.
        cv.wait_for(lk, std::chrono::milliseconds(milliseconds));
        if (!node)
            return std::nullopt;
    }
    return node->has_new_params();
}
