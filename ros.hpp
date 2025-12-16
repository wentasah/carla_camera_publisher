#include <cstdint>
#include <optional>
#include <string>
#include <array>

struct Params {
        unsigned width;
        unsigned height;
        float fov;
        float sensor_tick;
	std::string ego_vehicle_role_name;
        std::array<float, 3> position;
        std::array<float, 3> orientation;
        bool attach_to_ego;
};

void ros_init(int argc, const char *argv[]);
void ros_run();
void ros_publish(double stamp, uint32_t w, uint32_t h, double fov, void *data, const Params &params);
std::optional<Params> ros_wait_new_params(unsigned milliseconds);
