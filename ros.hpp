#include <cstdint>
#include <optional>
#include <string>

struct Params {
        unsigned width;
        unsigned height;
        float fov;
        float sensor_tick;
	std::string ego_vehicle_role_name;
        float position[3];
        float orientation[3];
};

void ros_init(int argc, const char *argv[]);
void ros_run();
void ros_publish(double stamp, uint32_t w, uint32_t h, double fov, void *data);
std::optional<Params> ros_has_new_params();
