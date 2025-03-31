#include <cstdio>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>

#include <carla/client/Actor.h>
#include <carla/client/ActorBlueprint.h>
#include <carla/client/ActorList.h>
#include <carla/client/BlueprintLibrary.h>
#include <carla/client/Client.h>
#include <carla/client/Map.h>
#include <carla/client/Sensor.h>
#include <carla/client/TimeoutException.h>
#include <carla/client/World.h>
#include <carla/geom/Transform.h>
#include <carla/image/ImageIO.h>
#include <carla/image/ImageView.h>
#include <carla/sensor/data/Image.h>

#include <fmt/core.h>

// We cannot include ROS stuff directly here, because it causes some
// collision between boost and ROS headers. Therefore, we have ROS
// code in a separate compilation unit use a minimal interface between
// those units.
#include "ros.hpp"

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

using std::to_string;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
    EXPECT_TRUE(range.size() > 0u);
    std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
    return range[dist(std::forward<RNG>(generator))];
}

static auto ParseArguments(int argc, const char *argv[]) {
    EXPECT_TRUE((argc == 1) || (argc >= 3));
    using ResultType = std::tuple<std::string, uint16_t>;
    return argc >= 3 ?
        ResultType{argv[1u], std::stoi(argv[2u])} :
        ResultType{"localhost", 2000u};
}

int main(int argc, const char *argv[])
{
    try {
        ros_init(argc, argv);

        Params params = ros_wait_new_params(0).value();

        // Run ROS in a separate thread
        std::atomic<bool> ros_running = true;
        std::thread ros_thread([&]() {
            ros_run();
            fmt::println("ROS node shut down");
            ros_running = false;
        });
        ros_thread.detach();

        std::string host;
        uint16_t port;
        std::tie(host, port) = ParseArguments(argc, argv);

        std::mt19937_64 rng((std::random_device())());

    reconnect:
        try {
            fmt::println("Connecting to CARLA at {}:{}", host, port);
            auto client = cc::Client(host, port);

            // Set short timeout for getting the version (and reconnection)
            client.SetTimeout(5s);
            fmt::println("Client API version: {}", client.GetClientVersion());
            fmt::println("Server API version: {}", client.GetServerVersion());

            // Longer timeout needed for getting the map
            client.SetTimeout(30s);
            fmt::println("Reading world...");
            auto world = client.GetWorld();

            auto settings = world.GetSettings();
            fmt::println("Fixed delta seconds: {}", settings.fixed_delta_seconds.get_value_or(0));
            fmt::println("Synchronous mode: {}", settings.synchronous_mode);

            auto vehicles = std::set<std::string>();
            boost::shared_ptr<cc::Actor> ego_vehicle;
            auto episode = client.GetCurrentEpisode().GetId();
            while (!ego_vehicle) {
                fmt::println("Waiting for the ego vehicle with role {}...", params.ego_vehicle_role_name);
                if (episode != client.GetCurrentEpisode().GetId()) {
                    fmt::println(stderr, "Episode changed. Reconnecting...");
                    goto reconnect;
                }
                auto actors = world.GetActors()->Filter("vehicle.*");
                for (auto actor : *actors) {
                    if (!vehicles.contains(actor->GetDisplayId())) {
                        fmt::println("Found actor: {}", actor->GetDisplayId());
                        for (auto attr : actor->GetAttributes()) {
                            fmt::println("  {}: {}", attr.GetId(), attr.GetValue());
                            if (attr.GetId() == "role_name" && attr.GetValue() == params.ego_vehicle_role_name)
                                ego_vehicle = actor;
                        }
                        vehicles.insert(actor->GetDisplayId());
                    }
                }
                if (!ros_running.load()) // Ctrl-C
                    return 0;

                if (!ego_vehicle)
                    sleep(1);
            }
            fmt::println("Found ego_vehicle: {}", ego_vehicle->GetDisplayId());

        create_camera:
            // Find a camera blueprint.
            auto blueprint_library = world.GetBlueprintLibrary();
            auto camera_bp = blueprint_library->Find("sensor.camera.rgb");
            EXPECT_TRUE(camera_bp != nullptr);
            const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("image_size_x")).Set(to_string(params.width));
            const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("image_size_y")).Set(to_string(params.height));
            const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("fov")).Set(to_string(params.fov));
            const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("sensor_tick")).Set(to_string(params.sensor_tick));
            for (auto &attr : *camera_bp)
                fmt::println("  {}: {}", attr.GetId(), attr.GetValue());

            // Spawn a camera attached to the vehicle.
            auto camera_transform = cg::Transform {
                cg::Location{params.position[0], params.position[1], params.position[2]}, // x, y, z.
                cg::Rotation{params.orientation[0], params.orientation[1], params.orientation[2]} }; // pitch, yaw, roll.
            auto cam_actor = world.SpawnActor(*camera_bp, camera_transform, ego_vehicle.get());
            auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

            // Register a callback to save images to disk.
            camera->Listen([&params](auto data) {
                boost::shared_ptr<csd::Image> image =
                    boost::static_pointer_cast<csd::Image>(data);
                EXPECT_TRUE(image != nullptr);
                fmt::println("Image @ {:.2f}s: {}x{} FoV:{} frame:{}",
                             image->GetTimestamp(), image->GetWidth(),
                             image->GetHeight(), image->GetFOVAngle(),
                             image->GetFrame());
                ros_publish(image->GetTimestamp(), image->GetWidth(),
                            image->GetHeight(), image->GetFOVAngle(),
                            image->data(), params);
            });

            client.SetTimeout(5s);
            std::optional<Params> new_params;
            while (ros_running && ego_vehicle->IsActive() && !new_params) {
                client.GetServerVersion(); // throws exception if the server dies
                new_params = ros_wait_new_params(1000);
            }
            fmt::println(stderr, "Loop exit");

            // Remove actors from the simulation.
            camera->Destroy();

            if (!ego_vehicle->IsActive() && ros_running) {
                fmt::println(stderr, "Ego vehicle is inactive. Reconnecting...");
                goto reconnect;
            }
            if (new_params && ros_running) {
                params = new_params.value();
                goto create_camera;
            }
        } catch (const cc::TimeoutException &e) {
            fmt::println("{}", e.what());
            goto reconnect;
        }
    } catch (const std::exception &e) {
        fmt::println("\nException: {}", e.what());
        return 2;
    }
    fmt::println(stderr, "Terminating");
}
