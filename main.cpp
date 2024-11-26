#include <carla/client/Actor.h>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <tuple>
#include <fmt/base.h>

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
#include <unistd.h>

namespace cc = carla::client;
namespace cg = carla::geom;
namespace csd = carla::sensor::data;

using namespace std::chrono_literals;
using namespace std::string_literals;

#define EXPECT_TRUE(pred) if (!(pred)) { throw std::runtime_error(#pred); }

/// Pick a random element from @a range.
template <typename RangeT, typename RNG>
static auto &RandomChoice(const RangeT &range, RNG &&generator) {
  EXPECT_TRUE(range.size() > 0u);
  std::uniform_int_distribution<size_t> dist{0u, range.size() - 1u};
  return range[dist(std::forward<RNG>(generator))];
}

/// Save a semantic segmentation image to disk converting to CityScapes palette.
/*
static void SaveSemSegImageToDisk(const csd::Image &image) {
  using namespace carla::image;

  char buffer[9u];
  std::snprintf(buffer, sizeof(buffer), "%08zu", image.GetFrame());
  auto filename = "_images/"s + buffer + ".png";

  auto view = ImageView::MakeColorConvertedView(
      ImageView::MakeView(image),
      ColorConverter::CityScapesPalette());
  ImageIO::WriteView(filename, view);
}
*/

static auto ParseArguments(int argc, const char *argv[]) {
  EXPECT_TRUE((argc == 1u) || (argc == 3u));
  using ResultType = std::tuple<std::string, uint16_t>;
  return argc == 3u ?
      ResultType{argv[1u], std::stoi(argv[2u])} :
      ResultType{"localhost", 2000u};
}

int main(int argc, const char *argv[]) {
  try {

    std::string host;
    uint16_t port;
    std::tie(host, port) = ParseArguments(argc, argv);

    std::mt19937_64 rng((std::random_device())());

    auto client = cc::Client(host, port);
    client.SetTimeout(40s);

    fmt::println("Client API version: {}", client.GetClientVersion());
    fmt::println("Server API version: {}", client.GetServerVersion());

    auto world = client.GetWorld();

    auto settings = world.GetSettings();
    fmt::println("Fixed delta seconds: {}", settings.fixed_delta_seconds.get_value_or(0));
    fmt::println("Synchronous mode: {}", settings.synchronous_mode);

    auto vehicles = std::set<std::string>();
    boost::shared_ptr<cc::Actor> ego_vehicle;
    while (true) {
      fmt::println("Waiting for the ego vehicle...");
      auto actors = world.GetActors()->Filter("vehicle.*");
      for (auto actor : *actors) {
        if (!vehicles.contains(actor->GetDisplayId())) {
          fmt::println("Actor{}", actor->GetDisplayId());
          for (auto attr : actor->GetAttributes()) {
            fmt::println("  {}: {}", attr.GetId(), attr.GetValue());
            if (attr.GetId() == "role_name" && attr.GetValue() == "ego_vehicle")
              ego_vehicle = actor;
          }
          vehicles.insert(actor->GetDisplayId());
        }
      }
      if (ego_vehicle)
        break;
      else
        sleep(1);
    }
    fmt::println("Found ego_vehicle: {}", ego_vehicle->GetDisplayId());

    /*
    -          "type": "sensor.camera.rgb",
    -          "id": "camera_zed",
    -          "spawn_point": {
    -            "x": 0.485, "y": 0.0506 , "z": 1.5472,
    -            "roll": 0.0, "pitch": -12, "yaw": 0.0
    -          },
    -          "image_size_x": 640,
    -          "image_size_y": 360,
    */
    // Find a camera blueprint.
    auto blueprint_library = world.GetBlueprintLibrary();
    auto camera_bp = blueprint_library->Find("sensor.camera.rgb");
    EXPECT_TRUE(camera_bp != nullptr);
    const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("image_size_x")).Set("640");
    const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("image_size_y")).Set("360");
    const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("fov")).Set("110");
    const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("sensor_tick")).Set(std::to_string(1.0/25.0));
    const_cast<cc::ActorAttribute &>(camera_bp->GetAttribute("enable_postprocess_effects")).Set("false");
    for (auto &attr : *camera_bp)
      fmt::println("  {}: {}", attr.GetId(), attr.GetValue());

    // Spawn a camera attached to the vehicle.
    auto camera_transform =
        cg::Transform{cg::Location{0.485f, 0.0506f, 1.5472f}, // x, y, z.
                      cg::Rotation{-12.0f, 0.0f, 0.0f}}; // pitch, yaw, roll.
    auto cam_actor = world.SpawnActor(*camera_bp, camera_transform, ego_vehicle.get());
    auto camera = boost::static_pointer_cast<cc::Sensor>(cam_actor);

    // Register a callback to save images to disk.
    camera->Listen([](auto data) {
      fmt::println("Image @ {}s", data->GetTimestamp());
      auto image = boost::static_pointer_cast<csd::Image>(data);
      EXPECT_TRUE(image != nullptr);
    });

    // Wait for signal, e.g. SIGINT sent by Ctrl-C
    pause();

    // Remove actors from the simulation.
    camera->Destroy();
  } catch (const cc::TimeoutException &e) {
    fmt::println("{}", e.what());
    return 1;
  } catch (const std::exception &e) {
    fmt::println("\nException: {}", e.what());
    return 2;
  }
}
