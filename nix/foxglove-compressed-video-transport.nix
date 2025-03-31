# Automatically generated by: ros2nix --fetch --output-as-nix-pkg-name
{ lib, buildRosPackage, fetchFromGitHub, ament-cmake, ament-cmake-clang-format, ament-cmake-ros, ament-lint-auto, ament-lint-common, ffmpeg-encoder-decoder, foxglove-msgs, image-transport, pluginlib, rclcpp, rcutils, ros-environment, sensor-msgs, std-msgs }:
buildRosPackage rec {
  pname = "ros-rolling-foxglove-compressed-video-transport";
  version = "1.0.2";

  src = fetchFromGitHub {
    owner = "ros-misc-utilities";
    repo = "foxglove_compressed_video_transport";
    rev = "306a8d994d671c790a0d1d72f52a02513e2bbe15";
    sha256 = "02kgjn2jnd8gx5x8krbk2kf49s80vxw1f7f8l1ijkq9278x2pc66";
  };

  buildType = "ament_cmake";
  buildInputs = [ ament-cmake ament-cmake-ros ros-environment ];
  checkInputs = [ ament-cmake-clang-format ament-lint-auto ament-lint-common ];
  propagatedBuildInputs = [ ffmpeg-encoder-decoder foxglove-msgs image-transport pluginlib rclcpp rcutils sensor-msgs std-msgs ];
  nativeBuildInputs = [ ament-cmake ament-cmake-ros ros-environment ];

  meta = {
    description = "foxglove_compressed_video_transport provides a plugin to image_transport for
    transparently sending an image stream encoded in foxglove compressed video packets.";
    license = with lib.licenses; [ "Apache-2" ];
  };
}
