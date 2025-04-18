# CARLA camera publisher

C++ ROS node for publishing camera images via ROS image transport.

This project solves the problem of high camera bandwidth requirements
between CARLA server and client when they run on different hosts. For
example, a camera producing 640x360 images at 50 FPS requires
bandwidth around 400 Mbit/s. When the server is shared by multiple
users, this can easily lead to saturation of 1 Gbit/s Ethernet links,
causing various problems and slowing down the simulation.

ROS [image_transport][] can compress camera frames as JPEGs or h264
stream (when using [ffmpeg_image_transport][]), which significantly
reduces the bandwidth.

## Features

- Publishes camera images in compressed form (JPEG, h264, etc.)
- Survives CARLA restarts, map changes and ego vehicle respawning.
- Camera position and other parameters are modifiable at runtime via
  ROS parameters.
- Publishes TF2 transformations to allow overlaying 3D objects over
  the video, e.g. in [Foxglove][].
- Tested with CARLA 0.9.15 and 0.10.0 and ROS Jazzy Jalisco.
- Limited compatibility with `rviz2` (unfortunately). Until
  [image_transport support is added][rviz_img_transport] `rviz2` can
  display only uncompressed images.

[Foxglove]: https://foxglove.dev/
[rviz_img_transport]: https://github.com/ros2/rviz/pull/1288

## Usage

Add the source code to a [ROS 2 workspace][ROS tutorial] and compile
it with:

    colcon build
    source install/setup.bash

Then run the node as follows:

    ros2 run carla_camera_publisher carla_camera_publisher localhost 2000 --ros-args -p topic:=/my_camera

If you run CARLA with non-default port, use it instead of 2000.

The node will wait for the ego vehicle, create a camera sensors and
start publishing images from the camera on the following topics
(their number and names can differ depending on image_transport
plugins you have available):

    /my_camera
    /my_camera/compressed
    /my_camera/compressedDepth
    /my_camera/ffmpeg
    /my_camera/foxglove
    /my_camera/theora
    /my_camera/zstd

To access the video from another host, run:

    export ROS_STATIC_PEERS=aa.bb.cc.dd  # IP address of the host running CARLA
    ros2 node list
    ros2 launch ...

If `ros2 node list` doesn't show `carla_camera_publisher`, try running
`ros2 daemon stop` and ensure that the ROS traffic is not blocked by a
firewall.

### Using with nix

If you use nix, instead of installing ROS and building the node with
`colcon`, you can build and run the node with the following command:

    nix run github:wentasah/carla_camera_publisher -- localhost 2000

### ROS parameters

ROS parameters and their default values are listed below:

<!-- `$ nix run .#dump-parameters` as yaml -->

```yaml
ego_vehicle_role_name: ego_vehicle
fov: 110.0
frame_id: carla_camera
height: 360
orientation:
  pitch: -10.0
  roll: 0.0
  yaw: 0.0
position:
  x: 0.0
  y: 0.0
  z: 2.0
sensor_tick: 0.04
start_type_description_service: true
topic: /carla_camera_publisher/camera
width: 640
```

### Streaming to Foxglove Studio

If you install [foxglove_compressed_video_transport][] (at least
version 1.0.2 is needed), you can stream hardware-accelerated video
directly to Foxglove. The following parameters give good results for
me with an NVIDIA GPU:

    ros2 run carla_camera_publisher carla_camera_publisher localhost 2000 --ros-args \
        -p width:=1920 -p height:=1080 \
        -p carla_camera_publisher.camera.foxglove.encoding:=h264_nvenc \
        -p carla_camera_publisher.camera.foxglove.preset:=ll \
        -p carla_camera_publisher.camera.foxglove.bit_rate:=30000000

To see the video in Foxglove run:

    export ROS_STATIC_PEERS=aa.bb.cc.dd  # IP address of the host running CARLA
    ros2 run foxglove_bridge foxglove_bridge

and connect Foxglove to `ws://localhost:8765`.

## Example

Comparison of camera visualization from CARLA running on a remote host
(left: this project + Foxglove; right: native CARLA protocol). The
video also shows changing of camera position via ROS parameters.

https://github.com/user-attachments/assets/be466116-8fb5-4e0b-b9a9-a912821238a6

[ffmpeg_image_transport]: https://github.com/ros-misc-utilities/ffmpeg_image_transport/
[foxglove_compressed_video_transport]: https://github.com/ros-misc-utilities/foxglove_compressed_video_transport/
[image_transport]: https://wiki.ros.org/image_transport
[ROS tutorial]: https://docs.ros.org/en/jazzy/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html
