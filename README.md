# CARLA camera publisher

C++ ROS node for publishing camera images via ROS image transport.

Using this node solves the problem of high bandwidth requirements
between CARLA server and client when they run on different hosts. For
example, a camera producing 640x360 images at 50 FPS requires
bandwidth around 400 Mbit/s. This can easily lead to saturation of
1 Gbit/s Ethernet links, causing various problems and slowing down the
simulation.

ROS image transport can compress the images as JPEGs or h264 stream
(when using `ffmpeg_image_transport`), which significantly reduces the
bandwidth. Theoretically, one can also use
`foxglove_compressed_video_transport` to show h264 stream in Foxglove,
but it seems that the current version of that transport is buggy and
crashes.

## Usage

Run the node as follows:

    ros2 run carla_camera_publisher carla_camera_publisher $CARLA_HOST $CARLA_PORT --ros-args -p topic:=/my_camera

This will publish the following topics (their number and names can
differ depending on image_transport plugins you have available):

    /my_camera
    /my_camera/compressed
    /my_camera/compressedDepth
    /my_camera/ffmpeg
    /my_camera/foxglove
    /my_camera/theora
    /my_camera/zstd
