cmake_minimum_required(VERSION 3.8)
project(carla_camera_publisher)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(fmt REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(carla_camera_publisher main.cpp)
set_property(TARGET carla_camera_publisher PROPERTY CXX_STANDARD 20)
target_link_libraries(carla_camera_publisher PRIVATE fmt::fmt)
# target_compile_options(carla_camera_publisher PUBLIC -fPIC -fsanitize=thread)
# target_link_options(carla_camera_publisher PUBLIC -fsanitize=thread)
# target_link_libraries(carla_camera_publisher PUBLIC Threads::Threads)
target_link_libraries(carla_camera_publisher PUBLIC
  -Wl,-Bstatic -lcarla_client -lrpc -Wl,-Bdynamic -lpng
  -ljpeg -lRecast -lDetour -lDetourCrowd -lboost_filesystem
  )


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
