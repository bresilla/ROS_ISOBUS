cmake_minimum_required(VERSION 3.16)
project(ros_isobus)

include(FetchContent)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


FetchContent_Declare(fmt
  GIT_REPOSITORY https://github.com/fmtlib/fmt
)
FetchContent_MakeAvailable(fmt)

FetchContent_Declare(AgIsoStack
   GIT_REPOSITORY https://github.com/Open-Agriculture/AgIsoStack-plus-plus.git
   GIT_TAG        main
)
FetchContent_MakeAvailable(AgIsoStack)
# add_subdirectory("ext/AgIsoStack-plus-plus")

include_directories(
  include
  include/cantroller/
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
)

find_package(Threads)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(fmt REQUIRED)
find_package(rosidl_default_generators REQUIRED)

set(ros_dependencies
  rclcpp
  std_msgs
  std_srvs
  lifecycle_msgs
  rclcpp_lifecycle
  sensor_msgs
  cv_bridge
  image_transport
  OpenCV
)

set(other_dependencies
  fmt::fmt
  isobus::Isobus
  isobus::HardwareIntegration
  isobus::Utility
  Threads::Threads
)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/Param.srv"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

add_executable(cantest src/cantest.cpp)
target_link_libraries(cantest ${other_dependencies} "${cpp_typesupport_target}")
ament_target_dependencies(cantest ${ros_dependencies})

install(TARGETS
  cantest
  DESTINATION lib/${PROJECT_NAME}
)

install(
    DIRECTORY include/
    DESTINATION include
)

install(
    DIRECTORY config
    DESTINATION share/${PROJECT_NAME}
)

install(
    DIRECTORY launch
    DESTINATION share/${PROJECT_NAME}
)

ament_package()
