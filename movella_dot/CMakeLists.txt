cmake_minimum_required(VERSION 3.8)
project(movella_dot)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(movella_msgs REQUIRED)

# Paths for movella dot libraries and xdpchandler
set(MOVELLA_DIR "/usr/local/movella")
set(MOVELLALIB_DIR "/usr/local/movella/lib")
set(MOVELLAEX_DIR "/usr/local/movella/examples")


## add executables
add_executable(movella_dot src/movella_dot.cpp)

#Include
target_include_directories(movella_dot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${MOVELLAEX_DIR}/xdpcsdk/cpp
  ${MOVELLA_DIR}/include/
)
# Linking
target_link_libraries(movella_dot
  ${MOVELLALIB_DIR}/libmovelladot_pc_sdk.so
  ${MOVELLALIB_DIR}/libxstypes.so.2022
  ${MOVELLAEX_DIR}/xdpcsdk/cpp/xdpchandler.cpp.o
  ${MOVELLAEX_DIR}/xdpcsdk/cpp/conio.c
)

## Linking pthreads
set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)
target_link_libraries(movella_dot PRIVATE Threads::Threads)

target_compile_features(movella_dot PUBLIC c_std_99 cxx_std_17) # Require C99, C++17
ament_target_dependencies(
  movella_dot
  "rclcpp"
  "std_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "movella_msgs"
)

# Installing
install(TARGETS  movella_dot
  DESTINATION lib/${PROJECT_NAME}
)

# Installs the directories
install(DIRECTORY launch
	DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()