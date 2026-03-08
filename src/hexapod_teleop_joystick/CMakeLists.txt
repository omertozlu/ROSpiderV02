cmake_minimum_required(VERSION 3.8)
project(hexapod_teleop_joystick)

# C++ 17 Standardı
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Bağımlılıkları Bulma (catkin YERİNE ament_cmake)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hexapod_msgs REQUIRED) # Yeni derlediğin mesaj paketi
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# 2. Executable Oluşturma
add_executable(hexapod_teleop_joystick src/hexapod_teleop_joystick.cpp)

# Header dosyalarını dahil etme
target_include_directories(hexapod_teleop_joystick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# 3. Bağımlılıkları Bağlama
ament_target_dependencies(hexapod_teleop_joystick
  rclcpp
  sensor_msgs
  hexapod_msgs
  geometry_msgs
  std_msgs
)

# 4. Yükleme (Install) Kuralları
install(TARGETS hexapod_teleop_joystick
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

# 5. Paketleme Makrosu (catkin_package YERİNE)
ament_package()