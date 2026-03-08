cmake_minimum_required(VERSION 3.8)
project(hexapod_msgs)

# C++ 17 standardını zorunlu kılıyoruz
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Bağımlılıkları Bulma
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

# 2. Mesaj Dosyalarını Tanımlama (add_message_files yerine)
set(msg_files
  "msg/LegJoints.msg"
  "msg/LegsJoints.msg"
  "msg/FeetPositions.msg"
  "msg/Pose.msg"
  "msg/RPY.msg"
  "msg/Sounds.msg"
)

# 3. Arayüzleri Oluşturma (generate_messages yerine)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${msg_files}
  DEPENDENCIES std_msgs geometry_msgs
)

# 4. Paketleme
ament_package()