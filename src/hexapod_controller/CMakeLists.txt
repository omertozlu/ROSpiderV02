cmake_minimum_required(VERSION 3.8)
project(hexapod_controller)

# C++ 17 zorunluluktur.
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# 1. Bağımlılıkları Bulma
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED) # <--- Kör noktan: Servisler için şart!
find_package(hexapod_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

# Bağımlılıkları bir listede toplayarak yönetmek rasyoneldir
set(HEXAPOD_DEPENDENCIES
  rclcpp
  geometry_msgs
  sensor_msgs
  std_msgs
  std_srvs
  hexapod_msgs
  nav_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  dynamixel_sdk
)

# 2. Dahili Kütüphaneleri Oluşturma
# Her kütüphaneyi tek tek tanımlamak yerine bir döngü veya toplu include kullanılabilir
# ama şimdilik senin yapını modernize edelim.
add_library(control_lib SHARED src/control.cpp)
add_library(gait_lib SHARED src/gait.cpp)
add_library(ik_lib SHARED src/ik.cpp)
add_library(servo_driver_lib SHARED src/servo_driver.cpp)

# 3. Header Yollarını Tanımlama
# PUBLIC kullanarak bu kütüphaneyi kullananların da headerları görmesini sağlıyoruz
foreach(lib control_lib gait_lib ik_lib servo_driver_lib)
  target_include_directories(${lib} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )
  ament_target_dependencies(${lib} ${HEXAPOD_DEPENDENCIES})
endforeach()

# 4. Ana Executable (Düğüm)
add_executable(hexapod_controller_node src/hexapod_controller.cpp)
target_link_libraries(hexapod_controller_node
  control_lib
  gait_lib
  ik_lib
  servo_driver_lib
)
ament_target_dependencies(hexapod_controller_node ${HEXAPOD_DEPENDENCIES})

# 5. Yükleme (Install) Kuralları - Burası kritik!
install(TARGETS 
  control_lib gait_lib ik_lib servo_driver_lib hexapod_controller_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

# Include klasörünü düzgünce yükle
install(DIRECTORY include/
  DESTINATION include
)

# Launch dosyalarını share altına taşı
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# 6. Export İşlemleri (Diğer paketlerin bu kütüphaneleri görebilmesi için)
ament_export_include_directories(include)
ament_export_libraries(control_lib gait_lib ik_lib servo_driver_lib)
ament_export_dependencies(${HEXAPOD_DEPENDENCIES})

ament_package()