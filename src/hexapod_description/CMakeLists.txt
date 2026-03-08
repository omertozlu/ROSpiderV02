cmake_minimum_required(VERSION 3.8)
project(hexapod_description)

# 1. Bağımlılıkları Bulma (catkin yerine ament_cmake)
find_package(ament_cmake REQUIRED)

# 2. Yükleme (Install) Kuralları
# Robotun 3D modellerini (meshes) ve URDF dosyalarını sisteme yüklüyoruz.
# ROS 2'de bunlar 'share' dizini altına gider.
# Dosyaları share dizinine kopyala (Hatanın çözümü burası)
install(DIRECTORY meshes urdf launch rviz
  DESTINATION share/${PROJECT_NAME}

)

# 3. Paketleme Makrosu (ROS 2 için zorunlu)
ament_package()
