cmake_minimum_required(VERSION 3.8)
project(hexapod_navigation)

# 1. ROS 2 Derleme Sistemini Bul
find_package(ament_cmake REQUIRED)

# 2. Dosyaları Sisteme Yükle (Install)
# Config, Launch ve Maps klasörlerini 'share' dizinine kopyalıyoruz
install(DIRECTORY config launch maps
  DESTINATION share/${PROJECT_NAME}
)

# 3. Paketleme Makrosu (Zorunlu)
ament_package()