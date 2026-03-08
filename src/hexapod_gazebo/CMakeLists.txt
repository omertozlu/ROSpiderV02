cmake_minimum_required(VERSION 3.8)
project(hexapod_gazebo)

# 1. ROS 2 Derleme Sistemini ve Bağımlılıkları Bul
find_package(ament_cmake REQUIRED)

# 2. Dosyaları Sisteme Yükle (Install)
# Launch, Config ve (varsa) World klasörlerini yüklüyoruz
install(DIRECTORY
  config
  launch
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# 3. Paketleme Makrosu (Zorunlu)
ament_package()