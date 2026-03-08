cmake_minimum_required(VERSION 3.8)
project(hexapod_bringup)

# 1. Bağımlılıkları Bulma
find_package(ament_cmake REQUIRED)

# 2. Yükleme (Install) Kuralları
# Launch klasörünü yüklüyoruz
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

# Yapılandırma XML dosyasını yüklüyoruz (eğer hala gerekliyse)
if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/hexapod_bringup.xml")
  install(FILES hexapod_bringup.xml
    DESTINATION share/${PROJECT_NAME}
  )
endif()

# 3. Paketleme Makrosu
ament_package()