#!/bin/bash

# Dosya yolunu değişkene atayalım (Daha düzenli durur)
WORLD_PATH="/root/colcon_ws/src/hexapod_gazebo/worlds/maze.world"

if [ ! -d "/root/colcon_ws/install" ]; then
    echo "Install klasörü bulunamadı, derleme yapılıyor..."
    cd /root/colcon_ws && colcon build
fi

# Hata oluştuğu an scripti durdur
set -e

echo "--- Golem Başlatma Protokolü Aktif ---"

# 1. Temizlik
pkill -9 gzserver || true
pkill -9 gzclient || true
pkill -9 robot_state_publisher || true

source /opt/ros/humble/setup.bash
source /root/colcon_ws/install/setup.bash

# 2. Gazebo Server (Buraya dünyayı ekledik!)
echo "--- Gazebo Labirent Dünyası ile Başlatılıyor ---"
ros2 launch gazebo_ros gzserver.launch.py world:=$WORLD_PATH &
sleep 5

# 3. Robot State Publisher
ros2 run robot_state_publisher robot_state_publisher --ros-args -p robot_description:="$(xacro /root/colcon_ws/src/hexapod_description/urdf/hexapod_model.xacro)" &
sleep 2

# 4. Robotu Spawn Et
echo "--- Robot spawn ediliyor ---"
# Yeni Hali:
if ! ros2 run gazebo_ros spawn_entity.py -topic robot_description -entity Golem -x -3.0 -y -4.0 -z 0.5; then    echo "HATA: Robot spawn edilemedi! Durduruluyor."
    exit 1
fi

sleep 5

# 5. Kontrolcüler
echo "--- Kontrolcüler yükleniyor ---"
ros2 run controller_manager spawner joint_state_broadcaster || { echo "Broadcaster hatası!"; exit 1; }
ros2 run controller_manager spawner hexapod_joint_group_controller || { echo "Controller hatası!"; exit 1; }

# 6. Gazebo Client
export DISPLAY=:1
export QT_X11_NO_MITSHM=1
export __NV_PRIME_RENDER_OFFLOAD=1
export __GLX_VENDOR_LIBRARY_NAME=nvidia
gzclient