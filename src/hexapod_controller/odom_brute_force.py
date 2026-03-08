import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import numpy as np

class GolemMasterPlanner(Node):
    def __init__(self):
        super().__init__('golem_master_planner')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # GRID Ayarları (Daha geniş alan, daha sıkı kontrol)
        self.res = 0.2
        self.grid_size = 100 # 20m x 20m alan
        self.grid = np.zeros((self.grid_size, self.grid_size))
        
        self.target_x, self.target_y = 10.0, 0.0
        self.curr_x, self.curr_y, self.curr_yaw = 0.0, 0.0, 0.0

    def world_to_grid(self, wx, wy):
        gx = int((wx + 10.0) / self.res)
        gy = int((wy + 10.0) / self.res)
        return gx, gy

    def scan_cb(self, msg):
        angle = msg.angle_min
        for r in msg.ranges:
            if 0.4 < r < 3.5:
                ox = self.curr_x + r * math.cos(angle + self.curr_yaw)
                oy = self.curr_y + r * math.sin(angle + self.curr_yaw)
                gx, gy = self.world_to_grid(ox, oy)
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    # INFLATION: Engeli ve etrafını kapat (Robotun genişliği için)
                    self.grid[max(0, gx-1):min(self.grid_size, gx+2), 
                              max(0, gy-1):min(self.grid_size, gy+2)] = 1
            angle += msg.angle_increment

    def check_corridor(self, distance, width_steps=2):
        """ Robotun önünde belirli bir mesafe boyunca koridor temiz mi? """
        for d in np.arange(0.2, distance, self.res):
            for w in range(-width_steps, width_steps + 1):
                # Robotun merkezinden sağa sola kaydırarak kontrol et
                check_x = self.curr_x + d * math.cos(self.curr_yaw) - (w * self.res * math.sin(self.curr_yaw))
                check_y = self.curr_y + d * math.sin(self.curr_yaw) + (w * self.res * math.cos(self.curr_yaw))
                gx, gy = self.world_to_grid(check_x, check_y)
                if 0 <= gx < self.grid_size and 0 <= gy < self.grid_size:
                    if self.grid[gx, gy] == 1:
                        return False
        return True

    def odom_cb(self, m):
        self.curr_x, self.curr_y = m.pose.pose.position.x, m.pose.pose.position.y
        q = m.pose.pose.orientation
        self.curr_yaw = math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))
        self.control_loop()

    def control_loop(self):
        t = Twist()
        dist = math.sqrt((self.target_x - self.curr_x)**2 + (self.target_y - self.curr_y)**2)
        angle_to_goal = math.atan2(self.target_y - self.curr_y, self.target_x - self.curr_x)
        err = math.atan2(math.sin(angle_to_goal - self.curr_yaw), math.cos(angle_to_goal - self.curr_yaw))

        if dist < 0.4:
            self.get_logger().info("HEDEFE VARILDI!")
            return

        # ÖNÜMÜZDEKİ 1.2 METRELİK KORİDORU KONTROL ET
        is_path_clear = self.check_corridor(1.2)

        if not is_path_clear:
            # Engel varsa: Hedef açısına göre değil, engelin olmadığı yöne dön
            t.linear.x = 0.0
            # Eğer sağ taraf daha boşsa sağa, değilse sola dön (Basit yönelme)
            t.angular.z = 0.6 
            self.get_logger().warn("KORİDOR KAPALI! Güvenli alan aranıyor...")
        else:
            # Yol temizse hedefe yönel
            if abs(err) > 0.2:
                t.angular.z = 0.4 if err > 0 else -0.4
            else:
                t.linear.x = 0.4
                t.angular.z = err * 0.5
        
        self.cmd_pub.publish(t)

def main():
    rclpy.init(); rclpy.spin(GolemMasterPlanner()); rclpy.shutdown()

if __name__ == '__main__': main()