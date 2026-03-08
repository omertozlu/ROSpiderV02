import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np
import heapq

class GolemStrategicPlanner(Node):
    def __init__(self):
        super().__init__('golem_strategic_planner')
        
        # Abonelikler
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # --- DÜZELTİLEN HARİTA AYARLARI ---
        self.res = 0.1
        self.width = 200 # Haritayı 100'den 200'e çıkardık (20x20 metre alan)
        self.grid = np.zeros((self.width, self.width))
        self.offset = 10.0 # Artık (0,0) noktası tam 100. kareye (Merkeze) denk geliyor.

        # --- STRATEJİK PARAMETRELER ---
        self.inflation_cells = 5
        self.max_view_dist = 6.0
        self.target = (4.0, 4.0)
        self.pos = [0.0, 0.0, 0.0] 
        
        self.last_path_time = 0.0
        self.path_update_interval = 1.0 # Tepki süresini hızlandırdık (2sn -> 1sn)
        self.current_path = []

        self.get_logger().info("GOLEM: Harita genişletildi. Merkezleme yapıldı.")

    def world_to_grid(self, x, y):
        gx = int((x + self.offset) / self.res)
        gy = int((y + self.offset) / self.res)
        return np.clip(gx, 0, self.width-1), np.clip(gy, 0, self.width-1)

    def scan_cb(self, msg):
        angle = msg.angle_min
        for r in msg.ranges:
            # R > 0.60 FİLTRESİ: Robotun bacaklarını haritaya yazmasını engeller
            if not math.isnan(r) and not math.isinf(r) and r < self.max_view_dist and r > 0.60:
                ox = self.pos[0] + r * math.cos(angle + self.pos[2])
                oy = self.pos[1] + r * math.sin(angle + self.pos[2])
                gx, gy = self.world_to_grid(ox, oy)
                
                for i in range(-self.inflation_cells, self.inflation_cells + 1):
                    for j in range(-self.inflation_cells, self.inflation_cells + 1):
                        ix, iy = gx + i, gy + j
                        if 0 <= ix < self.width and 0 <= iy < self.width:
                            self.grid[ix, iy] = 1 
            angle += msg.angle_increment

    def odom_cb(self, m):
        p, q = m.pose.pose.position, m.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pos = [p.x, p.y, yaw]
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        if (current_time - self.last_path_time > self.path_update_interval) or len(self.current_path) == 0:
            self.replan()
            self.last_path_time = current_time
        
        self.execute_drive()

    def replan(self):
        start = self.world_to_grid(self.pos[0], self.pos[1])
        goal = self.world_to_grid(self.target[0], self.target[1])
        
        # ROBOTUN KENDİ ALANINI TEMİZLE (Hayalet engel sorunu çözümü)
        for i in range(-3, 4): # 7x7 bir alanı (70cm) temizle, garanti olsun
            for j in range(-3, 4):
                cx, cy = start[0]+i, start[1]+j
                if 0 <= cx < self.width and 0 <= cy < self.width:
                    self.grid[cx, cy] = 0

        path = self.astar(start, goal)
        if path:
            self.current_path = path
            self.publish_path(path)
        else:
            self.get_logger().warn("Rota hesaplanamadı! Önü kapalı olabilir.")

    def execute_drive(self):
        t = Twist()
        dist_to_goal = math.sqrt((self.target[0]-self.pos[0])**2 + (self.target[1]-self.pos[1])**2)
        
        if dist_to_goal < 0.4:
            self.get_logger().info("HEDEFE VARILDI!")
            t.linear.x = 0.0; t.angular.z = 0.0
            self.cmd_pub.publish(t)
            return

        # DÜZELTME: Rota 5'ten kısaysa bile hareket etmeli!
        if self.current_path and len(self.current_path) > 1:
            # Yol uzunsa ileriye bak, kısaysa son noktaya bak
            lookahead = min(5, len(self.current_path)-1)
            target_pt = self.current_path[lookahead]
            
            wx = (target_pt[0] * self.res) - self.offset
            wy = (target_pt[1] * self.res) - self.offset
            
            angle_to_goal = math.atan2(wy - self.pos[1], wx - self.pos[0])
            err = math.atan2(math.sin(angle_to_goal - self.pos[2]), math.cos(angle_to_goal - self.pos[2]))
            
            if abs(err) > 0.4:
                t.angular.z = 0.8 if err > 0 else -0.8
                t.linear.x = 0.0
            else:
                t.linear.x = 0.35 
                t.angular.z = err * 1.5
        else:
            # Rota yoksa veya çok kısaysa dur, boşuna dönme
            t.linear.x = 0.0
            t.angular.z = 0.0 

        self.cmd_pub.publish(t)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "base_link" # RVIZ AYARINA DİKKAT: Burası 'odom' olmalı genelde ama senin setup'ına göre kalsın
        yaw = self.pos[2]
        for pt in path:
            p = PoseStamped()
            dx = ((pt[0]*self.res)-self.offset) - self.pos[0]
            dy = ((pt[1]*self.res)-self.offset) - self.pos[1]
            p.pose.position.x = dx*math.cos(-yaw) - dy*math.sin(-yaw)
            p.pose.position.y = dx*math.sin(-yaw) + dy*math.cos(-yaw)
            path_msg.poses.append(p)
        self.path_pub.publish(path_msg)

    def astar(self, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set(); came_from = {}; gscore = {start:0}
        fscore = {start: self.dist(start, goal)}
        oheap = []
        heapq.heappush(oheap, (fscore[start], start))
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            if current == goal:
                data = []
                while current in came_from:
                    data.append(current); current = came_from[current]
                return data[::-1]
            close_set.add(current)
            for i, j in neighbors:
                neighbor = (current[0]+i, current[1]+j)
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.width:
                    if self.grid[neighbor[0], neighbor[1]] == 1: continue
                else: continue
                if neighbor in close_set: continue
                
                tg = gscore[current] + self.dist(current, neighbor)
                if tg < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tg
                    fscore[neighbor] = tg + self.dist(neighbor, goal)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
        return None

    def dist(self, a, b):
        return math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)

def main():
    rclpy.init(); rclpy.spin(GolemStrategicPlanner()); rclpy.shutdown()

if __name__ == '__main__': main()