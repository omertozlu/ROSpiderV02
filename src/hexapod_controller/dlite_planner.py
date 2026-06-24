import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped
import math
import numpy as np
import heapq
import time  
import os    

class GolemDStarLitePlanner(Node):
    def __init__(self):
        super().__init__('golem_dstar_lite_planner')
        
        # Abonelikler
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)

        # Harita Ayarları
        self.res = 0.1
        self.width = 200 
        self.grid = np.zeros((self.width, self.width))
        self.offset = 10.0 

        # Stratejik Parametreler
        self.inflation_cells = 5
        self.max_view_dist = 6.0
        self.target = (4.0, 4.0)
        self.pos = [0.0, 0.0, 0.0] 
        
        self.last_path_time = 0.0
        self.path_update_interval = 1.0 
        self.current_path = []

        # METRİKLER VE SAYAÇLAR
        self.mission_start_time = 0.0
        self.goal_reached = False
        
        # Gerçek Gidilen Yol
        self.gercek_gidilen_yol = 0.0 
        self.eski_konum = None

        self.get_logger().info("GOLEM D* LITE (Geriye Doğru Arama Modu) Hazır.")

    def world_to_grid(self, x, y):
        gx = int((x + self.offset) / self.res)
        gy = int((y + self.offset) / self.res)
        return np.clip(gx, 0, self.width-1), np.clip(gy, 0, self.width-1)

    def scan_cb(self, msg):
        angle = msg.angle_min
        for r in msg.ranges:
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
        
        if self.mission_start_time == 0.0:
            self.mission_start_time = current_time

        if self.eski_konum is not None and not self.goal_reached:
            dx = p.x - self.eski_konum[0]
            dy = p.y - self.eski_konum[1]
            self.gercek_gidilen_yol += math.hypot(dx, dy)
        
        self.eski_konum = (p.x, p.y)

        if (current_time - self.last_path_time > self.path_update_interval) or len(self.current_path) == 0:
            if not self.goal_reached:
                self.replan()
            self.last_path_time = current_time
        
        self.execute_drive()

    def replan(self):
        start = self.world_to_grid(self.pos[0], self.pos[1])
        goal = self.world_to_grid(self.target[0], self.target[1])
        
        for i in range(-3, 4): 
            for j in range(-3, 4):
                cx, cy = start[0]+i, start[1]+j
                if 0 <= cx < self.width and 0 <= cy < self.width:
                    self.grid[cx, cy] = 0

        # ---- KRONOMETRE BAŞLANGICI ----
        start_time = time.perf_counter()
        
        # Algoritma çağrısı D* Lite olarak değişti
        path, nodes_visited = self.dstar_lite_core(start, goal)
        
        # ---- KRONOMETRE BİTİŞİ ----
        end_time = time.perf_counter()
        
        computation_time_ms = (end_time - start_time) * 1000
        
        path_length_m = 0.0
        if path and len(path) > 1:
            for i in range(1, len(path)):
                dx = path[i][0] - path[i-1][0]
                dy = path[i][1] - path[i-1][1]
                path_length_m += math.hypot(dx, dy) * self.res

        self.get_logger().info(f"Süre: {computation_time_ms:.2f}ms | İncelenen Düğüm: {nodes_visited} | Planlanan Rota: {path_length_m:.2f}m")

        try:
            file_name = 'algoritma_karsilastirma.csv'
            file_exists = os.path.isfile(file_name)
            
            with open(file_name, 'a') as f:
                if not file_exists:
                    f.write("Algoritma,Hesaplama_Suresi_ms,Ziyaret_Edilen_Dugum,Yol_Uzunlugu_m\n")
                
                # İsim D* Lite olarak kaydedilir
                f.write(f"D* Lite,{computation_time_ms:.4f},{nodes_visited},{path_length_m:.4f}\n")
        except Exception as e:
            self.get_logger().error(f"Veri kaydedilemedi: {e}")

        if path:
            self.current_path = path
            self.publish_path(path)
        else:
            self.get_logger().warn("Rota hesaplanamadı! Önü kapalı olabilir.")

    def execute_drive(self):
        t = Twist()
        dist_to_goal = math.sqrt((self.target[0]-self.pos[0])**2 + (self.target[1]-self.pos[1])**2)
        
        if dist_to_goal < 0.4:
            if not self.goal_reached:
                end_time = self.get_clock().now().nanoseconds / 1e9
                total_mission_time = end_time - self.mission_start_time
                
                if total_mission_time > 0:
                    ortalama_hiz = self.gercek_gidilen_yol / total_mission_time
                else:
                    ortalama_hiz = 0.0
                
                self.get_logger().info("\n" + "="*40 + 
                                       "\n🏆 HEDEFE BAŞARIYLA VARILDI!" +
                                       f"\n⏱️ Toplam Seyahat Süresi: {total_mission_time:.2f} Saniye" +
                                       f"\n📏 Gerçekte Gidilen Yol : {self.gercek_gidilen_yol:.2f} Metre" +
                                       f"\n🚀 Ortalama Hareket Hızı: {ortalama_hiz:.3f} m/s\n" +
                                       "="*40)
                self.goal_reached = True

            t.linear.x = 0.0; t.angular.z = 0.0
            self.cmd_pub.publish(t)
            return

        if self.current_path and len(self.current_path) > 1:
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
            t.linear.x = 0.0
            t.angular.z = 0.0 

        self.cmd_pub.publish(t)

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "base_link" 
        yaw = self.pos[2]
        for pt in path:
            p = PoseStamped()
            dx = ((pt[0]*self.res)-self.offset) - self.pos[0]
            dy = ((pt[1]*self.res)-self.offset) - self.pos[1]
            p.pose.position.x = dx*math.cos(-yaw) - dy*math.sin(-yaw)
            p.pose.position.y = dx*math.sin(-yaw) + dy*math.cos(-yaw)
            path_msg.poses.append(p)
        self.path_pub.publish(path_msg)

    # ==========================================
    # --- D* LITE ÇEKİRDEK MİMARİSİ (GERİYE DOĞRU ARAMA) ---
    # ==========================================
    def dstar_lite_core(self, start, goal):
        neighbors = [(0,1),(0,-1),(1,0),(-1,0),(1,1),(1,-1),(-1,1),(-1,-1)]
        close_set = set()
        came_from = {}
        
        # DİKKAT: D* Lite aramaya HEDEFTEN (goal) başlar!
        gscore = {goal: 0} 
        fscore = {goal: self.dist(goal, start)}
        
        oheap = []
        heapq.heappush(oheap, (fscore[goal], goal))
        
        nodes_visited = 0 
        
        while oheap:
            current = heapq.heappop(oheap)[1]
            nodes_visited += 1 
            
            # Başlangıç noktasına (robotun olduğu yere) ulaştığında arama biter.
            if current == start:
                data = []
                # Ağacı Baştan Sona doğru kurar
                while current in came_from:
                    data.append(current)
                    current = came_from[current]
                data.append(goal)
                return data, nodes_visited
                
            close_set.add(current)
            for i, j in neighbors:
                neighbor = (current[0]+i, current[1]+j)
                
                if 0 <= neighbor[0] < self.width and 0 <= neighbor[1] < self.width:
                    if self.grid[neighbor[0], neighbor[1]] == 1: continue 
                else: 
                    continue
                
                if neighbor in close_set: continue
                
                tg = gscore[current] + self.dist(current, neighbor)
                
                if tg < gscore.get(neighbor, float('inf')):
                    came_from[neighbor] = current
                    gscore[neighbor] = tg
                    # Sezgisel yaklaşım (Heuristic) robotun olduğu yere (start) doğru hesaplanır
                    fscore[neighbor] = tg + self.dist(neighbor, start)
                    heapq.heappush(oheap, (fscore[neighbor], neighbor))
                    
        return None, nodes_visited

    def dist(self, a, b):
        return math.sqrt((b[0]-a[0])**2 + (b[1]-a[1])**2)

def main():
    rclpy.init(); rclpy.spin(GolemDStarLitePlanner()); rclpy.shutdown()

if __name__ == '__main__': main()