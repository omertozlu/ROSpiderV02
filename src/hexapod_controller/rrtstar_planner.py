import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseStamped, Point, TransformStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA
from tf2_ros import TransformBroadcaster
import math
import numpy as np
import random
import time  
import os    

class TreeNode:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.cost = 0.0
        self.parent = None

class GolemRRTStarVisual(Node):
    def __init__(self):
        super().__init__('golem_rrt_star_visual')
        
        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/planned_path', 10)
        self.tree_pub = self.create_publisher(Marker, '/rrt_tree', 10)
        
        self.tf_broadcaster = TransformBroadcaster(self)

        self.res = 0.1
        self.width = 200 
        self.grid = np.zeros((self.width, self.width))
        self.offset = 10.0 

        self.inflation_cells = 5
        self.max_view_dist = 6.0
        self.target = (4.0, 4.0) 
        self.pos = [0.0, 0.0, 0.0] 
        
        self.last_path_time = 0.0
        self.path_update_interval = 2.0
        self.current_path = []

        self.max_iter = 2000       
        self.step_size = 0.4      
        self.search_radius = 1.0  
        self.goal_sample_rate = 5 

        # --- YENİ METRİKLER VE SAYAÇLAR ---
        self.mission_start_time = 0.0
        self.goal_reached = False
        self.gercek_gidilen_yol = 0.0 
        self.eski_konum = None

        self.get_logger().info("GOLEM RRT* (Kapsamlı Ölçüm ve Kilometre Sayacı Aktif) Hazır.")

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
        p = m.pose.pose.position
        q = m.pose.pose.orientation
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = p.x
        t.transform.translation.y = p.y
        t.transform.translation.z = p.z
        t.transform.rotation.x = q.x
        t.transform.rotation.y = q.y
        t.transform.rotation.z = q.z
        t.transform.rotation.w = q.w

        self.tf_broadcaster.sendTransform(t)

        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        self.pos = [p.x, p.y, yaw]
        
        current_time = self.get_clock().now().nanoseconds / 1e9
        
        # --- KİLOMETRE SAYACI MANTIĞI ---
        if self.mission_start_time == 0.0:
            self.mission_start_time = current_time

        if self.eski_konum is not None and not self.goal_reached:
            dx = p.x - self.eski_konum[0]
            dy = p.y - self.eski_konum[1]
            self.gercek_gidilen_yol += math.hypot(dx, dy)
        
        self.eski_konum = (p.x, p.y)
        # --------------------------------

        if (current_time - self.last_path_time > self.path_update_interval) or len(self.current_path) == 0:
            if not self.goal_reached:
                self.replan()
            self.last_path_time = current_time
        
        self.execute_drive()

    def replan(self):
        start_grid = self.world_to_grid(self.pos[0], self.pos[1])
        for i in range(-3, 4):
            for j in range(-3, 4):
                cx, cy = start_grid[0]+i, start_grid[1]+j
                if 0 <= cx < self.width and 0 <= cy < self.width:
                    self.grid[cx, cy] = 0

        # ---- KRONOMETRE BAŞLANGICI ----
        start_time = time.perf_counter()

        path, node_list = self.rrt_star(self.pos[:2], self.target)
        
        # ---- KRONOMETRE BİTİŞİ ----
        end_time = time.perf_counter()
        computation_time_ms = (end_time - start_time) * 1000
        
        # RRT* Ağacındaki toplam düğüm sayısını al
        nodes_visited = len(node_list)

        # Planlanan rota uzunluğunu hesapla (metre)
        path_length_m = 0.0
        if path and len(path) > 1:
            for i in range(1, len(path)):
                dx = path[i][0] - path[i-1][0]
                dy = path[i][1] - path[i-1][1]
                path_length_m += math.hypot(dx, dy) * self.res

        self.get_logger().info(f"Süre: {computation_time_ms:.2f}ms | Ağaçtaki Düğüm: {nodes_visited} | Planlanan Rota: {path_length_m:.2f}m")

        # --- CSV KAYIT ---
        try:
            file_name = 'algoritma_karsilastirma.csv'
            file_exists = os.path.isfile(file_name)
            
            with open(file_name, 'a') as f:
                if not file_exists:
                    f.write("Algoritma,Hesaplama_Suresi_ms,Ziyaret_Edilen_Dugum,Yol_Uzunlugu_m\n")
                
                # RRT* olarak kaydediyoruz
                f.write(f"RRT*,{computation_time_ms:.4f},{nodes_visited},{path_length_m:.4f}\n")
        except Exception as e:
            self.get_logger().error(f"Veri kaydedilemedi: {e}")

        # Görselleştirme
        self.publish_tree(node_list)

        if path:
            self.current_path = path
            self.publish_path(path) 
        else:
            self.get_logger().warn("RRT* Rota Bulamadı!")

    def rrt_star(self, start, goal):
        node_list = [TreeNode(start[0], start[1])]
        
        for i in range(self.max_iter):
            if random.randint(0, 100) > self.goal_sample_rate:
                rnd = [random.uniform(-10, 10), random.uniform(-10, 10)]
            else:
                rnd = [goal[0], goal[1]]

            nearest_ind = self.get_nearest_node_index(node_list, rnd)
            nearest_node = node_list[nearest_ind]

            theta = math.atan2(rnd[1] - nearest_node.y, rnd[0] - nearest_node.x)
            new_node = TreeNode(
                nearest_node.x + self.step_size * math.cos(theta),
                nearest_node.y + self.step_size * math.sin(theta)
            )
            new_node.cost = nearest_node.cost + self.step_size
            new_node.parent = nearest_ind

            if not self.check_collision(new_node.x, new_node.y):
                continue

            near_inds = self.find_near_nodes(node_list, new_node)
            for near_ind in near_inds:
                near_node = node_list[near_ind]
                d = math.sqrt((near_node.x - new_node.x)**2 + (near_node.y - new_node.y)**2)
                if near_node.cost + d < new_node.cost:
                    if self.check_collision_line(near_node, new_node):
                        new_node.parent = near_ind
                        new_node.cost = near_node.cost + d

            node_list.append(new_node)
            new_ind = len(node_list) - 1

            for near_ind in near_inds:
                near_node = node_list[near_ind]
                d = math.sqrt((new_node.x - near_node.x)**2 + (new_node.y - near_node.y)**2)
                if new_node.cost + d < near_node.cost:
                    if self.check_collision_line(new_node, near_node):
                        near_node.parent = new_ind
                        near_node.cost = new_node.cost + d

            dx = new_node.x - goal[0]
            dy = new_node.y - goal[1]
            if math.hypot(dx, dy) <= 0.5:
                goal_node = TreeNode(goal[0], goal[1])
                goal_node.parent = len(node_list) - 1
                goal_node.cost = new_node.cost + math.hypot(dx, dy)
                node_list.append(goal_node)
                return self.generate_final_course(len(node_list) - 1, node_list), node_list

        return None, node_list

    def publish_path(self, path):
        path_msg = Path()
        path_msg.header.frame_id = "odom" 
        
        for pt in path:
            p = PoseStamped()
            wx = (pt[0] * self.res) - self.offset
            wy = (pt[1] * self.res) - self.offset
            
            p.pose.position.x = wx
            p.pose.position.y = wy
            path_msg.poses.append(p)
            
        self.path_pub.publish(path_msg)

    def publish_tree(self, node_list):
        if not node_list: return
        
        marker = Marker()
        marker.header.frame_id = "odom" 
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "rrt_tree"
        marker.id = 0
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.02 
        marker.color = ColorRGBA(r=0.0, g=0.0, b=1.0, a=0.5) 

        for node in node_list:
            if node.parent is not None:
                p1 = Point(x=node.x, y=node.y, z=0.1)
                parent_node = node_list[node.parent]
                p2 = Point(x=parent_node.x, y=parent_node.y, z=0.1)
                marker.points.append(p1)
                marker.points.append(p2)
        
        self.tree_pub.publish(marker)

    def get_nearest_node_index(self, node_list, rnd):
        dlist = [(node.x - rnd[0])**2 + (node.y - rnd[1])**2 for node in node_list]
        return dlist.index(min(dlist))

    def find_near_nodes(self, node_list, new_node):
        nnode = len(node_list) + 1
        r = 50.0 * math.sqrt((math.log(nnode) / nnode))
        r = min(r, self.search_radius)
        dist_list = [(node.x - new_node.x)**2 + (node.y - new_node.y)**2 for node in node_list]
        near_inds = [dist_list.index(i) for i in dist_list if i <= r**2]
        return near_inds

    def check_collision(self, x, y):
        gx, gy = self.world_to_grid(x, y)
        if 0 <= gx < self.width and 0 <= gy < self.width:
            return self.grid[gx, gy] == 0
        return False

    def check_collision_line(self, node1, node2):
        steps = int(math.hypot(node2.x - node1.x, node2.y - node1.y) / (self.res))
        for i in range(steps):
            u = i / steps
            x = node1.x * (1 - u) + node2.x * u
            y = node1.y * (1 - u) + node2.y * u
            if not self.check_collision(x, y):
                return False
        return True

    def generate_final_course(self, goal_ind, node_list):
        path = []
        node = node_list[goal_ind]
        while node.parent is not None:
            gx, gy = self.world_to_grid(node.x, node.y)
            path.append((gx, gy))
            node = node_list[node.parent]
        path.append(self.world_to_grid(node.x, node.y))
        return path[::-1]

    def execute_drive(self):
        t = Twist()
        dist_to_goal = math.sqrt((self.target[0]-self.pos[0])**2 + (self.target[1]-self.pos[1])**2)
        
        if dist_to_goal < 0.4:
            if not self.goal_reached:
                end_time = self.get_clock().now().nanoseconds / 1e9
                total_mission_time = end_time - self.mission_start_time
                
                # --- ORTALAMA HIZ HESAPLAMASI ---
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
            lookahead = min(3, len(self.current_path)-1)
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

def main():
    rclpy.init(); rclpy.spin(GolemRRTStarVisual()); rclpy.shutdown()

if __name__ == '__main__': main()