import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist


class HexapodAutonomousWalker(Node):

    def __init__(self):
        super().__init__('autonomous_walker')

        self.publisher_ = self.create_publisher(
            Float64MultiArray,
            '/hexapod_joint_group_controller/commands',
            10
        )

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Zamanlayıcı (0.1 saniye)
        self.timer = self.create_timer(0.1, self.walk_cycle)

        self.phase = 0
        self.linear_x = 0.0
        self.angular_z = 0.0

    def cmd_vel_callback(self, msg):
        self.linear_x = msg.linear.x
        self.angular_z = msg.angular.z

    def walk_cycle(self):
        # Hareket yoksa dur
        if abs(self.linear_x) < 0.01 and abs(self.angular_z) < 0.01:
            return

        msg = Float64MultiArray()
        fwd_val = 0.5  # Dönüş ve ileri gitme miktarı

        # --- DÜZELTİLMİŞ DÖNÜŞ VE İLERİ MANTIĞI ---
        if abs(self.angular_z) > 0.1:
            # Sola dönüş (Pozitif z): Sağlar ileri (+), Sollar geri (-)
            # Sağa dönüş (Negatif z): Sağlar geri (-), Sollar ileri (+)
            fwd_R = fwd_val if self.angular_z > 0 else -fwd_val
            fwd_L = -fwd_val if self.angular_z > 0 else fwd_val
        else:
            # Normal İleri/Geri
            fwd_R = fwd_val if self.linear_x > 0 else -fwd_val
            fwd_L = fwd_val if self.linear_x > 0 else -fwd_val

        # Bacak açıları
        lift_R = [fwd_R, 0.1, -0.4]
        push_R = [-fwd_R, 0.7, -0.9]
        lift_L = [-fwd_L, 0.1, -0.4]
        push_L = [fwd_L, 0.7, -0.9]

        # Tripod Gait Fazları
        if self.phase == 0:
            msg.data = lift_R + push_L + push_R + lift_L + lift_R + push_L
            self.phase = 1
        else:
            msg.data = push_R + lift_L + lift_R + push_L + push_R + lift_L
            self.phase = 0

        self.publisher_.publish(msg)


def main():
    rclpy.init()
    node = HexapodAutonomousWalker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
