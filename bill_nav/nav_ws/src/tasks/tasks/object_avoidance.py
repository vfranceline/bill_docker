# The robot will use a LIDAR sensor to detect obstacles and navigate around them by adjusting its movement.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import String 

class ObjectAvoidanceNode(Node):
    def __init__(self):
        super().__init__('object_avoidance_node')
        
        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.lidar_callback,
            10)
        
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.safe_distance = 0.5  # Meters
        self.get_logger().info('Object Avoidance Node Started')

        self.velocity_msg = Twist()
        self.elapsed_time = 0.0  
        self.duration = 1.0  # Tempo de rotação
        self.is_rotating = False

        self.timer = None  # Timer para publicar a velocidade
        self.countdown_timer = None  # Timer para a contagem regressiva
        self.countdown_value = 10  # Tempo de espera antes da rotação

    def lidar_callback(self, msg):
        ranges = msg.ranges
        min_distance = min(ranges)
        
        twist_msg = Twist()

        if min_distance < self.safe_distance:
            # Obstacle detected, stop the robot
            twist_msg.linear.x = 0.0
        else:
            # No obstacle detected, move forward
            twist_msg.linear.x = 0.2
            twist_msg.angular.z = 0.0

        self.publisher.publish(twist_msg)

    def countdown(self):
        if self.countdown_value > 0:
            self.get_logger().info(f'Contagem regressiva: {self.countdown_value}')
            self.countdown_value -= 1
        else:
            self.get_logger().info('Iniciando rotação de 180 graus.')
            self.countdown_timer.cancel()  # Cancela o timer da contagem regressiva

            # Configura a velocidade angular para girar
            self.velocity_msg.angular.z = 0.5   

            # Cria um timer para publicar a velocidade
            self.timer = self.create_timer(0.1, self.publish_velocity)
    
    def publish_velocity(self):
        if self.elapsed_time < self.duration:
            self.publisher_.publish(self.velocity_msg)
            self.elapsed_time += 0.1
        else:
            self.velocity_msg.angular.z = 0.0  # Para a rotação
            self.publisher_.publish(self.velocity_msg)
            
            self.timer.cancel()  # Para o timer
            self.is_rotating = False  # Permite futuras ativações

        twist_msg = Twist()
        self.publisher.publish(twist_msg)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectAvoidanceNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()