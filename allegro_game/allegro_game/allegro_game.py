import rclpy
from rclpy.node import Node

class AllegroGame(Node):
    
    def __init__(self):
        super().__init__('allegro_game')
        
        self.timer = self.create_timer(0.5, self.timer_cb)
        
        
    def timer_cb(self):
        self.get_logger().info('Hello World')
        
        
        
def main(args=None):
    rclpy.init(args=args)
    allegro_game = AllegroGame()
    rclpy.spin(allegro_game)
    allegro_game.destroy_node()
    rclpy.shutdown()