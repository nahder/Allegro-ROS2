import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from control_hand.srv import Move

#CURRENT GESTURES: 0: rock, 1: paper, 2: scissors

class AllegroGame(Node):
    
    def __init__(self):
        super().__init__('allegro_game')
        
        self.timer = self.create_timer(0.01, self.timer_cb)
        self.gesture_sub = self.create_subscription(String, 'gesture', self.player_gesture_cb, 10) 
        self.move_client = self.create_client(Move, 'controller/move')
        self.reset_client = self.create_client(Trigger, 'controller/reset') 
        self.round = 0


    def move(self, config_idx):
        request = Move.Request()
        request.config_idx = config_idx
        self.move_client.call_async(request)
        
    def robot_gesture_generator(self):
        pass 
                
            
    def player_gesture_cb(self, msg):
        pass 
    
    
    def timer_cb(self):
        #after the player has done the same gestures, increment round and let the timer then move on to generate new gestures 
        pass         
        
def main(args=None):
    rclpy.init(args=args)
    allegro_game = AllegroGame()
    rclpy.spin(allegro_game)
    allegro_game.destroy_node()
    rclpy.shutdown()