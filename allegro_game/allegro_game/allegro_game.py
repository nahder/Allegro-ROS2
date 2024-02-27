import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from control_hand.srv import Move
import numpy as np 
from enum import Enum, auto
from std_srvs.srv import Empty
import time 
import asyncio
class State(Enum):
    # NEUTRAL = auto()
    IN_SEQUENCE = auto() #if the robot has gestures left
    FINISHED_SEQUENCE = auto() #if the robot has no gestures left
    
#CURRENT GESTURES: 0: rock, 1: paper, 2: scissors,(3: okay,  9: nonono) 
class AllegroGame(Node):
    def __init__(self):
        super().__init__('allegro_game')
        
        self.gesture_sub = self.create_subscription(String, 'gesture', self.player_gesture_cb, 10) 
        self.move_client = self.create_client(Move, 'controller/move')
        self.reset_client = self.create_client(Trigger, 'controller/reset') 
        self.delay_client = self.create_client(Empty, 'delay')
        
        gesture_dict = {0: "rock", 1: "paper", 2: "scissors"}
        
        self.player_gesture = None
        self.moves = []
        self.round = 5

        self.game_loop()
    # Performs all the gestures in the moves list
    # Send a reset move in between every gesture (config_idx = -100)
    
    def game_loop(self):
        request = Move.Request()
        self.moves = [-100, 0, -100, 1, -100, 2]
        
        for move in self.moves:
            request.config_idx = move
            self.move_client.call_async(request)
            self.delay()
            
    def delay(self): 
        self.future = self.delay_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
                
    def player_gesture_cb(self, msg):
        self.player_gesture = msg.data 
        self.get_logger().info('Player did: %s' % self.player_gesture)
        
    def demo(self): 
        # Give robot a gesture
        request = Move.Request()
        request.config_idx = 1
        self.move_client.call_async(request)
        self.get_logger().info('Robot performed gesture 1, waiting for player to respond...')
        
        # Set a timer to check the player's gesture after 2 seconds
        self.delay_timer = self.create_timer(timer_period_sec=2, callback=self.check_player_gesture)
        
    def check_player_gestures(self):
        if self.player_gesture == "scissors":  # Assuming the player_gesture is a string "1" if correct
            self.get_logger().info('Player did the same gesture')
            request = Move.Request()
            request.config_idx = 3  # "Yes" gesture
            self.move_client.call_async(request)
        else:
            self.get_logger().info(f'Player did not do the same gesture, they did: {self.player_gesture} when the correct gesture was: 1')
            request = Move.Request()
            request.config_idx = 9  # "No no no" gesture
            self.move_client.call_async(request)
    

#algorithm which, knowing the correct sequence of gestures, will wait for the player to do the same sequence of gestures
#after the player has done the same sequence of gestures, the robot will do a "yes" gesture and the game will continue  
        
def main(args=None):
    rclpy.init(args=args)
    allegro_game = AllegroGame()
    rclpy.spin(allegro_game)
    allegro_game.destroy_node()
    rclpy.shutdown()
    
#game starts with issuing one gesture to the hand 
#then, give the player 3 seconds to do the same gesture...read from the gesture topic 

#if the player does not do the same gesture, the robot will do a "no no no" gesture and the game will reset 
    
    #if the player does the same gesture, the robot will do a "yes" gesture and the game will continue

#after 3 seconds, round will be incremented, and the robot will generate #round number of gestures 






#define the configurations that lead to rock, paper, and scissors 
#test them with the service call, then try making this node perform them with the
#janky moveit controller

# rock_joints = [-0.1194, 1.2068, 1.0, 1.4042, 
                # -0.0093, 1.2481, 1.4073, 0.8163, 
                # 0.1116, 1.2712, 1.3881, 1.0122, 
                # 0.6017, 0.2976, 0.9034, 0.7929]

#service call:
# ros2 service call /set_joints control_hand/srv/SetJoints {"joint_positions: [-0.1194, 1.2068, 1.0, 1.4042, -0.0093, 1.2481, 1.4073, 0.8163, 0.1116, 1.2712, 1.3881, 1.0122, 0.6017, 0.2976, 0.9034, 0.7929]}


#reset all to zero service call
# static double rock[] = {
#   -0.1194, 1.2068, 1.0, 1.4042,
#   -0.0093, 1.2481, 1.4073, 0.8163,
#   0.1116, 1.2712, 1.3881, 1.0122,
#   0.6017, 0.2976, 0.9034, 0.7929};

# static double paper[] = {
#   -0.1220, 0.4, 0.6, -0.0769,
#   0.0312, 0.4, 0.6, -0.0,
#   0.1767, 0.4, 0.6, -0.0528,
#   0.5284, 0.3693, 0.8977, 0.4863};

# static double scissors[] = {
#   0.0885, 0.4, 0.6, -0.0704,
#   0.0312, 0.4, 0.6, -0.0,
#   0.1019, 1.2375, 1.1346,
#   1.0244, 1.0, 0.6331, 1.3509, 1.0};

# static double cust1[] = {
#   0.0, 1.6, 0.426, 0.0,
#   0.109, 0.0, 0.0, 0.0,
#   0.048, 0.0, 0.0, 0.072,
#   1.396, 0.018, 0.0, 1.089};

# static double cust2[] = {
#   -0.2124, -0.177, 0.148, 1.312,
#   0.012, 0.323, 1.079, 1.335,
#   -0.144, 0.943, 0.335, 1.458,
#   0.323, 0.182, 0.702, 0.297
# };

