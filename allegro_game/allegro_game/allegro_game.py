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
        
        self.rate = self.create_rate(2)
        self.gesture_sub = self.create_subscription(String, 'gesture', self.player_gesture_cb, 10) 
        self.move_client = self.create_client(Move, 'controller/move')
        self.reset_client = self.create_client(Trigger, 'controller/reset') 
        self.delay_client = self.create_client(Empty, 'delay')
        
        self.player_gesture = None
        self.moves = []
        self.round = 5

        self.robot_perform_gestures()
    # Performs all the gestures in the moves list
    # Send a reset move in between every gesture (config_idx = -100)
    
    def robot_perform_gestures(self):
        request = Move.Request()
        self.moves = [-100, 1, -100, 1, -100, 2, -100]
        
        for move in self.moves:
            request.config_idx = move
            self.move_client.call_async(request)
            self.delay()
            
    def delay(self): 
        self.future = self.delay_client.call_async(Empty.Request())
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()

        
        # self.get_logger().info('Gesture length: %s' % len(self.moves))
        # for move in self.moves:
        #     self.moves.pop(0)
        #     request.config_idx = move
        #     self.move_client.call_async(request)
        #     self.get_logger().info('Robot performed gesture: %s' % move)
            # self.delay_timer = self.create_timer(timer_period_sec=100, callback=self.check_player_gesture)
            
            # request.config_idx = -100   
            # self.move_client.call_async(request)
            
            # self.delay_timer.cancel()
        self.get_logger().info('Robot performed all gestures, waiting for player to respond...')
            
    # Generates as many gestures as the round number
    # def robot_gesture_generator(self):
    #     previous_gesture = -1 
    #     for _ in range(self.round):
    #         gesture_idx = np.random.randint(3)
    #         # Ensure that the same gesture is not generated consecutively
    #         while gesture_idx == previous_gesture:
    #             gesture_idx = np.random.randint(3)

    #         self.moves.append(gesture_idx)
    #         previous_gesture = gesture_idx
    
    

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
        
    def check_player_gesture(self):
        self.delay_timer.cancel()  # Cancel the timer to prevent it from calling this method again
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
    
    # def timer_cb(self):
    #     pass 
        # if self.start_game and len(self.moves)>0:
        #     self.robot_gesture_generator()
        #     for move in self.moves:
        #         self.move(move)
        #         self.moves.pop(0)
        
        #after the player has done the same gestures, increment round and let the timer then move on to generate new gestures 
        
        
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


