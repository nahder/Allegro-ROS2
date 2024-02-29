import rclpy
import random
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
from control_hand.srv import SetConfig
import numpy as np
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from enum import Enum


# CURRENT GESTURES: 0: rock, 1: paper, 2: scissors,(3: okay,  9: nonono)
class AllegroGame(Node):
    def __init__(self):
        super().__init__("allegro_game")

        self.cbgroup1 = MutuallyExclusiveCallbackGroup()
        self.cbgroup2 = MutuallyExclusiveCallbackGroup()

        self.defined_gestures = ["paper", "rock", "scissors", "okay", "3claw"]
        self.gesture_sub = self.create_subscription(
            String, "/gesture", self.player_gesture_cb, 10, callback_group=self.cbgroup1
        )

        self.timer = self.create_timer(
            0.05, self.accumulate_player_gestures, callback_group=self.cbgroup2
        )

        self.game_timer = self.create_timer(
            0.05, self.game_loop, callback_group=self.cbgroup2
        )

        self.set_config_srv = self.create_client(SetConfig, "set_config")
        self.reset_client = self.create_client(Trigger, "controller/reset")

        self.cur_player_gesture = None
        self.prev_player_gesture = None

        self.player_gestures = []
        self.performed_gestures = []
        self.round = 2

        self.sampling_count = 0
        self.play_round = True

    # perform as many gestures as the round number
    def perform_gestures(self):
        request = SetConfig.Request()
        indexes = random.sample(range(self.round), self.round)

        for index in indexes:
            request.name = self.defined_gestures[index]
            self.set_config_srv.call_async(request)

        self.performed_gestures = [self.defined_gestures[i] for i in indexes]

    def player_gesture_cb(self, msg):
        self.cur_player_gesture = msg.data

    def game_loop(self):
        if self.play_round:
            self.perform_gestures()
            self.get_logger().info("robot gestures: %s" % self.performed_gestures)
            self.get_logger().info("Start copying the robot gestures!")
            self.sampling_count = 0  # begins accumulating player gestures
            self.play_round = False

    # can start sampling by setting sampling_count to 0
    # TODO: improve stability; sometimes rock at first will add scissors then rock
    def accumulate_player_gestures(self):
        if self.sampling_count < 100:
            start_time = self.get_clock().now().to_msg()
            start_time = start_time.sec + start_time.nanosec * 1e-9
            cur_time = 0.0
            cur_time = self.get_clock().now().to_msg()
            cur_time = cur_time.sec + cur_time.nanosec * 1e-9

            if self.cur_player_gesture != self.prev_player_gesture:
                # self.get_logger().info("adding gesture: %s" % self.cur_player_gesture)
                self.player_gestures.append(self.cur_player_gesture)
                self.prev_player_gesture = self.cur_player_gesture

            # print("player gestures", self.player_gestures)
            self.sampling_count += 1
        else:
            self.get_logger().info(
                "player gestures: %s" % self.player_gestures, once=True
            )


def main(args=None):
    rclpy.init(args=args)
    allegro_game = AllegroGame()
    rclpy.spin(allegro_game)
    allegro_game.destroy_node()
    rclpy.shutdown()


# game starts with issuing one gesture to the hand
# then, give the player 3 seconds to do the same gesture...read from the gesture topic

# if the player does not do the same gesture, the robot will do a "no no no" gesture and the game will reset

# if the player does the same gesture, the robot will do a "yes" gesture and the game will continue

# after 3 seconds, round will be incremented, and the robot will generate #round number of gestures


# algorithm which, knowing the correct sequence of gestures, will wait for the player to do the same sequence of gestures
# after the player has done the same sequence of gestures, the robot will do a "yes" gesture and the game will continue


# TO DO:
# 1. perform a sequence of gestures
# 2. perform gestures, give the player 3 seconds to do the same,
# if the player did the same,

# x. retrain mediapipe to be tight on rock, paper, scissors, and claw

# def delay(self):
#     self.future = self.delay_client.call_async(Empty.Request())
#     rclpy.spin_until_future_complete(self, self.future)
#     return self.future.result()
