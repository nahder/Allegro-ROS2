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

        self.round_logger = ResetLogger(self.get_logger().info)
        self.cbgroup1 = MutuallyExclusiveCallbackGroup()
        self.cbgroup2 = MutuallyExclusiveCallbackGroup()

        # self.defined_gestures = ["rock", "scissors", "3claw"]

        self.defined_gestures = ["rock", "scissors"]

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
        self.round = 1

        self.sampling_count = 0
        self.stability_count = 0
        self.play_round = True

    # perform as many gestures as the round number
    # want to generate #round indices which are indexable to defined_gestures

    def perform_gestures(self):
        request = SetConfig.Request()
        num_gestures = min(self.round, len(self.defined_gestures))
        indexes = random.sample(range(len(self.defined_gestures)), num_gestures)

        for index in indexes:
            request.name = self.defined_gestures[index]
            self.set_config_srv.call_async(request)

        self.performed_gestures = [self.defined_gestures[i] for i in indexes]

    def player_gesture_cb(self, msg):
        self.stability_count += 1
        if self.stability_count > 20:
            self.cur_player_gesture = msg.data
            self.stability_count = 0

    def game_loop(self):
        if self.play_round and self.round < 3:
            self.round_logger.reset()
            self.round_logger.log("Round: %d" % self.round)
            self.player_gestures = []
            self.prev_player_gesture = None
            self.perform_gestures()
            self.round_logger.log("robot gestures: %s" % self.performed_gestures)
            self.sampling_count = 0  # begins accumulating player gestures
            self.play_round = False

    def accumulate_player_gestures(self):
        if self.sampling_count < 120:  # accumulating for 5 seconds
            self.round_logger.log("Start copying the robot's gestures!")

            if self.cur_player_gesture != self.prev_player_gesture:
                self.player_gestures.append(self.cur_player_gesture)
                self.prev_player_gesture = self.cur_player_gesture

            self.sampling_count += 1

        else:  # 5 seconds have passed
            self.round_logger.log("player gestures: %s" % self.player_gestures)

            if self.performed_gestures == self.player_gestures:
                self.round += 1
                self.play_round = True
                self.prev_player_gesture = None
                self.round_logger.log("You did it! Next round!")
                self.player_gestures = []

            else:
                request = SetConfig.Request()
                request.name = "nonono"
                self.set_config_srv.call_async(request)
                self.round_logger.reset()
                self.round = 1
                self.play_round = True
                self.round_logger.log("You failed!")
                self.round_logger.log("robot gestures: %s" % self.performed_gestures)
                self.round_logger.log("player gestures: %s" % self.player_gestures)
                self.round_logger.log("Restarting...")
                self.player_gestures = []


class ResetLogger:
    def __init__(self, logger_func):
        self.logger_func = logger_func
        self.logged_set = set()

    def log(self, msg):
        if msg not in self.logged_set:
            self.logged_set.add(msg)
            self.logger_func(msg)

    def reset(self):
        self.logged_set.clear()


def main(args=None):
    rclpy.init(args=args)
    allegro_game = AllegroGame()
    rclpy.spin(allegro_game)
    allegro_game.destroy_node()
    rclpy.shutdown()
