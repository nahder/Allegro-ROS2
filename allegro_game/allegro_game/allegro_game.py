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

        self.defined_gestures = ["rock", "scissors", "L"]

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
        self.performed_gestures = []
        last_gesture_index = None

        for _ in range(self.round):
            # Generate a list of possible next gestures (exclude the last gesture to avoid immediate repetition)
            possible_next_gestures = [
                i for i in range(len(self.defined_gestures)) if i != last_gesture_index
            ]

            # Randomly select the next gesture from the possible next gestures
            next_gesture_index = random.choice(possible_next_gestures)

            # Prepare and send the request for the next gesture
            request.name = self.defined_gestures[next_gesture_index]
            self.set_config_srv.call_async(request)

            # Update the performed gestures list and the last gesture index
            self.performed_gestures.append(self.defined_gestures[next_gesture_index])
            last_gesture_index = next_gesture_index

    # maybe make a diff timer that sets the robot's gestures so
    # that we can control how often its updated (this is just the gesture publishing timer)
    def player_gesture_cb(self, msg):
        # stability_count_thresh = seconds / 0.02 (gestures published every 0.02s)
        seconds = 2.5
        stability_count_thresh = seconds / 0.02
        self.stability_count += 1  # gestures published every 0.02s
        if self.stability_count > stability_count_thresh:  # 3 seconds
            self.cur_player_gesture = msg.data
            self.stability_count = 0

    def game_loop(self):
        if self.play_round and self.round < 10:
            self.round_logger.reset()
            self.round_logger.log("Round: %d" % self.round)
            self.player_gestures = []
            self.prev_player_gesture = None
            self.cur_player_gesture = None
            self.perform_gestures()
            self.round_logger.log("robot gestures: %s" % self.performed_gestures)
            self.sampling_count = 0  # begins accumulating player gestures
            self.play_round = False
            self.round += 1

    def accumulate_player_gestures(self):

        if self.sampling_count < 500:
            self.round_logger.log("Start copying the robot's gestures!")

            if self.cur_player_gesture != self.prev_player_gesture:
                self.player_gestures.append(self.cur_player_gesture)
                self.round_logger.log("Added gesture: %s" % self.cur_player_gesture)
                self.prev_player_gesture = self.cur_player_gesture

            if self.player_gestures == self.performed_gestures:
                self.round_logger.log("You did it! Next round!")
                self.play_round = True

            self.sampling_count += 1

        else:  # 5 seconds have passed
            self.round_logger.log("player gestures: %s" % self.player_gestures)
            request = SetConfig.Request()
            request.name = "nonono"
            self.set_config_srv.call_async(request)
            self.round = 1
            self.round_logger.log("You failed!")
            self.round_logger.log("robot gestures: %s" % self.performed_gestures)
            self.round_logger.log("player gestures: %s" % self.player_gestures)
            self.round_logger.log("Restarting...")
            self.play_round = True


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


# need to add some indication as to when the player has completed 1 gesture out of the sequence
