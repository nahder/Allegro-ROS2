import rclpy
from rclpy.node import Node
import mediapipe as mp
import cv2
import numpy as np
from .submodules.model import KeyPointClassifier
from .utils import calc_landmark_list, pre_process_landmark
from std_msgs.msg import String


class GestureClassifier(Node):

    def __init__(self):
        super().__init__("gesture_classifier")

        self.kpclf = KeyPointClassifier()
        self.gestures = {
            1: "rock",
            2: "paper",
            3: "scissors",
        }

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_drawing_styles = mp.solutions.drawing_styles
        self.mp_hands = mp.solutions.hands
        self.cap = cv2.VideoCapture(0)
        self.hands = self.mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5,
        )

        self.timer = self.create_timer(0.01, self.timer_cb)

        # set up publisher for detected gesture
        self.gesture_pub = self.create_publisher(String, "gesture", 10)

    def timer_cb(self):
        success, image = self.cap.read()
        if not success:
            self.get_logger().error("Ignoring empty camera frame.")
            return

        # Process the image
        image.flags.writeable = False
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        results = self.hands.process(image)

        # Draw the hand annotations on the image.
        image.flags.writeable = True
        image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        gesture_index = -1  # Initialize to -1
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                landmark_list = calc_landmark_list(image, hand_landmarks)
                keypoints = pre_process_landmark(landmark_list)
                gesture_index = self.kpclf(keypoints)

                self.mp_drawing.draw_landmarks(
                    image,
                    hand_landmarks,
                    self.mp_hands.HAND_CONNECTIONS,
                    self.mp_drawing_styles.get_default_hand_landmarks_style(),
                    self.mp_drawing_styles.get_default_hand_connections_style(),
                )

        # Handle gesture recognition result

        gesture_msg = String()
        if gesture_index in self.gestures:
            self.get_logger().info(f"Gesture detected: {self.gestures[gesture_index]}")
            gesture_msg.data = self.gestures[gesture_index]
        else:
            self.get_logger().info("No gesture detected")
            gesture_msg.data = "none"

        # TODO EITHER PUBLISH ACCURACY, OR TAKE AVERAGE OF LAST 5 GESTURES

        self.gesture_pub.publish(gesture_msg)

        # display the image
        final_image = cv2.flip(image, 1)
        cv2.imshow("MediaPipe Hands", final_image)
        if cv2.waitKey(5) & 0xFF == 27:
            self.cap.release()
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    gesture_classifier = GestureClassifier()
    rclpy.spin(gesture_classifier)
    gesture_classifier.cap.release()  # Release the camera resource
    cv2.destroyAllWindows()  # Close any OpenCV windows
    rclpy.shutdown()


if __name__ == "__main__":
    main()
