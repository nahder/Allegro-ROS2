import time
import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty

class Delay(Node): 

    def __init__(self):
        super().__init__('delay_node')
        
        self.delay_service = self.create_service(Empty, 'delay', self.delay_cb)
        
    def delay_cb(self, request, response):
        time.sleep(0.2) 
        return response 
        
    #MAKE A DELAY SERVICE WITH TIME.SLEEP IN IT AND THEN AWAIT THAT SERVICE 
def main(args=None):
    rclpy.init(args=args)
    delay = Delay()
    rclpy.spin(delay)
    delay.destroy_node()
    rclpy.shutdown()