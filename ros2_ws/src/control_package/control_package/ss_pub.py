import rclpy
import get_speed, get_steer
import can, time, struct, socket
from rclpy.node import Node
from geometry_msgs.msg import Twist




class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        self.publisher_ = self.create_publisher(Twist, 'speed_steer_topic', 10)
        self.msg = Twist()
        tempo=time.time()
        self.connect_canbus()


        #self.get_logger().info(f"Received: {msg}")


    def connect_canbus(self):
        global bus 
        bustype = 'socketcan'
        channel = 'can0'
        filters = [
            {"can_id": 0xCFE6C17,"can_mask":0x1FFFFFFF, "extended": True},#SPEED
            {"can_id": 0x8F01D13, "can_mask": 0x1FFFFFFF, "extended": True},#STEER
        ]
        bus = can.interface.Bus(channel="can0", bustype="socketcan", can_filters=filters)
        __notifier = can.Notifier(bus, [self.update], 0)

    def publish(self):

        self.publisher_.publish(self.msg)
        #self.get_logger().info(f"Published NMEA sentence: {msg.data}")

    def update(self,msg):
        if(msg.arbitration_id == int("0xCFE6C17",0)):
        #print("VEIO SPEED")
            self.msg.linear.x = get_speed.byte_to_value_speed(msg.data)

        elif(msg.arbitration_id == int("0x8F01D13",0)):
            self.msg.angular.z = get_steer.byte_to_value_steer(msg.data)
        #print("VEIO STEER")
        self.publish()



def main(args=None):
    limit = lambda num, lower_lim, upper_lim: max(min(upper_lim,num),lower_lim)
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
