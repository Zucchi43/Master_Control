import pynmeagps, rclpy, math
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from rclpy.node import Node
goal =  "$GPRMC,125059,A,2339.602063,S,04631.802676,W,000.03,193.3,081023,17.8,W,D*03\r\n"
goal2 = "$GPRMC,133614,A,2339.608724,S,04631.811639,W,000.01,13.7,081023,17.8,W,D*33\r\n"
class GPS_Follow(Node):
    def __init__(self):
        super().__init__("gps_follower_node")
        self.target =pynmeagps.NMEAReader.parse(goal)
        self.gps_sub = self.create_subscription(
            String,
            "/nmea_sentence_topic",
            self.gps_callback,
            10
        )
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.twist_cmd = Twist()
        self.gps_parsed = pynmeagps.NMEAReader.parse(goal)
        self.max_speed = 8 #Km/h
        self.slowing_dist = 10 #meters

    def gps_callback(self,msg):
        #print(msg)
        self.gps_parsed = pynmeagps.NMEAReader.parse(msg.data)
        self.bearing_err = self.calc_bearing()
        self.distance_err = self.calc_dist()
        self.send_twist()

    def calc_dist(self):
        lat1 = self.target.lat
        lon1 = self.target.lon
        lat2= self.gps_parsed.lat
        lon2 = self.gps_parsed.lon
        R = 6371.0 * 1000  # Radius of the Earth in m
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        a = math.sin(dlat / 2) ** 2 + math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.sin(dlon / 2) ** 2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
        dist = R * c
        return dist

    def calc_bearing(self):
        lat1 = self.target.lat
        lon1 = self.target.lon
        lat2= self.gps_parsed.lat
        lon2 = self.gps_parsed.lon
        dlat = math.radians(lat2 - lat1)
        dlon = math.radians(lon2 - lon1)
        y = math.sin(dlon) * math.cos(math.radians(lat2))
        x = math.cos(math.radians(lat1)) * math.sin(math.radians(lat2)) - math.sin(math.radians(lat1)) * math.cos(math.radians(lat2)) * math.cos(dlon)
        brng = math.atan2(y, x)
        brng = math.degrees(brng)
        brng = (brng + 360) % 360
        return brng
    
    def send_twist(self):
        self.twist_cmd.linear.x = float(self.speed_control())
        #self.twist_cmd.angular.z = float(self.bearing_to_steer())
        self.publisher.publish(self.twist_cmd)

    def speed_control(self):
        if(self.distance_err > self.slowing_dist): return self.max_speed
        elif(self.distance_err >=1): return (1-(1/2*self.distance_err))*self.max_speed
        elif(self.distance_err < 1): self.target =pynmeagps.NMEAReader.parse(goal2)
        return 0

    def bearing_to_steer():
        R = 2 # Distance 
        w = 1 #distence between wheels lateral

        return 0

def main(args=None):
    rclpy.init(args=args)
    line_follower = GPS_Follow()
    rclpy.spin(line_follower)
    line_follower.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
