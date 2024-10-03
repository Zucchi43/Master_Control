import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from sensor_msgs.msg import NavSatStatus
import pynmeagps,time

class NavSatFixPublisher(Node):
    def __init__(self):

        self.gps_qualities = {
            # Unknown
            -1: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # Invalid
            0: [
                self.default_epe_quality0,
                NavSatStatus.STATUS_NO_FIX,
                NavSatFix.COVARIANCE_TYPE_UNKNOWN
            ],
            # SPS
            1: [
                self.default_epe_quality1,
                NavSatStatus.STATUS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # DGPS
            2: [
                self.default_epe_quality2,
                NavSatStatus.STATUS_SBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Fix
            4: [
                self.default_epe_quality4,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # RTK Float
            5: [
                self.default_epe_quality5,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ],
            # WAAS
            9: [
                self.default_epe_quality9,
                NavSatStatus.STATUS_GBAS_FIX,
                NavSatFix.COVARIANCE_TYPE_APPROXIMATED
            ]
        }


        super().__init__('navsat_fix_publisher')
        self.erro_available = 0
        self.publisher_ = self.create_publisher(NavSatFix, 'navsat_fix_topic', 10)
        self.stream = serial.Serial('/dev/ttyS0',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
        self.reader = pynmeagps.NMEAReader(self.stream)
        #self.current_message = pynmeagps.NMEAReader.parse(self.stream)
        self.timer_ = self.create_timer(1.0, self.publish_navsat_fix)
        

    def message_sort(self):
        if(self.current_message.msgID == 'RMC'):
            self.status = self.current_message.status
            self.lat = self.current_message.lat
            self.lon = self.current_message.lon
            self.spd = self.current_message.spd
            self.orientation = self.current_message.cog
            self.mag_dev = self.current_message.mv
            self.mag_dev_ew = self.current_message.mvEW
            self.rmc_available= 1
        elif(self.current_message.msgID == 'GGA'):
            self.lat = self.current_message.lat
            self.lon = self.current_message.lon
            self.quality = self.current_message.quality
            self.numSV = self.current_message.numSV
            self.HDOP = self.current_message.HDOP
            self.altitude = self.current_message.alt
            self.geoid_separation = self.current_message.sep
            self.gga_available = 1
        elif(self.current_message.msgID == 'GST') :
            self.stdLat_dev = self.current_message.stdLat
            self.stdLon_dev = self.current_message.stdLong
            self.stdAlt_dev = self.current_message.stdAlt
            self.gst_available = 1

    def callback(self):
        while(1):
            self.current_message = self.reader.parse()
            self.message_sort()
            self.publish_navsat_fix()
            #time.sleep(0.5)

    def publish_navsat_fix(self):
        #(raw_data, parsed_data) = self.nmr.read()
        parsed_data = self.test
        msg = NavSatFix()
        msg.latitude = self.lat # Set latitude value
        msg.longitude = self.lon # Set longitude value

        if(self.altitude): msg.altitude = self.altitude # Set altitude value
        else: msg.altitude = 0.0
        if(self.gst_available==1 and self.gga_available == 1):
            gps_qual = self.gps_qualities[self.quality]
            msg.status.status = gps_qual[1]
            msg.position_covariance_type = gps_qual[2]
            msg.position_covariance[0]= (self.HDOP * self.stdLon_dev) **2
            msg.position_covariance[4]= (self.HDOP * self.stdLat_dev) **2
            msg.position_covariance[8]= (2*self.HDOP * self.stdAlt_dev) **2


        self.publisher_.publish(msg)
        self.get_logger().info(f"Published NavSatFix message: lat={msg.latitude}, lon={msg.longitude}, alt={msg.altitude}")

def main(args=None):
    rclpy.init(args=args)
    navsat_fix_publisher = NavSatFixPublisher()
    rclpy.spin(navsat_fix_publisher)
    navsat_fix_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
