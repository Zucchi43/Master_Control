import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import serial, time

class NMEASentencePublisher(Node):
    def __init__(self):
        super().__init__('nmea_sentence_publisher')
        self.nmea_data = ""
        self.publisher_ = self.create_publisher(String, 'nmea_sentence_topic', 10)
        #self.timer_ = self.create_timer(1.0, self.publish_nmea_sentence)
        self.stream = serial.Serial('/dev/ttyUSB0',baudrate=9600,parity=serial.PARITY_NONE,stopbits=serial.STOPBITS_ONE,bytesize=serial.EIGHTBITS,timeout=1)
        #self.stream = open("TESTE_FATEC.txt", 'rb')
        self.pub=1
        self.callback()
    
    def callback(self):
        while(1):
            try:
                self.nmea_data = self.stream.readline()
            except:
                time.sleep(0.2)
            #self.nmea_data ="$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"
            self.publish_nmea_sentence()
            #print(self.nmea_data)
            #time.sleep(0.5)


    def nmea_str(self):
        try:
            self.nmea_text = self.nmea_data.decode()
            self.pub=1
        except:
            self.nmea_text = ""
            self.pub = 0
    def publish_nmea_sentence(self):
        self.nmea_str()
        msg = String()
        msg.data = self.nmea_text
        #msg.data = "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"  # Set NMEA sentence data
        if(self.pub==1):
            self.publisher_.publish(msg)
        #self.get_logger().info(f"Published NMEA sentence: {msg.data}")

def main(args=None):
    rclpy.init(args=args)
    nmea_sentence_publisher = NMEASentencePublisher()
    rclpy.spin(nmea_sentence_publisher)
    nmea_sentence_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
