import get_speed, get_steer
import can, time, struct, socket
from rclpy.node import Node
from geometry_msgs.msg import Twist

steer_speed = 50
range = 0.4
speed_pwm = lambda speed: 1.23*speed + 11.43

class SubscriberNode(Node):
    def __init__(self):
        global current_speed, current_steer
        super().__init__("subscriber_node")
        self.subscription = self.create_subscription(
            Twist,
            "/cmd_vel",
            self.cmd_vel_callback,
            10
        )
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'speed_steer_topic', 10)
        self.pub_msg = Twist()
        tempo=time.time()
        connect_canbus()
    def pub(self,speed,steer):
        self.pub_msg.linear.x = float(speed)
        self.pub_msg.angular.z = float(steer)
        self.publisher_.publish(self.pub_msg)

    def cmd_vel_callback(self, msg):
        speed_target = msg.linear.x
        steer_target = msg.angular.z
        Speed.update_target(speed=speed_target)
        Steer.update_target(steer=steer_target)

        #self.get_logger().info(f"Received: {msg}")

class Malha_PID():
    def __init__(self, kp, ki, kd,control_id):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.id = control_id #0 -> ACCEL e #2 -> DIR
        self.u_ant = 0
        self.erro = 0
        self.erro_ant = 0
        #self.erro_ant_ant = 0
        self.speed_target =0
        self.steer_target =0
        self.atual =0
        #self.current_speed =0
        self.err_sum=0
    def update_target(self,speed =0,steer=0):
        self.speed_target = speed
        self.steer_target = steer

    def update_malha(self,ts):
        if(self.id == 0):
            #self.erro = self.speed_target
            self.erro = self.speed_target - self.atual
            self.err_sum += self.erro * ts
            #self.erro = self.speed_target
            #print("Current Speed: ",self.atual)
        elif(self.id == 2):
            #atual = current_steer
            self.erro = self.steer_target - self.atual
            self.err_sum += self.erro * ts
            #print("\nERRO: ")
            #print(self.erro)
        #u = self.u_ant + self.kp*(self.erro-self.erro_ant)+self.ki*(ts/2)*(self.erro-self.erro_ant)+(self.kd/ts)*(self.erro - self.erro_ant)
        u = self.kp*self.erro + self.ki * self.err_sum + self.kd * (self.erro - self.erro_ant)
        #if(self.id == 2 and u >0): u = u+36
        #if(self.id == 2 and u< 0): u = u-36
        self.u_ant = u        
        self.erro_ant_ant = self.erro_ant
        self.erro_ant = self.erro

        if(self.id == 0):
            u = speed_pwm(u)
            u= limit(u,0,25)
        elif(self.id == 2):
            print("ERRO: ",self.erro)
            steer_speed = (abs(u)*0.6 + 40)
            print("STEER ",steer_speed)
            #if(abs(self.erro) > 1): steer_speed = 100
            #elif(abs(self.erro) > 0.8): steer_speed = 75
            #elif(abs(self.erro) > 0.6): steer_speed = 60
            #elif(abs(self.erro) > 0.4): steer_speed = 45
            if(self.erro > 0 and self.erro > range):
                u = steer_speed
                print("DIREITA")
            elif(self.erro <0 and self.erro < -range):
                u = -steer_speed
                print("ESQUERDA")
            else: u =0
            #u=steer_speed
            #if(u<0): u= (-1)*limit(u,-100,-40)
            #else: u= (-1)*limit(u,40,100)
            #if(u<0): u = limit(u,-100,-40)
            #else: u =  limit(u,40,100)
            #self.u_ant = -u
            #u = limit(u,-50,50)
        control_array[self.id] = int(round(u))
        if(self.id == 2):print(control_array)
        send_data(control_array)




def connect_canbus():
    global bus
    bustype = 'socketcan'
    channel = 'can0'
    filters = [
        {"can_id": 0xCFE6C17,"can_mask":0x1FFFFFFF, "extended": True},#SPEED
        {"can_id": 0x8F01D13, "can_mask": 0x1FFFFFFF, "extended": True},#STEER
    ]
    bus = can.interface.Bus(channel="can0", bustype="socketcan", can_filters=filters)
    __notifier = can.Notifier(bus, [update], 0)


def update(msg):
    global tempo
    if(msg.arbitration_id == int("0xCFE6C17",0)):
        #print("VEIO SPEED")
        current_speed = get_speed.byte_to_value_speed(msg.data)
        ts = time.time() - tempo
        Speed.atual = current_speed
#ts=0.1
 #       node.pub(current_speed,0)
        Speed.update_malha(ts)
        tempo = time.time()

    elif(msg.arbitration_id == int("0x8F01D13",0)):
        current_steer = get_steer.byte_to_value_steer(msg.data)
        #print("VEIO STEER")
        ts = time.time() - tempo
        Steer.atual = current_steer
        Steer.update_malha(ts)
        tempo = time.time()
   # node.pub(current_speed,0)



def connect_to_action_server():
    #SOCKET
    HOST = "127.0.0.1"  # The server's hostname or IP address
    PORT = 8080  # The port used by the server

    global stream
    stream = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    stream.connect((HOST, PORT))

def send_data(arr):
    data = struct.pack('4i',*arr)
    stream.send(data)

def enable_auto():
    arr = control_array
    arr[3] =1
    send_data(arr)

def main(args=None):
    global control_array, current_speed, current_steer, speed_target, steer_target, limit, tempo
    current_steer =0
    current_speed=0
    speed_target=0
    steer_target =0
    limit = lambda num, lower_lim, upper_lim: max(min(upper_lim,num),lower_lim)
    tempo =0
    tempo = time.time()
    control_array = [0,0,0,0]
    connect_to_action_server()
    enable_auto()
    global Speed, Steer, node
    Speed = Malha_PID(5,0.5,0.05,0)
    Steer = Malha_PID(4,0,0,2)
    print("LETS GO")
    rclpy.init(args=args)
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()













