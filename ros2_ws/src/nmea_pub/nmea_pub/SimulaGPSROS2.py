'''
*******************************************************************************
 FileName:     SimulaGPSROS2.py FATEC SANTO ANDRÉ
 Dependencies: veja seção de importações
 Processor:    Intel
 Compiler:     Python 3.7.9
 Company:      FATEC Santo Andre
 Author:       Prof. Edson Kitani
 Date:         14/09/2023
 Software License Agreement: Somente para fins didáticos
 *******************************************************************************
 File Description: Simula dados enviados de um GPS no formato NMEA e envia como
                   mensagens do ROS2. Os dados do GPS estão num array.
                   O programa cria uma tread para enviar os dados via serial e 
                   no programa principal temos o loop de interrupção caso 
                   pressionemos a tecla q + return
 Change History:
 1.0   14/09/2023  Versão inicial
 
*******************************************************************************
'''

# Seção de importações

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading

# Área de Globais

keepSending = True # Variável global para controlar o loop de envio

# Publica no ROS2
class NmeaPublisher(Node):
    def __init__(self, nmeaMessages):
        super().__init__('nmea_publisher')
        self.publisher_ = self.create_publisher(String, '/nmea_sentence', 10)
        self.nmeaMessages = nmeaMessages

    def send_nmea_messages(self):
        global keepSending
        while keepSending:
            for message in self.nmeaMessages:
                if not keepSending:
                    break
                msg = String()
                msg.data = message
                self.publisher_.publish(msg)
                time.sleep(1)  # Aguardar 1 segundo


nmeaMessages = [
    "$GPRMC,123519,A,2339.6278,S,4631.8142,W,0,0,140923,,*6A",
    "$GPRMC,123520,A,2339.6277,S,4631.8141,W,0,0,140923,,*6A",
    "$GPRMC,123521,A,2339.6276,S,4631.8140,W,0,0,140923,,*6A",
    "$GPRMC,123522,A,2339.6275,S,4631.8139,W,0,0,140923,,*6A",
    "$GPRMC,123523,A,2339.6274,S,4631.8138,W,0,0,140923,,*6A",
    "$GPRMC,123524,A,2339.6273,S,4631.8137,W,0,0,140923,,*6A",
    "$GPRMC,123525,A,2339.6272,S,4631.8136,W,0,0,140923,,*6A",
    "$GPRMC,123526,A,2339.6271,S,4631.8135,W,0,0,140923,,*6A",
    "$GPRMC,123527,A,2339.6270,S,4631.8134,W,0,0,140923,,*6A",
    "$GPRMC,123528,A,2339.6269,S,4631.8133,W,0,0,140923,,*6A",
    "$GPRMC,123529,A,2339.6268,S,4631.8132,W,0,0,140923,,*6A",
    "$GPRMC,123530,A,2339.6267,S,4631.8131,W,0,0,140923,,*6A",
    "$GPRMC,123531,A,2339.6266,S,4631.8130,W,0,0,140923,,*6A",
    "$GPRMC,123532,A,2339.6265,S,4631.8129,W,0,0,140923,,*6A",
    "$GPRMC,123533,A,2339.6264,S,4631.8128,W,0,0,140923,,*6A",
    "$GPRMC,123534,A,2339.6263,S,4631.8127,W,0,0,140923,,*6A",
    "$GPRMC,123535,A,2339.6262,S,4631.8126,W,0,0,140923,,*6A",
    "$GPRMC,123536,A,2339.6261,S,4631.8125,W,0,0,140923,,*6A",
    "$GPRMC,123537,A,2339.6260,S,4631.8124,W,0,0,140923,,*6A",
    "$GPRMC,123538,A,2339.6259,S,4631.8123,W,0,0,140923,,*6A",
    "$GPRMC,123539,A,2339.6258,S,4631.8122,W,0,0,140923,,*6A"
]

# Inicializar o ROS 2
rclpy.init()

# Cria um nó de publicação e uma thread para enviar mensagens
nmea_publisher = NmeaPublisher(nmeaMessages)
send_thread = threading.Thread(target=nmea_publisher.send_nmea_messages)
send_thread.start()

# Área do programa principal

print(" ")
print("Iniciando transmissão...")
print(" ")

# Aguardar pela entrada do usuário para parar o loop
print("Pressione 'q' para parar o envio...")
while True:
    userInput = input()
    if userInput.lower() == 'q':
        keepSending = False
        break

# Encerra o ROS 2
rclpy.shutdown()
