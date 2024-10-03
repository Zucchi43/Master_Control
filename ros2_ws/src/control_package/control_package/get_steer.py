import can, time

pi = 3.1416


def remap(old_val, old_min, old_max, new_min, new_max):
    return (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min

def remap_turn(turn_hex):
    table={'0x20':0,
           '0x21':1,
           '0x22':2,
           '0x23':3,
           '0x1F':-1,
           '0x1E':-2,
           '0x1D':-3}
    if(turn_hex == '0x1d'): return -3
    elif(turn_hex == '0x20'): return 0
    elif(turn_hex == '0x21'): return 1
    elif(turn_hex == '0x22'): return 2
    elif(turn_hex == '0x23'): return 3
    elif(turn_hex == '0x1f'): return -1
    elif(turn_hex == '0x1e'): return -2
    return 0


def byte_to_value_steer(data):
    bytes_resto = hex( (data[1] << 8) | data[0])
    #goes from 3.1416(0x8A10) to -3.1416(0x7185) -> 1 turn | Step -> 0.001 = 0x1 | 6283 numbers
    bytes_turns = hex(data[2]) 
    # 0 turn -> 0x20 | 1 turn -> 0x21 | 2 turn -> 0x22 | 3 turn ->0x23
    # 0 turn -> 0x20 | -1 turn -> 0x1F | -2 turn -> 0x1E | -3 turn ->0x1D
    bytes_pi = hex( (data[5] << 8) | data[4]) #8A10 -> 3.1416
    resto_rad = remap(int(bytes_resto,16),int('7185',16),int('8A10',16),-pi,pi)

    converted_data = remap_turn(bytes_turns) *(2*pi)  + resto_rad
   # print("RESTO: ")
    #print(resto_rad)
    #print("\n")
    #print("Voltas: ")
    #print(remap_turn(bytes_turns))
    #print("\n")
    return converted_data #RADS de 7pi até -7pi +-

if __name__ == "__main__":
    bustype = 'socketcan'
    channel = 'can0'
    filters = [
        {"can_id": 0x8F01D13, "can_mask": 0x1FFFFFFF, "extended": True},
    ]
#volante
#speed CPC
#speed ICUC - Pego video Ultimos dois Bytes
#speed Break

    #bus = can.interface.Bus(channel="can0", bustype="socketcan", can_filters=filters)
    while(1):
       # message = bus.recv(0.5)
       # print(byte_to_value(message.data))
       a = bytearray.fromhex('3b 78 22 ff 10 8a f5 ea')
       print(byte_to_value_steer(a))
       time.sleep(1)