import can, time

pi = 3.1416


def remap(old_val, old_min, old_max, new_min, new_max):
    return (new_max - new_min)*(old_val - old_min) / (old_max - old_min) + new_min



def byte_to_value_speed(data):
    byte_speed_dec = hex(data[7])
    byte_speed_frac = hex(data[6])
    frac_dec = remap(int(byte_speed_frac,16),0,int('80',16),0,0.5)
    converted_data = int(byte_speed_dec,16) + frac_dec
    return converted_data #RADS de 7pi até -7pi +-

if __name__ == "__main__":
    bustype = 'socketcan'
    channel = 'can0'
    filters = [
        {"can_id": 0xCFE6C17,"can_mask":0x1FFFFFFF, "extended": True},
    ]
#volante
#speed CPC
#speed ICUC - Pego video Ultimos dois Bytes
#speed Break

    #bus = can.interface.Bus(channel="can0", bustype="socketcan", can_filters=filters)
    while(1):
       # message = bus.recv(0.5)
       # print(byte_to_value(message.data))
       a = bytearray.fromhex('7b 1f ff c0 00 fe e6 00')
       print(byte_to_value_speed(a))
       time.sleep(1)