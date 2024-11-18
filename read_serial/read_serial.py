import serial
import struct
import time
from enum import Enum

class EventType(Enum):
    START = 0
    END = 1

class PayloadType(Enum):
    NONE = 0
    FILTER = 1
    MPC = 2

file_name = ""
COM_port = "COM4"

struct_format = '32s I I I x 12s H'
struct_size  = struct.calcsize(struct_format)

mpc_payload_format = 'f f I'
mpc_payload_size = struct.calcsize(mpc_payload_format)

filter_payload_format = 'f f 4x'
filter_payload_size = struct.calcsize(filter_payload_format)

ser = serial.Serial(COM_port)
ser.baudrate = 1000000
# data_file = open(file_name + time.strftime("%Y%m%d-%H%M%S") + '.json', 'a+')

try:
    print("started reading")
    while True:
        binary_data = ser.read_until(expected='\r\n'.encode('UTF-8'))

        # print(binary_data)
        # print(binary_data[0:-2])

        binary_data = binary_data[0:-2]

        if len(binary_data) == struct_size:
            unpacked_data = struct.unpack(struct_format, binary_data)

            # Parse the unpacked data
            taskName = unpacked_data[0].decode('utf-8').strip('\x00')
            eventType = unpacked_data[1]
            time = unpacked_data[2]
            payloadType = unpacked_data[3]
            payload = unpacked_data[4]
            failedMessages = unpacked_data[5]

            print(f"Task Name: {taskName}")
            print(f"Event Type: {eventType}")
            print(f"Time: {time}")
            print(f"Payload Type: {payloadType}")
            # print(f"Payload (raw): {payload}")
            if payloadType == PayloadType.FILTER.value:
                unpacked_payload = struct.unpack(filter_payload_format, payload)
                print(f"original value: {unpacked_payload[0]}")
                print(f"filtered value: {unpacked_payload[1]}")
            elif payloadType == PayloadType.MPC.value:
                unpacked_payload = struct.unpack(mpc_payload_format, payload)
                print(f"MPC u: {unpacked_payload[0]}")
                print(f"MPC cost: {unpacked_payload[1]}")
                print(f"MPC Computation time: {unpacked_payload[2]}")
            print(f"Failed Messages: {failedMessages}")
            print("\n")

except KeyboardInterrupt:
    print("Exiting and saving data...")
finally:
    # data_file.close()
    ser.close()