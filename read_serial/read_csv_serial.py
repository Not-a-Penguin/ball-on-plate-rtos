import serial
import struct
import time
from enum import Enum
import csv


file_name = "raw_"
COM_port = "COM4"

ser = serial.Serial(COM_port)
ser.baudrate = 1000000
file_name += time.strftime("%Y%m%d-%H%M%S")

file   = open(f"./data/{file_name}.csv", "w")

try:
    print("started reading")
    while True:
        data = ser.read_until(expected='\n'.encode('UTF-8'))

        if len(data) > 10 and len(data) < 100:
            print(data.decode('utf-8'), end="")
            file.write(data.decode('utf-8'))

except KeyboardInterrupt:
    print("Exiting and saving data...")
finally:
    file.close()
    ser.close()