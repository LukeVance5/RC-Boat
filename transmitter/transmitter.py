import serial
import numpy as np
import keyboard
import time
port = "COM22"

with serial.Serial(port,9600,timeout = 0.050) as ser:
    while (1):
        key = keyboard.read_key()
        if (key):
            if (key == "w"):
                print("Transmitting w")
                ser.write(b'w')
                while(1):
                    data = ser.read(1)
                    if len(data) > 0: #was there a byte to read?
                        print("Returned data: " + chr(ord(data)))
                        break;
            if (key == "a"):
                print("Transmitting a")
                ser.write(b'a')
                while(1):
                    data = ser.read(1)
                    if len(data) > 0: #was there a byte to read?
                        print("Returned data: " + chr(ord(data)))
                        break;
            if (key == "s"):
                print("Transmitting s")
                ser.write(b's')
                while(1):
                    data = ser.read(1)
                    if len(data) > 0: #was there a byte to read?
                        print("Returned data: " + chr(ord(data)))
                        break;            
            if (key == "d"):
                print("Transmitting d")
                ser.write(b'd')
                while(1):
                    data = ser.read(1)
                    if len(data) > 0: #was there a byte to read?
                        print("Returned data: " + chr(ord(data)))
                        break;     
            if (key == "c"):
                print("Transmitting c")
                ser.write(b'c')
                while(1):
                    data = ser.read(1)
                    if len(data) > 0: #was there a byte to read?
                        print("Returned data: " + chr(ord(data)))
                        break;     
            if (key == "r"):
                print("Transmitting r")
                ser.write(b'r')
                while(1):
                    data = ser.read(1)
                    if len(data) > 0: #was there a byte to read?
                        print("Returned data: " + chr(ord(data)))
                        break;    
            if (key == "z"):
                print("Transmitting z")
                ser.write(b'z')
                while(1):
                    data = ser.read(1)
                    if len(data) > 0: #was there a byte to read?
                        print("Returned data: " + chr(ord(data)))
                        break;    
        time.sleep(0.5)
