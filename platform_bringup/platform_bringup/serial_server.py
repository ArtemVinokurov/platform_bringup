#!/usr/bin/env python3

import serial


class SerialServer():
    def __init__(self, port):
        try:
            self.serial_port = serial.Serial(port, 1000000, timeout=1)
        except serial.SerialException:
            print("Serial connect failed")
            return None
        print("Serial connection is done!")
        self.serial_port.reset_input_buffer()

    def send_cmd(self, cmd):
        print("Sending: " + str(cmd))
        self.serial_port.write(bytes(str(cmd), "utf-8"))

    def recieve_cmd(self):
        self.serial_port.flush()
        line = self.serial_port.readline()
        # self.serial_port.flushInput()
        if len(line) > 0:
            data = line.decode("utf-8").strip()
            #data = line.strip()
            return data
        else:
            # print("Failed recieved data")
            return None
   
    def __del__(self):
        self.serial_port.close()

    

