#!/usr/bin/env python3
# coding: utf-8

import socket
import sys
from mainte_data import *
import ctypes as c

server_ip = "0.0.0.0"
server_port = 3333
listen_num = 5
buffer_size = 1024

tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
tcp_server.bind((server_ip, server_port))
tcp_server.listen(listen_num)

control_data = ControlData()

print("!!! Input Ctrl + C to stop server !!!")
try:
    while True:
        client, address = tcp_server.accept()
        print("[*] Connected!! [ Source : {}]".format(address))

        while True:
            data = client.recv(buffer_size)
            print("[*] Received Data : {}".format(data))
            client.send(control_data)

        client.close()
except KeyboardInterrupt:
    print("!!! Call terminate !!!")
    tcp_server.close()
    sys.exit(0)