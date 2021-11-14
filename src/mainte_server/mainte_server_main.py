#!/usr/bin/env python3
# coding: utf-8

from multiprocessing import Process
import socket
from packet_data import *
class MainteServerMain(Process):
    def __init__(self, input_queue, output_queue):
        super(MainteServerMain, self).__init__()
        print("MainteServer Constructor")
    
    def run(self):
        # parameter
        server_ip = "0.0.0.0"
        server_port = 3333
        listen_num = 5
        buffer_size = 1024
        # start tcp server
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((server_ip, server_port))
        tcp_server.listen(listen_num)
        
        # main loop
        print("!!! Input Ctrl + C to stop server !!!")
        try:
            while True:
                client, address = tcp_server.accept()
                print("[*] Connected!! [ Source : {}]".format(address))
                while True:
                    data = client.recv(buffer_size)
                    print("[*] Received Data : {}, type : {}".format(data, type(data)))
                    tcp_header = TcpHeader()
                    memmove(addressof(tcp_header), data, sizeof(tcp_header))
                    print("Type({}) = {}, Size({}) = {}".format(type(tcp_header.type), tcp_header.type, type(tcp_header.size), tcp_header.size))
                    if tcp_header.type == PacketType.ROBOT_TO_MAINTE_ARM_INFO:
                        arm_info = PacketArmInfoReq()
                        memmove(addressof(arm_info), data, sizeof(arm_info))
                        print("num_arm = {}, num_joint = {}".format(arm_info.num_arm, arm_info.num_joint))
                        control_data = PacketControlDataReq()
                        control_data.arm_id = 3
                        client.send(control_data)
                client.close()
        except KeyboardInterrupt:
            print("!!! Call terminate !!!")
            tcp_server.close()