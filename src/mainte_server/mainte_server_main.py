#!/usr/bin/env python3
# coding: utf-8

from multiprocessing import Process
import socket
from packet_data import *
from msg_data import *
class MainteServerMain(Process):
    def __init__(self, input_queue, output_queue):
        super(MainteServerMain, self).__init__()
        print("MainteServer Constructor")
        self.input_queue = input_queue
        self.output_queue = output_queue

    def run(self):
        # parameter
        server_ip = "0.0.0.0"
        server_port = 3333
        listen_num = 5
        # start tcp server
        tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        tcp_server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        tcp_server.bind((server_ip, server_port))
        tcp_server.listen(listen_num)
        # main loop
        print("!!! Input Ctrl + C to stop server !!!")

        # wait until establish connection
        client, address = tcp_server.accept()
        print("[*] Connected!! [ Source : {}]".format(address))

        # start receive tcp thread
        receive_thread = Process(target=self.receive, args=(client,))
        receive_thread.daemon = True
        receive_thread.start()

        # start send tcp and receive queue from gui process
        terminate_flag = False
        while not terminate_flag:
            try:
                while not self.input_queue.empty():
                    msg = self.input_queue.get()
                    if msg.header.type == MsgType.MSG_ALL_TERMINATE:
                        print("!!! Receive terminate !!!")
                        terminate_flag = True
                    if msg.header.type == MsgType.MSG_GUI_TO_SERVER_CONTROL_DATA:
                        print("!!! Receive Control !!!")
                        control_data_req = PacketControlDataReq(msg.arm_id, msg.control_data)
                        client.send(control_data_req)
            except KeyboardInterrupt:
                print("!!! Call terminate !!!")
                terminate_flag = True
        print("!!! Exit !!!")
        tcp_server.close()

    def receive(self, client):
        buffer_size = 1024
        while True:
            # receive
            data = client.recv(buffer_size)
            print("[*] Received Data : {}, type : {}".format(data, type(data)))
            tcp_packet_type = GetPacketType(data)
            if tcp_packet_type == PacketType.ROBOT_TO_MAINTE_ARM_INFO:
                arm_info = PacketArmInfoRes()
                memmove(addressof(arm_info), data, sizeof(arm_info))
                print("num_arm = {}, num_joint = {}".format(arm_info.num_arm, arm_info.num_joint))
