#!/usr/bin/env python3
# coding: utf-8

import socket

server_ip = "0.0.0.0"
server_port = 3333
listen_num = 5
buffer_size = 1024

# 1.ソケットオブジェクトの作成
tcp_server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

# 2.作成したソケットオブジェクトにIPアドレスとポートを紐づける
tcp_server.bind((server_ip, server_port))

# 3.作成したオブジェクトを接続可能状態にする
tcp_server.listen(listen_num)

# 4.ループして接続を待ち続ける
while True:
    # 5.クライアントと接続する
    client,address = tcp_server.accept()
    print("[*] Connected!! [ Source : {}]".format(address))

    # 6.データを受信する
    data = client.recv(buffer_size)
    print("[*] Received Data : {}".format(data))

    # 7.クライアントへデータを返す
    client.send(b"ACK!!")

    # 8.接続を終了させる
    client.close()