#!/usr/bin/env python3
# coding: utf-8

from mainte_server_main import *
from mainte_gui_main import *
from multiprocessing import Queue

def main():
    mq_gui_to_server = Queue()
    mq_server_to_gui = Queue()

    # server thread
    mainte_server_thread = MainteServerMain(mq_gui_to_server, mq_server_to_gui)
    mainte_server_thread.start()

    # gui thread (main thread)
    mainte_gui = MainteGuiMain()
    mainte_gui.thread(mq_server_to_gui, mq_gui_to_server)

    mainte_server_thread.join()

if __name__ == '__main__':
    main()