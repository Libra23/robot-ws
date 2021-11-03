#!/usr/bin/env python3
# coding: utf-8

from mainte_server_main import *
from mainte_gui_main import *

# server thread
mainte_server_thread = MainteServerMain()
mainte_server_thread.start()

# gui thread (main thread)
mainte_gui = MainteGuiMain()
mainte_gui.thread()