#!/usr/bin/env python3
# coding: utf-8

import tkinter
from tkinter import ttk
from ttkthemes import ThemedTk
from reference_gui import *
from mainte_data import *
from msg_data import *
from packet_data import *

class MainteGuiMain:
    def __init__(self):
        print("MainteGui Constructor")

    def thread(self, input_queue, output_queue):
        root = ThemedTk(theme='radiance')
        app = MainteGui(master = root, num_arm = NUM_ARM, num_joint_per_arm = NUM_JOINT, input_queue=input_queue, output_queue=output_queue)
        app.mainloop()

class MainteGui(ttk.Frame):
    def __init__(self, master, num_arm, num_joint_per_arm, input_queue, output_queue):
        super().__init__(master)
        # init parameter
        self.num_arm = num_arm
        self.num_joint = num_joint_per_arm
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.control_data = [ControlData() for i in range(self.num_arm)]
        # init frame parameter
        self.mode = tkinter.StringVar()
        self.enable_states = [[tkinter.BooleanVar(value = True) for j in range(self.num_joint)] for i in range(self.num_arm)]
        self.reference_frame = [None for i in range(self.num_arm)]
        # prepare frame
        master.title('MainteServer')
        self.create_widgets()
        self.pack()

    def create_widgets(self):
        # frame
        menu_frame =  ttk.Frame(self)
        main_frame = ttk.Frame(self)
        # create menu widgets
        label = ttk.Label(menu_frame, text = 'Control Mode')
        label.grid(row = 0, column = 0)
        mode_labels = [mode.name for mode in ControlMode]
        mode_combo = ttk.Combobox(menu_frame, textvariable = self.mode, value = mode_labels, state = "readonly", width = 6)
        mode_combo.current(0)
        mode_combo.grid(row = 0, column = 1)
        menu_frame.pack(anchor = tkinter.E)
        # create main widgets
        for i in range(self.num_arm):
            # create arm widgets
            arm_frame = ttk.Frame(main_frame)
            label = ttk.Label(arm_frame, text = 'Arm (' + str(i) + ')')
            label.grid(row = 0, column = 0)
            control_button = ttk.Button(arm_frame, text = 'Control', command = self.control_callback(i), width = 7)
            control_button.grid(row = 1, column = 0)
            for j in range(self.num_joint):
                enable_button = ttk.Checkbutton(arm_frame, text = 'Joint(' + str(j) + ')', variable = self.enable_states[i][j])
                enable_button.grid(row = 1, column = j + 1)
            reference_button = ttk.Button(arm_frame, text = 'R', command = self.reference_callback(i), width = 1)
            reference_button.grid(row = 1, column = self.num_joint + 1)
            arm_frame.pack()
            # create separator
            separator = ttk.Separator(main_frame)
            separator.pack(fill="both")
        main_frame.pack()

    # Callback Function --->>>
    def control_callback(self, i):
        def callback():
            print('Call control ' + str(i) + ' Mode = ' + self.mode.get())
            for j in range(self.num_joint):
                if self.enable_states[i][j].get() == True:
                    self.control_data[i].enable[j] = 1
                else:
                    self.control_data[i].enable[j] = 0
            msg = MsgCmdControlData(i, self.control_data[i])
            self.output_queue.put(msg)
        return callback

    def reference_callback(self, i):
        def callback():
            print('Call reference ' + str(i))
            if self.reference_frame[i] == None or not self.reference_frame[i].winfo_exists():
                self.reference_frame[i] = tkinter.Toplevel(self)
                ReferenceGui(self.reference_frame[i], self.mode.get(), i, self.num_joint, self.control_data[i].reference)
        return callback
    # <<<--- Callback Function

if __name__ == '__main__':
    root = ThemedTk(theme='radiance')
    app = MainteGui(master = root, num_arm = 4, num_joint_per_arm = 4)
    app.mainloop()