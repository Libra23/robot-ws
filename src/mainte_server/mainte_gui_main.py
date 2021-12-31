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
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.root = ThemedTk(theme='radiance')
        app = MainteGui(master = self.root, num_arm = NUM_ARM, num_joint_per_arm = NUM_JOINT, input_queue = self.input_queue, output_queue = self.output_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.quit)
        self.root.mainloop()

    def quit(self):
        print("Call destroy")
        msg = MsgCmd(MsgType.MSG_ALL_TERMINATE)
        self.output_queue.put(msg)
        self.root.destroy()

class MainteGui(ttk.Frame):
    def __init__(self, master, num_arm, num_joint_per_arm, input_queue, output_queue):
        super().__init__(master)
        # init parameter
        self.num_arm = num_arm
        self.num_joint = num_joint_per_arm
        self.input_queue = input_queue
        self.output_queue = output_queue
        self.control_data = [ControlData() for i in range(self.num_arm)]
        self.control_state = [False for i in range(self.num_arm)]
        # init frame parameter
        self.mode = tkinter.StringVar()
        self.enable_states = [[tkinter.BooleanVar(value = True) for j in range(self.num_joint)] for i in range(self.num_arm)]
        self.reference_frame = [None for i in range(self.num_arm)]
        style = ttk.Style()
        style.configure("control_on.TButton", foreground="red")
        style.configure("control_off.TButton", foreground="blue")
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
            control_button = ttk.Button(arm_frame, text = 'Control', width = 7, style="control_off.TButton")
            control_button.configure(command = self.control_callback(i, control_button))
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
    def control_callback(self, i, button):
        def callback():
            for mode in ControlMode:
                if mode.name == self.mode.get():
                    self.control_data[i].control_mode = mode.value
            for j in range(self.num_joint):
                if self.enable_states[i][j].get() == True:
                    self.control_data[i].enable[j] = 1
                else:
                    self.control_data[i].enable[j] = 0
            if self.control_state[i]:
                print('Call control off ' + str(i) + ' Mode = ' + self.mode.get())
                self.control_state[i] =  False
                button.configure(style="control_off.TButton")
                msg = MsgCmdControl(MsgType.MSG_GUI_TO_SERVER_CONTROL_OFF, i, self.control_data[i])
                self.output_queue.put(msg)
            else:
                print('Call control on ' + str(i) + ' Mode = ' + self.mode.get())
                self.control_state[i] =  True
                button.configure(style="control_on.TButton")
                msg = MsgCmdControl(MsgType.MSG_GUI_TO_SERVER_CONTROL_ON, i, self.control_data[i])
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