#!/usr/bin/env python3
# coding: utf-8

import tkinter
from tkinter import ttk
from ttkthemes import ThemedTk
from reference_gui import *
from mainte_data import *

class MainteServerGui(ttk.Frame):
    def __init__(self, master, num_arm, num_joint_per_arm):
        super().__init__(master)
        # init frame parameter
        self.master = master
        self.mode = tkinter.StringVar()
        self.enable_states = [[tkinter.BooleanVar(value = True) for j in range(num_joint_per_arm)] for i in range(num_arm)]
        self.reference_frame = [None for i in range(num_arm)]
        # prepare frame
        self.master.title('MainteServer')
        self.create_widgets(num_arm, num_joint_per_arm)
        self.pack()
        
    def create_widgets(self, num_arm, num_joint_per_arm):
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
        for i in range(num_arm):
            # create arm widgets
            arm_frame = ttk.Frame(main_frame)
            label = ttk.Label(arm_frame, text = 'Arm (' + str(i) + ')')
            label.grid(row = 0, column = 0)
            control_button = ttk.Button(arm_frame, text = 'Control', command = self.control_callback(i), width = 7)
            control_button.grid(row = 1, column = 0)
            for j in range(num_joint_per_arm):
                enable_button = ttk.Checkbutton(arm_frame, text = 'Joint(' + str(j) + ')', variable = self.enable_states[i][j])
                enable_button.grid(row = 1, column = j + 1)
            reference_button = ttk.Button(arm_frame, text = 'R', command = self.reference_callback(i), width = 1)
            reference_button.grid(row = 1, column = num_joint_per_arm + 1)
            arm_frame.pack()
            # create separator
            separator = ttk.Separator(main_frame)
            separator.pack(fill="both")
        main_frame.pack()

    # Callback Function --->>>
    def control_callback(self, i):
        def callback():
            print('call control ' + str(i))
            mode = ControlMode[self.mode.get()]
            print(mode)
        return callback

    def reference_callback(self, i):
        def callback():
            print('call reference ' + str(i))
            if self.reference_frame[i] == None or not self.reference_frame[i].winfo_exists():
                self.reference_frame[i] = tkinter.Toplevel(self)
                ReferenceGui(self.reference_frame[i], self.mode.get())
        return callback
    # <<<--- Callback Function

if __name__ == '__main__':
    root = ThemedTk(theme='radiance')
    app = MainteServerGui(master = root, num_arm = 4, num_joint_per_arm = 4)
    app.mainloop()