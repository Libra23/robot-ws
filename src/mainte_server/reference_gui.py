#!/usr/bin/env python3
# coding: utf-8

import tkinter
from tkinter import ttk
from tkinter import filedialog
from ttkthemes import ThemedTk
import yaml
from mainte_data import *

class ReferenceGui(ttk.Frame):
    def __init__(self, master, mode, arm_index, num_joint_per_arm, reference):
        super().__init__(master)
        # init parameter
        self.mode = mode
        self.num_joint = num_joint_per_arm
        self.reference = reference
        # init frame parameter
        labels = ['Joint' + str(i) for i in range(self.num_joint)]
        if self.mode == 'IK':
            labels = ['X', 'Y', 'Z', 'Roll', 'Pitch', 'Yaw']
        self.reference_list = [{'label' : label,
                                'type' : tkinter.StringVar(),
                                'amp' : tkinter.StringVar(),
                                'base' : tkinter.StringVar(),
                                'freq' : tkinter.StringVar(),
                                'phase' : tkinter.StringVar()}  for label in labels]
        for reference in self.reference_list:
            reference['amp'].set('0.0')
            reference['base'].set('0.0')
            reference['freq'].set('0.0')
            reference['phase'].set('0.0')
        # prepare frame
        master.title('Reference' + str(arm_index))
        self.create_widgets()
        self.pack()

    def create_widgets(self):
        # frame
        option_frame = ttk.Frame(self)
        reference_frame = ttk.Frame(self)
        menu_frame =  ttk.Frame(self)
        # create option_frame widgets
        mode_label = ttk.Label(option_frame, text = 'Control Mode : ' + self.mode)
        mode_label.grid(row = 0, column = 0)
        open_button = ttk.Button(option_frame, text='Open', command=self.open_callback, width=7)
        open_button.grid(row = 0, column = 1)
        save_button = ttk.Button(option_frame, text='Save', command=self.save_callback, width=7)
        save_button.grid(row = 0, column = 2)
        #set option_frame widgets
        option_frame.pack(anchor = tkinter.E)
        # create reference_frame widgets
        wave_form_labels = ['Type', 'Amp', 'Base', 'Freq', 'Phase']
        for i in range(len(wave_form_labels)):
            label = ttk.Label(reference_frame, text=wave_form_labels[i])
            label.grid(row = i + 1, column = 0)
        wave_type_labels = [type.name for type in WaveType]
        for i in range(len(self.reference_list)):
            label = ttk.Label(reference_frame, text=self.reference_list[i]['label'])
            wave_type_combo = ttk.Combobox(reference_frame, textvariable=self.reference_list[i]['type'], value=wave_type_labels, state="readonly", width=7)
            wave_type_combo.current(0)
            amp_spin = ttk.Spinbox(reference_frame, textvariable=self.reference_list[i]['amp'], from_=0.0,to=180.0,increment=0.1, width=7)
            base_spin = ttk.Spinbox(reference_frame, textvariable=self.reference_list[i]['base'], from_=-180.0,to=180.0,increment=0.1, width=7)
            freq_spin = ttk.Spinbox(reference_frame, textvariable=self.reference_list[i]['freq'], from_=0.0,to=20.0,increment=0.1, width=7)
            phase_spin = ttk.Spinbox(reference_frame, textvariable=self.reference_list[i]['phase'], from_=-180.0,to=180.0,increment=0.1, width=7)
            label.grid(row = 0, column = i + 1)
            wave_type_combo.grid(row = 1, column = i + 1)
            amp_spin.grid(row = 2, column = i + 1)
            base_spin.grid(row = 3, column = i + 1)
            freq_spin.grid(row = 4, column = i + 1)
            phase_spin.grid(row = 5, column = i + 1)
        # set reference_frame widgets
        reference_frame.pack()
        # create menu_frame widgets
        cancel_button = ttk.Button(menu_frame, text='Cancel', command=self.cancal_callback, width=7)
        ok_button = ttk.Button(menu_frame, text='OK', command=self.ok_callback, width=7)
        cancel_button.grid(row = 0, column = 0)
        ok_button.grid(row = 0, column = 1)
        #set menu_frame widgets
        menu_frame.pack(anchor = tkinter.E)

    def get_reference(self):
        return self.reference

    # Callback Function --->>>
    def open_callback(self):
        print('call open')
        type = [('Refernce File','*.yml')] 
        reference_file = filedialog.askopenfilename(filetypes = type) 
        with open(reference_file, 'r') as file:
            obj = yaml.safe_load(file)
            for i in range(len(self.reference_list)):
                label = self.reference_list[i]['label']
                self.reference_list[i]['type'].set(obj[self.mode][label]['type'])
                self.reference_list[i]['amp'].set(obj[self.mode][label]['amp'])
                self.reference_list[i]['base'].set(obj[self.mode][label]['base'])
                self.reference_list[i]['freq'].set(obj[self.mode][label]['freq'])
                self.reference_list[i]['phase'].set(obj[self.mode][label]['phase'])

    def save_callback(self):
        print('call save')
        data = {}
        for i in range(len(self.reference_list)):
            label = self.reference_list[i]['label']
            type = self.reference_list[i]['type'].get()
            amp = self.reference_list[i]['amp'].get()
            base = self.reference_list[i]['base'].get()
            freq = self.reference_list[i]['freq'].get()
            phase = self.reference_list[i]['phase'].get()
            data[label] = {'type' : type, 'amp' : amp, 'base' : base, 'freq' : freq, 'phase' : phase}
        file_type = [('Refernce File','*.yml')] 
        reference_file = filedialog.asksaveasfilename(filetypes = file_type)
        print(reference_file)
        with open(reference_file, 'w') as file:
            out = {}
            out[self.mode] = data
            yaml.dump(out, file)

    def ok_callback(self):
        print('Call ok')
        if self.mode == 'FK':
            for i in range(len(self.reference_list)):
                self.reference.fk[i].type = WaveType[self.reference_list[i]['type'].get()]
                self.reference.fk[i].amplitude = float(self.reference_list[i]['amp'].get())
                self.reference.fk[i].base = float(self.reference_list[i]['base'].get())
                self.reference.fk[i].frequency = float(self.reference_list[i]['freq'].get())
                self.reference.fk[i].phase = float(self.reference_list[i]['phase'].get())
        elif self.mode == 'IK':
            for i in range(len(self.reference_list)):
                self.reference.ik[i].type = WaveType[self.reference_list[i]['type'].get()]
                self.reference.ik[i].amplitude = float(self.reference_list[i]['amp'].get())
                self.reference.ik[i].base = float(self.reference_list[i]['base'].get())
                self.reference.ik[i].frequency = float(self.reference_list[i]['freq'].get())
                self.reference.ik[i].phase = float(self.reference_list[i]['phase'].get())
        elif self.mode == 'ACT_FK':
            for i in range(len(self.reference_list)):
                self.reference.act_fk[i].type = WaveType[self.reference_list[i]['type'].get()]
                self.reference.act_fk[i].amplitude = float(self.reference_list[i]['amp'].get())
                self.reference.act_fk[i].base = float(self.reference_list[i]['base'].get())
                self.reference.act_fk[i].frequency = float(self.reference_list[i]['freq'].get())
                self.reference.act_fk[i].phase = float(self.reference_list[i]['phase'].get())
        self.master.destroy()

    def cancal_callback(self):
        print('Call cancel')
        self.master.destroy()
    # <<<--- Callback Function

if __name__ == '__main__':
    root = ThemedTk(theme='radiance')
    app = ReferenceGui(master=root, mode='FK')
    app.mainloop()