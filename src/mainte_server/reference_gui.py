#!/usr/bin/env python3
# coding: utf-8

import tkinter
from tkinter import ttk
from tkinter import filedialog
from ttkthemes import ThemedTk
import yaml
from canvas_gui import *

NUM_JOINT = 4

class Reference(ttk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.master.title('reference')
        # define parameter list
        self.reference_list = [{'label' : 'Joint' + str(i),
                                'type' : tkinter.StringVar(),
                                'amp' : tkinter.StringVar(),
                                'base' : tkinter.StringVar(),
                                'freq' : tkinter.StringVar(),
                                'phase' : tkinter.StringVar()}  for i in range(NUM_JOINT)]
        for reference in self.reference_list:
            reference['amp'].set('0.0')
            reference['base'].set('0.0')
            reference['freq'].set('0.0')
            reference['phase'].set('0.0')
        self.canvas_value_list = [[0 for i in range(10)] for j in range(NUM_JOINT)]
        self.canvas = [None for i in range(NUM_JOINT)]
        self.create_widgets()
        self.pack()

    def create_widgets(self):
        # frame
        self.option_frame = ttk.Frame(self)
        self.reference_frame = ttk.Frame(self)
        self.menu_frame =  ttk.Frame(self)
        # create option_frame widgets
        self.open_button = ttk.Button(self.option_frame, text='Open', command=self.open_callback, width=7)
        self.save_button = ttk.Button(self.option_frame, text='Save', command=self.save_callback, width=7)
        self.open_button.grid(row = 0, column = 0)
        self.save_button.grid(row = 0, column = 1)
        #set option_frame widgets
        self.option_frame.pack(anchor = tkinter.E)
        # create reference_frame widgets
        wave_form_labels = ['Type', 'Amp', 'Base', 'Freq', 'Phase']
        for i in range(len(wave_form_labels)):
            label = ttk.Label(self.reference_frame, text=wave_form_labels[i])
            label.grid(row = i + 1, column = 0)
        wave_type_labels = ['Const', 'Sin' ,'Rect', 'Tri', 'Canvas']
        for i in range(NUM_JOINT):
            label = ttk.Label(self.reference_frame, text=self.reference_list[i]['label'])
            wave_type_combo = ttk.Combobox(self.reference_frame, textvariable=self.reference_list[i]['type'], value=wave_type_labels, state="readonly", width=7)
            wave_type_combo.current(0)
            amp_spin = ttk.Spinbox(self.reference_frame, textvariable=self.reference_list[i]['amp'], from_=0.0,to=180.0,increment=0.1, width=7)
            base_spin = ttk.Spinbox(self.reference_frame, textvariable=self.reference_list[i]['base'], from_=-180.0,to=180.0,increment=0.1, width=7)
            freq_spin = ttk.Spinbox(self.reference_frame, textvariable=self.reference_list[i]['freq'], from_=0.0,to=20.0,increment=0.1, width=7)
            phase_spin = ttk.Spinbox(self.reference_frame, textvariable=self.reference_list[i]['phase'], from_=-180.0,to=180.0,increment=0.1, width=7)
            label.grid(row = 0, column = i + 1)
            wave_type_combo.grid(row = 1, column = i + 1)
            amp_spin.grid(row = 2, column = i + 1)
            base_spin.grid(row = 3, column = i + 1)
            freq_spin.grid(row = 4, column = i + 1)
            phase_spin.grid(row = 5, column = i + 1)
            canvas_button = ttk.Button(self.reference_frame, text='Canvas'+str(i), command=self.canvas_callback(i),width=7)
            canvas_button.grid(row = 6, column = i + 1)
        # set reference_frame widgets
        self.reference_frame.pack()
        # create menu_frame widgets
        self.cancel_button = ttk.Button(self.menu_frame, text='Cancel', command=self.cancal_callback, width=7)
        self.ok_button = ttk.Button(self.menu_frame, text='OK', command=self.ok_callback, width=7)
        self.cancel_button.grid(row = 0, column = 0)
        self.ok_button.grid(row = 0, column = 1)
        #set menu_frame widgets
        self.menu_frame.pack(anchor = tkinter.E)

    # Callback Function --->>>
    def open_callback(self):
        print('call open')
        type = [('Refernce File','*.yml')] 
        reference_file = filedialog.askopenfilename(filetypes = type) 
        with open(reference_file, 'r') as file:
            obj = yaml.safe_load(file)
            for i in range(NUM_JOINT):
                self.reference_list[i]['type'].set(obj[self.reference_list[i]['label']]['type'])
                self.reference_list[i]['amp'].set(obj[self.reference_list[i]['label']]['amp'])
                self.reference_list[i]['base'].set(obj[self.reference_list[i]['label']]['base'])
                self.reference_list[i]['freq'].set(obj[self.reference_list[i]['label']]['freq'])
                self.reference_list[i]['phase'].set(obj[self.reference_list[i]['label']]['phase'])
                self.canvas_value_list[i] = obj[self.reference_list[i]['label']]['canvas']
        print(reference_file)

    def save_callback(self):
        print('call save')
        dict = {}
        for i in range(NUM_JOINT):
            label = self.reference_list[i]['label']
            type = self.reference_list[i]['type'].get()
            amp = self.reference_list[i]['amp'].get()
            base = self.reference_list[i]['base'].get()
            freq = self.reference_list[i]['freq'].get()
            phase = self.reference_list[i]['phase'].get()
            dict[label] = {'type' : type, 'amp' : amp, 'base' : base, 'freq' : freq, 'phase' : phase}
            if self.canvas[i] != None:
                dict[label]['canvas'] = self.canvas[i].get_points()
        type = [('Refernce File','*.yml')] 
        reference_file = filedialog.asksaveasfilename(filetypes = type)
        print(reference_file)
        with open(reference_file, 'w') as file:
            yaml.dump(dict, file)

    def ok_callback(self):
        print('call ok')
        self.master.destroy()

    def cancal_callback(self):
        print('call cancel')
        self.master.destroy()

    def canvas_callback(self, i):
        print('call canvas')
        def x():
            dialog = tkinter.Toplevel(self)
            self.canvas[i] = Canvas(dialog, 'Canvas'+str(i), self.canvas_value_list[i])
        return x
    # <<<--- Callback Function

if __name__ == '__main__':
    root = ThemedTk(theme='radiance')
    app = Reference(master=root)
    app.mainloop()