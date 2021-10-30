#!/usr/bin/env python3
# coding: utf-8

import tkinter
from tkinter import ttk
from tkinter import filedialog
from ttkthemes import ThemedTk
import yaml
from canvas_gui import *

NUM_JOINT = 4

class ReferenceGui(ttk.Frame):
    def __init__(self, master, mode):
        super().__init__(master)
        # init parameter
        self.master = master
        self.mode = mode
        
        self.canva_frame = [None for i in range(4)]
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
        # prepare frame
        self.master.title('reference')
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
        wave_type_labels = ['Const', 'Sin' ,'Rect', 'Tri', 'Canvas']
        for i in range(NUM_JOINT):
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
            canvas_button = ttk.Button(reference_frame, text='Canvas'+str(i), command=self.canvas_callback(i),width=7)
            canvas_button.grid(row = 6, column = i + 1)
        # set reference_frame widgets
        reference_frame.pack()
        # create menu_frame widgets
        cancel_button = ttk.Button(menu_frame, text='Cancel', command=self.cancal_callback, width=7)
        ok_button = ttk.Button(menu_frame, text='OK', command=self.ok_callback, width=7)
        cancel_button.grid(row = 0, column = 0)
        ok_button.grid(row = 0, column = 1)
        #set menu_frame widgets
        menu_frame.pack(anchor = tkinter.E)

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
                y = self.canvas_value_list[i]
                x = [i for i in range(len(y))]
                f = interpolate.interp1d(x, y,kind="cubic",fill_value="extrapolate")
                num_spline = 100
                for i in range(num_spline - 1):
                    x_value = len(x) / num_spline * i
                    y_value = f(x_value)
                    print('{:.3f}, '.format(y_value), end="")
                print("\n")
        #print(reference_file)

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
    app = ReferenceGui(master=root, mode='FK')
    app.mainloop()