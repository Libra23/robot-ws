#!/usr/bin/env python3
# coding: utf-8

import tkinter
from tkinter import ttk
from ttkthemes import ThemedTk
from canvas_gui import *

NUM_ARM = 4

class MainteServerGui(ttk.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.master.title('MainteServer')
        self.create_widgets()
        self.pack()

    def create_widgets(self):
        # frame
        self.main_frame = ttk.Frame(self, bg='white')
        self.menu_frame =  ttk.Frame(self, bg='white')
        # create main widgets
        wave_form_labels = ['Type', 'Amp', 'Base', 'Freq', 'Phase']
        for i in range(len(wave_form_labels)):
            label = ttk.Label(self.reference_frame, text=wave_form_labels[i])
            label.grid(row = i + 1, column = 0)
        wave_type_labels = ['Const', 'Sin' ,'Rect', 'Tri', 'Canvas']
        for i in range(NUM_JOINT):
            label = ttk.Label(self.reference_frame, text='Joint'+str(i))
            label.grid(row = 0, column = i + 1)
            wave_type_combo = ttk.Combobox(self.reference_frame, textvariable=self.wave_type_list[i], value=wave_type_labels, state="readonly", width=7)
            wave_type_combo.current(0)
            self.amp_list[i].set('0.0')
            amp_spin = ttk.Spinbox(self.reference_frame, textvariable=self.amp_list[i], from_=0.0,to=180.0,increment=0.1, width=7)
            self.base_list[i].set('0.0')
            base_spin = ttk.Spinbox(self.reference_frame, textvariable=self.base_list[i], from_=-180.0,to=180.0,increment=0.1, width=7)
            self.freq_list[i].set('0.0')
            freq_spin = ttk.Spinbox(self.reference_frame, textvariable=self.freq_list[i], from_=0.0,to=20.0,increment=0.1, width=7)
            self.phase_list[i].set('0.0')
            phase_spin = ttk.Spinbox(self.reference_frame, textvariable=self.phase_list[i], from_=-180.0,to=180.0,increment=0.1, width=7)
            wave_type_combo.grid(row = 1, column = i + 1)
            amp_spin.grid(row = 2, column = i + 1)
            base_spin.grid(row = 3, column = i + 1)
            freq_spin.grid(row = 4, column = i + 1)
            phase_spin.grid(row = 5, column = i + 1)
            canvas_button = ttk.Button(self.reference_frame, text='Canvas'+str(i), width=7)
            canvas_button.bind('<Button-1>', self.canvas_callback)
            canvas_button.grid(row = 6, column = i + 1)
        # set reference_frame widgets
        self.reference_frame.pack()
        # create menu_frame widgets
        self.cancel_button = ttk.Button(self.menu_frame, text='Cancel', command=self.cancal_callback, width=7)
        self.ok_button = ttk.Button(self.menu_frame, text='OK', command=self.ok_callback, width=7)
        self.cancel_button.grid(row = 0, column = 0)
        self.ok_button.grid(row = 0, column = 1)
        #set menu_frame widgets
        self.menu_frame.pack(side="right")

    # Callback Function --->>>
    def ok_callback(self):
        print('call ok')
        self.master.destroy()

    def cancal_callback(self):
        print('call cancel')
        self.master.destroy()

    def canvas_callback(self, event):
        print('call canvas')
        self.dialog = tkinter.Toplevel(self)
        self.canvas = Canvas(self.dialog, event.widget['text'])
        self.dialog.grab_set()
    # <<<--- Callback Function

if __name__ == '__main__':
    root = ThemedTk(theme='radiance')
    app = Reference(master=root)
    app.mainloop()