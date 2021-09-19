#!/usr/bin/env python3
# coding: utf-8

import tkinter
from tkinter import ttk
import numpy as np
from scipy import interpolate

class Canvas(ttk.Frame):
    def __init__(self, master=None, title='reference canvas'):
        super().__init__(master)
        self.master = master
        self.master.title(title)
        self.width = 600
        self.height = 600
        self.create_widgets()
        self.pack()
        self.x_range = {'min' : 0.0, 'max' : int(self.num_point.get()) - 1, 'tick' : 1}
        self.y_range = {'min' : -90.0, 'max' : 90.0, 'tick' : 30.0}
        self.draw_grid()
        self.points = [{'id' : i, 'value' : 0.0} for i in range(int(self.num_point.get()))]
        self.draw_point()
        self.draw_spline()

    def create_widgets(self):
        # frame
        self.canvas_frame = tkinter.Frame(self, bg='white')
        self.menu_frame =  tkinter.Frame(self, bg='white')
        # create canvas_frame widgets
        self.main_canvas = tkinter.Canvas(self.canvas_frame, bg='white', width=self.width, height=self.height, highlightthickness=0)
        # set canvas_frame widgets
        self.main_canvas.grid(row = 0, column = 0)
        self.canvas_frame.pack()
        # create menu_frame widgets
        self.num_point = tkinter.StringVar()
        self.num_point.set('10')
        self.num_point_spin = ttk.Spinbox(self.menu_frame, textvariable=self.num_point, from_=1, to=100, increment=1, command=self.num_point_callback, width=5)
        self.clear_button = ttk.Button(self.menu_frame, text='clear', command=self.clear_callback, width=5)
        self.cancel_button = ttk.Button(self.menu_frame, text='cancel', command=self.cancal_callback, width=5)
        self.ok_button = ttk.Button(self.menu_frame, text='ok', command=self.ok_callback, width=5)
        #set menu_frame widgets
        self.num_point_spin.grid(row = 0, column = 0)
        self.clear_button.grid(row = 0, column = 1)
        self.cancel_button.grid(row = 0, column = 2)
        self.ok_button.grid(row = 0, column = 3)
        self.menu_frame.pack(side="right")

    def draw_grid(self):
        num_x_tick = int((self.x_range['max'] - self.x_range['min']) / self.x_range['tick']) + 1
        num_y_tick = int((self.y_range['max'] - self.y_range['min']) / self.y_range['tick']) + 1
        for i in range(num_x_tick):
            x_tick = self.x_range['tick'] * i + self.x_range['min']
            x_canvas = self.x_to_canvas(x_tick)
            self.main_canvas.create_line(x_canvas, 0, x_canvas, self.height, width=1.0, fill='gray')
        for i in range(num_y_tick):
            y_tick = self.y_range['tick'] * i + self.y_range['min']
            y_canvas = self.y_to_canvas(y_tick)
            self.main_canvas.create_line(0, y_canvas, self.width, y_canvas, width=1.0, fill='gray')

    def draw_point(self):
        num_x_tick = int((self.x_range['max'] - self.x_range['min']) / self.x_range['tick']) + 1
        radius = 5
        x = [self.x_range['tick'] * i + self.x_range['min'] for i in range(num_x_tick)]
        for i in range(num_x_tick):
            x_canvas = self.x_to_canvas(x[i])
            y_canvas = self.y_to_canvas(self.points[i]['value'])
            id = self.main_canvas.create_oval(x_canvas - radius, y_canvas - radius, x_canvas + radius, y_canvas + radius, fill='red')
            self.points[i]['id'] = id
            self.main_canvas.tag_bind(id, '<ButtonPress-1>', self.start)
            self.main_canvas.tag_bind(id, '<B1-Motion>', self.move)
            self.main_canvas.bind(id, '<ButtonRelease-1>', self.end)

    def draw_spline(self):
        num_x_tick = int((self.x_range['max'] - self.x_range['min']) / self.x_range['tick']) + 1
        self.main_canvas.delete("spline")
        x = [self.x_range['tick'] * i + self.x_range['min'] for i in range(num_x_tick)]
        y = [self.points[i]['value'] for i in range(num_x_tick)]
        f = interpolate.interp1d(x, y,kind="cubic")
        num_spline = 200
        for i in range(1, num_spline):
            x_pre = (self.x_range['max'] - self.x_range['min']) / num_spline * (i - 1)
            y_pre = f(x_pre)
            x_now = (self.x_range['max'] - self.x_range['min']) / num_spline * i
            y_now = f(x_now)
            self.main_canvas.create_line(self.x_to_canvas(x_pre), self.y_to_canvas(y_pre), self.x_to_canvas(x_now), self.y_to_canvas(y_now), width=1.0, fill='red', tag="spline")

    # Convert Function --->>>
    def x_to_canvas(self, x):
        return self.width / (self.x_range['max'] - self.x_range['min']) * (x - self.x_range['min'])

    def y_to_canvas(self, y):
        return self.height / (self.y_range['min'] - self.y_range['max']) * (y - self.y_range['max'])

    def x_to_value(self, x):
        return (self.x_range['max'] - self.x_range['min']) / self.width * x + self.x_range['min']

    def y_to_value(self, y):
        return (self.y_range['min'] - self.y_range['max']) / self.height * y + self.y_range['max']

    def id_to_index(self, id):
        ret = [i for i, point in enumerate(self.points) if point['id'] == id[0]]
        return ret[0]
    # <<<--- Convert Function

    # Callback Function --->>>
    def num_point_callback(self):
        self.main_canvas.delete(tkinter.ALL)
        self.x_range = {'min' : 0.0, 'max' : int(self.num_point.get()) - 1, 'tick' : 1}
        self.y_range = {'min' : -90.0, 'max' : 90.0, 'tick' : 30.0}
        self.draw_grid()
        new_points = [{'id' : i, 'value' : 0.0} for i in range(int(self.num_point.get()))]
        for i in range(int(self.num_point.get())):
            if i < len(self.points):
                new_points[i]['value'] = self.points[i]['value']
        self.points = new_points
        self.draw_point()
        self.draw_spline()

    def ok_callback(self):
        print('call ok')
        num_x_tick = int((self.x_range['max'] - self.x_range['min']) / self.x_range['tick']) + 1
        x = [self.x_range['tick'] * i + self.x_range['min'] for i in range(num_x_tick)]
        y = [self.points[i]['value'] for i in range(num_x_tick)]
        f = interpolate.interp1d(x, y,kind="cubic")
        num_spline = 200
        y_list = []
        for i in range(num_spline):
            x_value = (self.x_range['max'] - self.x_range['min']) / num_spline * i
            y_value = f(x_value)
            y_list.append(y_value)
            print(str(y_value) + ',', end="")
        #self.master.destroy()

    def cancal_callback(self):
        print('call cancel')
        self.master.destroy()

    def clear_callback(self):
        print('call clear')
        self.main_canvas.delete(tkinter.ALL)
        self.x_range = {'min' : 0.0, 'max' : int(self.num_point.get()) - 1, 'tick' : 1}
        self.y_range = {'min' : -90.0, 'max' : 90.0, 'tick' : 30.0}
        self.draw_grid()
        self.points = [{'id' : i, 'value' : 0.0} for i in range(int(self.num_point.get()))]
        self.draw_point()
        self.draw_spline()
    # <<<--- Callback Function

    # Interact Function --->>>
    def start(self, event):
        self.id = self.main_canvas.find_closest(event.x, event.y)
        self.x = event.x
        self.y = event.y

    def move(self, event):
        self.main_canvas.move(self.id, 0 ,event.y - self.y)
        self.x = event.x
        self.y = event.y
        radius = 5
        self.points[self.id_to_index(self.id)]['value'] = self.y_to_value(event.y + radius)
        self.draw_spline()

    def end(self, event):
        self.x = None
        self.y = None
    # <<<--- Interact Function

if __name__ == '__main__':
    root = tkinter.Tk()
    app = Application(master=root)
    app.mainloop()