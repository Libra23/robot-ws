#!/usr/bin/env python3
# coding: utf-8

import tkinter

class Application(tkinter.Frame):
    def __init__(self, master=None):
        super().__init__(master)
        self.master = master
        self.master.title('tkinter canvas trial')
        self.pack()
        self.create_widgets()
        self.setup()

    def create_widgets(self):
        self.vr = tkinter.IntVar()
        self.vr.set(1)
        self.write_radio = tkinter.Radiobutton(self, text='write', variable=self.vr, value=1, command=self.change_radio)
        self.write_radio.grid(row=0, column=0)
        self.erase_radio = tkinter.Radiobutton(self, text='erase', variable=self.vr, value=2, command=self.change_radio)
        self.erase_radio.grid(row=0, column=1)

        self.clear_button = tkinter.Button(self, text='clear all', command=self.clear_canvas)
        self.clear_button.grid(row=0, column=2)

        self.save_button = tkinter.Button(self, text='save', command=self.save_canvas)
        self.save_button.grid(row=0, column=3)

        self.test_canvas = tkinter.Canvas(self, bg='white', width=600, height=600)
        self.test_canvas.grid(row=1, column=0, columnspan=4)
        self.test_canvas.bind('<B1-Motion>', self.paint)
        self.test_canvas.bind('<ButtonRelease-1>', self.reset)

    def setup(self):
        self.old_x = None
        self.old_y = None
        self.color = 'black'
        self.eraser_on = False

    def change_radio(self):
        if self.vr.get() == 1:
            self.eraser_on = False
        else:
            self.eraser_on = True

    def clear_canvas(self):
        self.test_canvas.delete(tkinter.ALL)

    def save_canvas(self):
        self.test_canvas.postscript(file='out.ps', colormode='color')

    def paint(self, event):
        if self.eraser_on:
            paint_color = 'white'
        else:
            paint_color = 'black'
        if self.old_x and self.old_y:
            self.test_canvas.create_line(self.old_x, self.old_y, event.x, event.y, width=5.0, fill=paint_color, capstyle=tkinter.ROUND, smooth=tkinter.TRUE, splinesteps=36)
        self.old_x = event.x
        self.old_y = event.y

    def reset(self, event):
        self.old_x, self.old_y = None, None

if __name__ == '__main__':
    root = tkinter.Tk()
    app = Application(master=root)
    app.mainloop()