#!/usr/bin/env python3

""" Provide control of the Wireless Sensor device using a GUI.
"""

from __future__ import print_function, division

import sys
import binascii
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, BTLEException

import tkinter as tk
import threading
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

from OpenGL import GL
import OpenGL.GL.shaders
import pyopengltk
import time

# Roll, Pitch, Yaw
euler_angles = [ 0, 0, 0]

# Avoiding glitches in pyopengl-3.0.x and python3.4
def bytestr(s):
    return s.encode("utf-8") + b"\000"

def sign_extend(value, bits):
    sign_bit = 1 << (bits - 1)
    return (value & (sign_bit - 1)) - (value & sign_bit)

vshader_source = """
#version 120
attribute vec4 a_position;
attribute vec4 a_color;
uniform mat4 u_mvp_matrix;
varying vec4 g_color;
void main(void) {
  gl_Position = u_mvp_matrix * a_position;
  g_color = a_color;
}
"""

fshader_source = """
varying vec4 g_color;
void main(void) {
  gl_FragColor = g_color;
}
"""


def rotational_matrix(roll, pitch, yaw):
    s = np.sin(roll)
    c = np.cos(roll)
    am = np.array(((c, s, 0, 0), (-s, c, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)), np.float32)
    s = np.sin(yaw)
    c = np.cos(yaw)
    bm = np.array(((c, 0, -s, 0), (0, 1, 0, 0), (s, 0, c, 0), (0, 0, 0, 1)), np.float32)
    s = np.sin(pitch)
    c = np.cos(pitch)
    cm = np.array(((1, 0, 0, 0), (0, c, s, 0), (0, -s, c, 0), (0, 0, 0, 1)), np.float32)
    return np.dot(np.dot(cm, bm), am)
    #return np.dot(np.dot(am, bm), cm)


class ShaderFrame(pyopengltk.OpenGLFrame):

    def initgl(self):

        self.degrees_per_radian = 180.0 / np.pi ## 57.2957795
        vertex_data_attribute_size = 3
        self.vertex_data = np.array(
             [-0.5, -0.5,  0.5,   # front side lower left
              0.5, -0.5,  0.5,    # front side lower right
              0.5,  0.5,  0.5,    # front side upper right
             -0.5,  0.5,  0.5,    # front side upper left

             -0.5, -0.5, -0.5,    # rear side 
             -0.5,  0.5, -0.5,    # rear side 
              0.5,  0.5, -0.5,    # rear side 
              0.5, -0.5, -0.5,    # rear side 

             -0.5, -0.5,  0.5,    # left side 
             -0.5,  0.5,  0.5,    # left side 
             -0.5,  0.5, -0.5,    # left side 
             -0.5, -0.5, -0.5,    # left side 

              0.5, -0.5,  0.5,    # right side
              0.5, -0.5, -0.5,    # right side
              0.5,  0.5, -0.5,    # right side
              0.5,  0.5,  0.5,    # right side

             -0.5,  0.5,  0.5,    # top side
              0.5,  0.5,  0.5,    # top side
              0.5,  0.5, -0.5,    # top side
             -0.5,  0.5, -0.5,    # top side

             -0.5, -0.5,  0.5,    # bottom side
             -0.5, -0.5, -0.5,    # bottom side
              0.5, -0.5, -0.5,    # bottom side
              0.5, -0.5,  0.5],    # bottom side
              dtype=np.float32)


        self.color_data = np.array(
             [0.5,  0.5,  0.1,
              0.5,  0.5,  0.1,
              0.5,  0.5,  0.1,
              0.5,  0.5,  0.1,

              0.5,  0.1,  0.8,
              0.5,  0.1,  0.8,
              0.5,  0.1,  0.8,
              0.5,  0.1,  0.8,

              0.1,  0.5,  0.8,
              0.1,  0.5,  0.8,
              0.1,  0.5,  0.8,
              0.1,  0.5,  0.8,

              0.1,  0.7,  0.8,
              0.1,  0.7,  0.8,
              0.1,  0.7,  0.8,
              0.1,  0.7,  0.8,

              0.7,  0.1,  0.8,
              0.7,  0.1,  0.8,
              0.7,  0.1,  0.8,
              0.7,  0.1,  0.8,

              0.5,  0.9,  0.1,
              0.5,  0.9,  0.1,
              0.5,  0.9,  0.1,
              0.5,  0.9,  0.1],
              dtype=np.float32)
        if not hasattr(self, "shader"):
            self.shader = OpenGL.GL.shaders.compileProgram(
                OpenGL.GL.shaders.compileShader(vshader_source, GL.GL_VERTEX_SHADER),
                OpenGL.GL.shaders.compileShader(fshader_source, GL.GL_FRAGMENT_SHADER)
                )
            self.attr_position = GL.glGetAttribLocation(self.shader, bytestr('a_position'))
            self.attr_color = GL.glGetAttribLocation(self.shader, bytestr('a_color'))
            self.unif_mvp = GL.glGetUniformLocation(self.shader, bytestr('u_mvp_matrix'))
        # clear color buffer
        GL.glClearColor( 1.0, 1.0, 1.0, 1.0 )
        GL.glClear( GL.GL_COLOR_BUFFER_BIT )
        # Prepare a framebuffer for rendering
        self.buffers = GL.glGenBuffers(2)
        # Prepare viewport
        GL.glViewport(0, 0, 384, 384)

        # Upload vertex data to a buffer
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.buffers[0])
        vd = self.vertex_data.tobytes()
        GL.glBufferData(GL.GL_ARRAY_BUFFER, len(vd), vd, GL.GL_STATIC_DRAW)
        GL.glVertexAttribPointer(self.attr_position, vertex_data_attribute_size, GL.GL_FLOAT, GL.GL_FALSE, 0, None);
        GL.glEnableVertexAttribArray(self.attr_position);

        # Upoad color data to a buffer
        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.buffers[1])
        cd = self.color_data.tobytes()
        GL.glBufferData(GL.GL_ARRAY_BUFFER, len(cd), cd, GL.GL_STATIC_DRAW)
        GL.glVertexAttribPointer(self.attr_color, vertex_data_attribute_size, GL.GL_FLOAT, GL.GL_FALSE, 0, None)
        GL.glEnableVertexAttribArray(self.attr_color);

    def redraw(self):
        """Render a single frame"""
        global euler_angles

        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glUseProgram (self.shader)

        self.mvp = rotational_matrix(float(euler_angles[0]) / self.degrees_per_radian, float(euler_angles[1]) / self.degrees_per_radian, float(euler_angles[2]) / self.degrees_per_radian) 

        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.buffers[0])
        GL.glUniformMatrix4fv(self.unif_mvp, 1, GL.GL_FALSE, self.mvp)

        GL.glFrontFace(GL.GL_CCW)
        GL.glCullFace(GL.GL_BACK)
        GL.glEnable(GL.GL_CULL_FACE)

        start_pos = 0
        for i in range(6):
            GL.glDrawArrays( GL.GL_TRIANGLE_FAN, start_pos, 4)
            start_pos = start_pos + 4


class ScanDelegateFindAHRS(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.dev_addr = None

    def handleDiscovery(self, dev, isNewDev, isNewData):
        try:
            local_name = dev.getValueText(0x09)
            manufacturer_data = dev.getValueText(0xFF)
        except BTLEException as e:
            print(e)
            sys.exit(1)
        if local_name == 'AHRS' and manufacturer_data is not None and len(manufacturer_data) == 16 and manufacturer_data[0:4] == 'ffff':
            self.dev_addr = dev.addr
            print("Found AHRS at addr %s" % ( self.dev_addr ))

    def getAHRSAddr(self):
        return self.dev_addr


class ScanDelegatePassive(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleDiscovery(self, dev, isNewDev, isNewData):
        try:
            manufacturer_data = dev.getValue(255)
        except BTLEException as e:
            print(e)
            sys.exit(1) 
        if manufacturer_data is not None and len(manufacturer_data) == 8 and manufacturer_data[0:2] == b'\xff\xff': 
            #print("dev.addr %s data %s type %s len %d" % ( dev.addr , manufacturer_data.hex() , type(manufacturer_data), len(manufacturer_data)))
            euler_angles[0] = sign_extend(manufacturer_data[3] << 8 | manufacturer_data[2], 16)
            euler_angles[1] = sign_extend(manufacturer_data[5] << 8 | manufacturer_data[4], 16)
            euler_angles[2] = sign_extend(manufacturer_data[7] << 8 | manufacturer_data[6], 16)
            #print(f"roll {euler_angles[0]} pitch {euler_angles[1]} yaw {euler_angles[2]}")
            #console.setAHRSData( 0, euler_angles)

class AHRSDataFrame():
    """
    The Data Frame can hold a series of optional Data Items, Scales, Spinboxes, Checkboxes, and Buttons. 
    """
    def __init__(self, master, data_group_name, row, column, data_names=None, data_label_width=9, data_plot_names=None, scales=None, spinboxes=None, check_button_names=None, check_button_cmds=None, button_stack="horizontal", buttons=None, sticky=None):
        # Save master object
        self.master = master
        paddingx = 2
        paddingy = 2

        self.label_frame = tk.LabelFrame(master, text=data_group_name)
        if sticky:
            self.label_frame.grid(column=column, row=row, padx=paddingx, pady=paddingy, sticky=sticky)
        else:
            self.label_frame.grid(column=column, row=row, padx=paddingx, pady=paddingy)

        #print(f"{data_group_name} Row {row} Col {column}")
        #print(self.label_frame.grid_info())

        frame_row = 0
        frame_col = 0

        clearAxes = True
        def handler(event, self=self, clearAxes=clearAxes):
            return self._dataFrameHandler(event, clearAxes)
        self.label_frame.bind('<Button-1>', handler)

        clearAxes = False
        def handler(event, self=self, clearAxes=clearAxes):
            return self._dataFrameHandler(event, clearAxes)
        self.label_frame.bind('<Button-3>', handler)

        self.dataItems = []
        self.data_group_name = data_group_name
        if data_names:
            if check_button_names:
                # create separate frames for data and check buttons
                self.dataFrame = tk.Frame(self.label_frame)
                self.dataFrame.grid(column=frame_col, row=frame_row, padx=paddingx, pady=paddingy)
                frame_row = frame_row + 1
            else:
                self.dataFrame = self.label_frame

            row_first = row
            for data_name in data_names:
                if isinstance(data_name, (list)):
                    # List of separate DataItem objects
                    for dat in data_name:
                        row = row + 1
                        self.dataItems.append(AHRSDataItem(self.dataFrame, data_group_name, dat, data_label_width, row, column, data_plot_names))
                    row = row_first
                    column = column + 2
                else:
                    # These are related data items which are all updated at the same time.
                    row = row + 1
                    self.dataItems.append(AHRSDataItem(self.dataFrame, data_group_name, data_name, data_label_width, row, column, data_plot_names))

        if scales:
            self.scalesFrame = tk.Frame(self.label_frame)
            self.scalesFrame.grid(column=frame_col, row=frame_row, padx=paddingx, pady=paddingy)
            frame_row = frame_row + 1
            scale_row = 0
            for scale in scales:
                scale.create(frame=self.scalesFrame, row=scale_row)
                scale_row = scale_row + 1

        if spinboxes:
            self.spinboxesFrame = tk.Frame(self.label_frame)
            self.spinboxesFrame.grid(column=frame_col, row=frame_row, padx=paddingx, pady=paddingy)
            frame_row = frame_row + 1
            spinbox_row = 0
            for spinbox in spinboxes:
                spinbox.create(frame=self.spinboxesFrame, row=spinbox_row)
                spinbox_row = spinbox_row + 1

        if check_button_names or buttons:
            self.buttonFrame = tk.Frame(self.label_frame)
            self.buttonFrame.grid(column=frame_col, row=frame_row, padx=paddingx, pady=paddingy)
            frame_row = frame_row + 1
            brow = bcolumn = 0
            bcolumn_start = 0
            bcolumn_last = 3

        if check_button_names:
            self.checkButtonData = dict()
            anch = tk.CENTER
            bidx = 0
            for name in check_button_names:
                self.checkButtonData[name] = tk.StringVar()
                self.checkButtonData[name].set(0)
                if check_button_cmds:
                   button_cmd = check_button_cmds[bidx]
                   bidx = bidx + 1
                else:
                   button_cmd = None
                def buttonHandler(self=self, button_name=name, button_cmd=button_cmd):
                    return self._buttonHandler(button_name, button_cmd)
                tk.Checkbutton(self.buttonFrame, text=name, command=buttonHandler, variable=self.checkButtonData[name], indicatoron=1, anchor=anch).grid(column=bcolumn, row=brow, padx=paddingx, pady=paddingy)
                if button_stack == "horizontal":
                    bcolumn = bcolumn + 1
                    if bcolumn == bcolumn_last:
                        brow = brow + 1
                        bcolumn = bcolumn_start
                else:
                    brow = brow + 1

        if buttons:
            for button in buttons:
                button.create(frame=self.buttonFrame, row=brow, col=bcolumn)
                if button_stack == "horizontal":
                    bcolumn = bcolumn + 1
                    if bcolumn == bcolumn_last:
                        brow = brow + 1
                        bcolumn = bcolumn_start
                else:
                    brow = brow + 1

    def _dataFrameHandler(self, event, clearAxes):
        if clearAxes:
            self.ax.cla()
        self.setupPlot()
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value');
        self.ax.legend()
        self.clearPlot()

    def _buttonHandler(self, button_name, button_cmd):
        if not self.master.connected.get():
            print("Not connected to AHRS")
        elif button_cmd:
            self.master.writeCmdStr(button_cmd)
        else:
            self.master.writeCmdStr('IMU_SELECT_' + self.data_group_name.upper() )
            self.master.writeCmdStr('IMU_SENSOR_DATA_' + button_name.upper() + '_TOGGLE')

    def setButtonData(self, button_name, data):
        self.checkButtonData[button_name].set(data)

    def setData(self, idx, data):
        self.dataItems[idx].setData(data)

    def getData(self, idx):
        return self.dataItems[idx].getData()

    def setPlotAxes(self, ax):
        self.ax = ax
        for dItem in self.dataItems:
            dItem.setPlotAxes(ax)

    def setupPlot(self):
        # setup data plot for each data item
        for dItem in self.dataItems:
            dItem.setupPlot()

    def clearPlot(self):
        # setup data plot
        for dItem in self.dataItems:
            dItem.clearPlot()

class AHRSButton():
    def __init__(self, master, name, command):
        self.master = master
        self.name = name
        self.command = command
        # hard-coded
        self.paddingx = 2
        self.paddingy = 2
        if type(self.command) == str:
            self.handler = self._buttonHandler
        else:
            self.handler = self.command

    def _buttonHandler(self):
        if not self.master.connected.get():
            print("Not connected to AHRS")
        else:
            self.master.writeCmdStr(self.command)

    def create(self, frame=None, row=0, col=0):
        tk.Button(frame, text=self.name, command=self.handler).grid(column=col, row=row, padx=self.paddingx, pady=self.paddingy)

class AHRSScale():
    def __init__(self, name, command, var, orient, minValue, maxValue, resolution):
        self.name = name
        self.command = command
        self.var = var
        self.orient = orient
        self.minValue = minValue
        self.maxValue = maxValue
        self.resolution = resolution
        # hard-coded
        self.paddingx = 2
        self.paddingy = 2
        self.scale_width = 10
        self.scale_length = 150

    def create(self, frame=None, row=0, col=0):
        tk.Label(frame, text=self.name).grid(column=col, row=row, padx=self.paddingx, pady=self.paddingy)
        tk.Scale(frame, orient=self.orient, command=self.command, from_=self.minValue , to_=self.maxValue, resolution=self.resolution, variable=self.var, width=self.scale_width, length=self.scale_length).grid(column=col+1, row=row, padx=self.paddingx, pady=self.paddingy)

class AHRSSpinbox():
    def __init__(self, name, command, var, minValue, maxValue, increment, textformat):
        self.name = name
        self.command = command
        self.var = var
        self.minValue = minValue
        self.maxValue = maxValue
        self.increment = increment
        self.fmt = textformat
        # hard-coded
        self.paddingx = 2
        self.paddingy = 2
        self.spinbox_width = 10

    def create(self, frame=None, row=0, col=0):
        tk.Label(frame, text=self.name).grid(column=col, row=row, padx=self.paddingx, pady=self.paddingy)
        tk.Spinbox(frame, command=self.command, from_=self.minValue , to_=self.maxValue, increment=self.increment, textvariable=self.var, width=self.spinbox_width, format=self.fmt).grid(column=col+1, row=row, padx=self.paddingx, pady=self.paddingy)

class AHRSDataItem():
    def __init__(self, master, data_group_name, data_name, data_label_width, row, column, data_plot_names=None):
        self.data_name = data_name
        self.data_group_name = data_group_name
        self.data = tk.StringVar()
        self.data.set(0)
        self.plot_name = data_group_name + ' ' + data_name
        self.data_plot_names = data_plot_names
        self.dataFrame = master
        paddingx = 5
        paddingy = 5
        self.xpoints = 500
        col_start=column
        row_start=row
        tk.Label(self.dataFrame, text=data_name, justify=tk.LEFT).grid(column=col_start, row=row_start, padx=paddingx, pady=paddingy, sticky=tk.W)
        self.data_label = tk.Label(self.dataFrame, relief=tk.SUNKEN, textvariable=self.data, width=data_label_width)
        self.data_label.grid(column=col_start+1, row=row_start, padx=paddingx, pady=paddingy)

        clearAxes = True
        def handler(event, self=self, clearAxes=clearAxes):
            return self._labelHandler(event, clearAxes)
        self.data_label.bind('<Button-1>', handler)

        clearAxes = False
        def handler(event, self=self, clearAxes=clearAxes):
            return self._labelHandler(event, clearAxes)
        self.data_label.bind('<Button-3>', handler)

        # Initially no data available
        self.dataListLen = 0

        # account for a maximum of 3 data sub-items within a single data label (for example X, Y, Z).
        self.max_data_subitems = 3
        self.dataplot = np.zeros((self.max_data_subitems,self.xpoints))
        self.line = [None,None,None]
        self.dataplotidx = 0
        self.ax = None

    def _labelHandler(self, event, clearAxes):
        if clearAxes:
            self.ax.cla()
        self.setupPlot()
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value');
        self.ax.legend()
        self.clearPlot()

    def setData(self, data):
        self.data.set(data)
        if isinstance(data, (list)):
            self.dataListLen = len(data)
        else:
            self.dataListLen = 1
        if self.line[0]: 
            if isinstance(data, (list)):
                # go through the data sub-items.
                if self.dataplotidx == self.xpoints - 1:
                    for idx in range(self.dataListLen):
                        self.dataplot[idx] = np.roll(self.dataplot[idx], -1)
                else:
                   self.dataplotidx = self.dataplotidx + 1
                for idx in range(self.dataListLen):
                    self.dataplot[idx][self.dataplotidx] = float(data[idx])
                    if self.line[idx]:
                        self.line[idx].set_data(np.arange(self.xpoints), self.dataplot[idx])
            else:
                if self.dataplotidx == self.xpoints - 1:
                    self.dataplot[0] = np.roll(self.dataplot[0], -1)
                else:
                   self.dataplotidx = self.dataplotidx + 1
                self.dataplot[0][self.dataplotidx] = float(data)
                self.line[0].set_data(np.arange(self.xpoints), self.dataplot[0])

    def getData(self):
        return self.data

    def setPlotAxes(self, ax):
        self.ax = ax

    def clearPlot(self):
        if self.line[0]: 
            self.dataplot = np.zeros((self.max_data_subitems,self.xpoints))
            # go through the data sub-items.
            for idx in range(self.dataListLen):
                self.line[idx].set_xdata(np.arange(self.xpoints))
                self.line[idx].set_ydata(self.dataplot[idx])
        self.ax.relim()
        self.dataplotidx = 0
        self.ax.autoscale_view()

    def setupPlot(self):
        for idx in range(self.dataListLen):
            subDataItemLabel = self.plot_name
            # if self.dataListLen == 3:
            if self.dataListLen > 1:
                sub_data_name = None 
                if self.data_plot_names:
                    sub_data_name = self.data_plot_names[idx]
                elif idx == 0:
                    sub_data_name = 'X'
                elif idx == 1:
                    sub_data_name = 'Y'
                elif idx == 2:
                    sub_data_name = 'Z'
                if sub_data_name:
                    subDataItemLabel = subDataItemLabel + ' ' + sub_data_name 
            self.line[idx], = self.ax.plot(np.arange(self.xpoints), self.dataplot[idx], label=subDataItemLabel)

class AHRSConsole(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()

        self.connected = tk.BooleanVar()
        self.connected.set(False)
        self.connect_thread = None
        self.connect_button = None

        self.scanner_thread = None
        self.scanner = None

        self.calibrate = tk.IntVar()
        self.calibrate.set(0)

        self.twoKp = tk.DoubleVar()
        self.twoKi = tk.DoubleVar()
        self.sampleFreq = tk.DoubleVar()
        self.gyroCorrection = tk.DoubleVar()
        self.magnetometerStability = tk.IntVar()
        self.gyroscopeEnable = tk.IntVar()
        self.AHRSalgorithm = tk.IntVar()
        self.betaGain = tk.DoubleVar()
        self.dataRateHz = tk.DoubleVar()
        self.uncalibratedDisplay = tk.IntVar()
        self.settingsDisplay = tk.IntVar()

        # PID variables
        self.pidKP = tk.DoubleVar()
        self.pidKI = tk.DoubleVar()
        self.pidKD = tk.DoubleVar()
        self.pidSP = tk.DoubleVar()
        self.pidPV = tk.DoubleVar()
        self.pidOutput = tk.DoubleVar()

        self.pidKP2 = tk.DoubleVar()
        self.pidKI2 = tk.DoubleVar()
        self.pidKD2 = tk.DoubleVar()
        self.pidSP2 = tk.DoubleVar()
        self.pidPV2 = tk.DoubleVar()
        self.pidOutput2 = tk.DoubleVar()

        # Clock Select
        self.pwmClock = tk.IntVar()

        self.accelNoiseThreshold = tk.DoubleVar()

        self.resetAHRSSettings()
        self.xpoints = 500

        self.data_group = dict()
        self.dataplotcnt = 0

        self.dataplot_cnv = None
        self.line = []
        self.dataplot = []
        self.dataplotidx = 0
        for i in range(3):
            self.line.append(None)
            self.dataplot.append(np.zeros(self.xpoints))

        self.createWidgets()
        self.peripheral = None
        self.dev_addr = None

        # Command dictionary

        self.commandDict = dict() 
        self.commandDict['IMU_NOCMD'] = 0
        self.commandDict['IMU_SELECT_MAGNETOMETER'] = 1
        self.commandDict['IMU_SELECT_GYROSCOPE'] = 2
        self.commandDict['IMU_SELECT_ACCELEROMETER'] = 3
        # AHRS and Quaternion overloaded
        self.commandDict['IMU_SELECT_AHRS'] = 4
        self.commandDict['IMU_SELECT_QUATERNION'] = 4
        #
        self.commandDict['IMU_SENSOR_CALIBRATE_NORMALIZED'] = 5
        self.commandDict['IMU_SENSOR_CALIBRATE_ZERO_OFFSET'] = 6
        self.commandDict['IMU_SENSOR_CALIBRATE_MAGNETOMETER'] = 7
        self.commandDict['IMU_SENSOR_CALIBRATE_GYROSCOPE'] = 8
        self.commandDict['IMU_SENSOR_CALIBRATE_RESET'] = 9

        self.commandDict['IMU_SENSOR_CALIBRATE_SAVE'] = 10 
        #self.commandDict['IMU_AHRS_INPUT_TOGGLE'] = 11
        #self.commandDict['IMU_AHRS_YAW_TOGGLE'] = 12
        #self.commandDict['IMU_AHRS_PITCH_TOGGLE'] = 13
        #self.commandDict['IMU_AHRS_ROLL_TOGGLE'] = 14
        self.commandDict['IMU_SENSOR_DATA_HOLD_TOGGLE'] = 11
        self.commandDict['IMU_SENSOR_DATA_IDEAL_TOGGLE'] = 12
        self.commandDict['IMU_SENSOR_DATA_FIXED_TOGGLE'] = 13
        self.commandDict['IMU_SENSOR_DATA_DISPLAY_TOGGLE'] = 14
        self.commandDict['IMU_AHRS_PROP_GAIN_UP'] = 15
        self.commandDict['IMU_AHRS_PROP_GAIN_DOWN'] = 16
        self.commandDict['IMU_AHRS_INTEG_GAIN_UP'] = 17
        self.commandDict['IMU_AHRS_INTEG_GAIN_DOWN'] = 18
        self.commandDict['IMU_AHRS_SAMPLE_FREQ_UP'] = 19

        self.commandDict['IMU_AHRS_SAMPLE_FREQ_DOWN'] = 20
        self.commandDict['IMU_GYROSCOPE_CORRECTION_UP'] = 21
        self.commandDict['IMU_GYROSCOPE_CORRECTION_DOWN'] = 22
        self.commandDict['IMU_MAGNETOMETER_STABILITY_TOGGLE'] = 23
        self.commandDict['IMU_AHRS_BETA_GAIN_UP'] = 24
        self.commandDict['IMU_AHRS_BETA_GAIN_DOWN'] = 25
        self.commandDict['IMU_AHRS_ALGORITHM_TOGGLE'] = 26
        self.commandDict['IMU_GYROSCOPE_ENABLE_TOGGLE'] = 27
        self.commandDict['IMU_UNCALIBRATED_DISPLAY_TOGGLE'] = 28
        self.commandDict['IMU_SETTINGS_DISPLAY_TOGGLE'] = 29

        self.commandDict['IMU_SELECT_ODR'] = 30
        self.commandDict['NOISE_THRESHOLD_UP'] = 31
        self.commandDict['NOISE_THRESHOLD_DOWN'] = 32

        # Motor Driver Settings
        self.commandDict['MOTOR_DRIVER_NOCMD'] = 33
        self.commandDict['MOTOR_DRIVER_TOGGLE_POWER'] = 34
        self.commandDict['MOTOR_DRIVER_TOGGLE_DISPLAY'] = 35
        self.commandDict['PWM_CLOCK_UP'] = 36
        self.commandDict['PWM_CLOCK_DOWN'] = 37

        # PID settings 
        self.commandDict['PID_NOCMD'] = 38
        self.commandDict['PID_KP_UP'] = 39
        self.commandDict['PID_KP_DOWN'] = 40
        self.commandDict['PID_KI_UP'] = 41
        self.commandDict['PID_KI_DOWN'] = 42
        self.commandDict['PID_KD_UP'] = 43
        self.commandDict['PID_KD_DOWN'] = 44
        self.commandDict['PID_SP_UP'] = 45
        self.commandDict['PID_SP_DOWN'] = 46 
        self.commandDict['PID_PARAM_SAVE'] = 47
        self.commandDict['PID_PARAM_ERASE'] = 48

        # PID settings 
        self.commandDict['PID_NOCMD'] = 49
        self.commandDict['PID2_KP_UP'] = 50
        self.commandDict['PID2_KP_DOWN'] = 51
        self.commandDict['PID2_KI_UP'] = 52
        self.commandDict['PID2_KI_DOWN'] = 53
        self.commandDict['PID2_KD_UP'] = 54
        self.commandDict['PID2_KD_DOWN'] = 55
        self.commandDict['PID2_SP_UP'] = 56
        self.commandDict['PID2_SP_DOWN'] = 57
        self.commandDict['PID2_PARAM_SAVE'] = 58
        self.commandDict['PID2_PARAM_ERASE'] = 59

        self.startScanPassive()

    def scannerThreadPassive(self):
        if not self.connected.get() and (not self.scanner_thread or not self.scanner_thread.is_alive()):
            self.scanner_thread = threading.Thread(target=self.scanner.process, args=(5,))
            self.scanner_thread.start()
        if not self.connected.get(): 
            self.after(500, self.scannerThreadPassive)

    def startScanPassive(self):
        if not self.scanner:
            self.scanner = Scanner().withDelegate(ScanDelegatePassive())
        else:
            try:
                self.scanner.stop()
            except BTLEException as e:
                self.connected.set(False)
                print(e)
        self.scanner.clear()
        self.scanner.start(passive=True)
        self.after(500, self.scannerThreadPassive)

    def stopScanPassive(self):
        if (self.scanner_thread and self.scanner_thread.is_alive()):
            self.scanner_thread.join(timeout=20)
            self.scanner_thread = None

    def resetAHRSSettings(self):
        self.twoKp.set(0.0)
        self.twoKpClient = 0.0
        self.twoKi.set(0.0)
        self.twoKiClient = 0.0
        self.sampleFreq.set(0.0)
        self.sampleFreqClient = 0.0
        self.gyroCorrection.set(0.0)
        self.gyroCorrectionClient = 0.0
        self.AHRSalgorithm.set(0)
        self.AHRSalgorithmClient = 0
        self.betaGain.set(0.0)
        self.betaGainClient = 0.0
        self.dataRateHz.set(0.0)
        self.dataRateHzClient = 0.0

        # PID variables
        self.pidKP.set(0.0)
        self.pidKPClient = 0.0
        self.pidKI.set(0.0)
        self.pidKIClient = 0.0
        self.pidKD.set(0.0)
        self.pidKDClient = 0.0
        self.pidSP.set(0.0)
        self.pidSPClient = 0.0
        self.pwmClock.set(0)
        self.pwmClockClient = 0
        self.pidKP2.set(0.0)
        self.pidKP2Client = 0.0
        self.pidKI2.set(0.0)
        self.pidKI2Client = 0.0
        self.pidKD2.set(0.0)
        self.pidKD2Client = 0.0
        self.pidSP2.set(0.0)
        self.pidSP2Client = 0.0

        self.accelNoiseThreshold.set(0.0)
        self.accelNoiseThresholdClient = 0.0

    def scalePlot(self):
        self.ax.relim()
        self.ax.autoscale_view(tight=False, scaley=True, scalex=False)
        self.dataplot_cnv.draw_idle()

    def clearPlot(self):
        for v in self.data_group.values():
            v.clearPlot()
        self.dataplot_cnv.draw_idle()

    def setAHRSData(self, idx, data):
        """ 
        Receive notifications and process them
        """
        global euler_angles
        # FIXME -- change hard-coded constants comparing to idx, over to not hard-coded
        if idx == 0:
            gidx = 'Euler Angles'
            # Euler Angles data arrives together as a list of length 3 (roll, pitch yaw)
            # gi ranges from 0 to 2
            for gi in range(3):
                self.data_group[gidx].setData(gi, data[gi])
            # Roll, Pitch, Yaw
            euler_angles = data 
        elif idx <= 4:
            # Accelerometer
            # gidx 1 to 3
            gidx = 'Accelerometer' 
            # gi 0 to 3
            gi = idx - 1 
            self.data_group[gidx].setData(gi, data)
        elif idx <= 8:
            # gidx 1 to 3
            gidx = 'Magnetometer' 
            # gi 0 to 3
            gi = idx - 5
            self.data_group[gidx].setData(gi, data)
        elif idx <= 12:
            # gidx 1 to 3
            gidx = 'Quaternion' 
            # gi 0 to 3
            gi = idx - 9
            # Quaternion data arrives separate as individual items
            # gi ranges from 0 to 3
            self.data_group[gidx].setData(gi, data)
        elif idx <= 18:
            # Gyroscope
            gidx = 'Gyroscope'
            gi = idx - 13
            self.data_group[gidx].setData(gi, data)
        elif idx == 19:
            # Gyro sensitivity
            self.gyroCorrection.set(data[0])
            self.gyroCorrectionClient = float(data[0])
        elif idx == 20:
            # bit flags 
            bit_flags = int(data[0])
            if bit_flags & 0x1:
                self.magnetometerStability.set(1)
            else:
                self.magnetometerStability.set(0)
            if bit_flags & 0x2:
                self.gyroscopeEnable.set(1)
            else:
                self.gyroscopeEnable.set(0)

            if bit_flags & 0x4:
                self.data_group['Accelerometer'].setButtonData('Hold', 1)
            else:
                self.data_group['Accelerometer'].setButtonData('Hold', 0)
            if bit_flags & 0x8:
                self.data_group['Magnetometer'].setButtonData('Hold', 1)
            else:
                self.data_group['Magnetometer'].setButtonData('Hold', 0)
            if bit_flags & 0x10:
                self.data_group['Gyroscope'].setButtonData('Hold', 1)
            else:
                self.data_group['Gyroscope'].setButtonData('Hold', 0)

            if bit_flags & 0x20:
                self.data_group['Accelerometer'].setButtonData('Ideal', 1)
            else:
                self.data_group['Accelerometer'].setButtonData('Ideal', 0)
            if bit_flags & 0x40:
                self.data_group['Magnetometer'].setButtonData('Ideal', 1)
            else:
                self.data_group['Magnetometer'].setButtonData('Ideal', 0)
            if bit_flags & 0x80:
                self.data_group['Gyroscope'].setButtonData('Ideal', 1)
            else:
                self.data_group['Gyroscope'].setButtonData('Ideal', 0)

            if bit_flags & 0x100:
                self.data_group['Quaternion'].setButtonData('Display', 1)
            else:
                self.data_group['Quaternion'].setButtonData('Display', 0)
            if bit_flags & 0x200:
                self.data_group['Accelerometer'].setButtonData('Display', 1)
            else:
                self.data_group['Accelerometer'].setButtonData('Display', 0)
            if bit_flags & 0x400:
                self.data_group['Magnetometer'].setButtonData('Display', 1)
            else:
                self.data_group['Magnetometer'].setButtonData('Display', 0)
            if bit_flags & 0x800:
                self.data_group['Gyroscope'].setButtonData('Display', 1)
            else:
                self.data_group['Gyroscope'].setButtonData('Display', 0)

            if bit_flags & 0x1000:
                self.uncalibratedDisplay.set(1)
            else:
                self.uncalibratedDisplay.set(0)

            if bit_flags & 0x2000:
                self.settingsDisplay.set(1)
            else:
                self.settingsDisplay.set(0)
            #FIXME
            #if bit_flags & 0x4000:
            #    self.data_group['ODR'].setButtonData('Ideal', 1)
            #else:
            #    self.data_group['ODR'].setButtonData('Ideal', 0)
            #if bit_flags & 0x8000:
            #    self.data_group['ODR'].setButtonData('Display', 1)
            #else:
            #    self.data_group['ODR'].setButtonData('Display', 0)

        elif idx == 21:
            # prop gain 
            self.twoKp.set(data[0])
            self.twoKpClient = float(data[0])
        elif idx == 22:
            # integral gain 
            self.twoKi.set(data[0])
            self.twoKiClient = float(data[0])
        elif idx == 23:
            # sample frequency
            self.sampleFreq.set(data[0])
            self.sampleFreqClient = float(data[0])
        elif idx == 24:
            # AHRS algorithm
            self.AHRSalgorithm.set(data[0])
            self.AHRSalgorithmClient = int(data[0])
        elif idx == 25:
            # Beta gain 
            self.betaGain.set(data[0]) 
            self.betaGainClient = float(data[0])
        elif idx <= 28:
            # ODR Hz 
            gidx = 'ODR'
            gi = idx - 26
            #FIXME  self.data_group[gidx].setData(gi, data)
        elif idx == 29:
            # accelerotometer noise threshold
            # magnetometer (30), gyro threshold(31) not currently used.
            self.accelNoiseThreshold.set(data[0])
            self.accelNoiseThresholdClient = float(data[0])

        elif idx == 32:
            self.pwmClock.set(data[0])
            self.pwmClockClient = int(data[0])
        elif idx == 33:
            # Motor Enabled 
            self.data_group['Motor'].setButtonData('Enabled', data[0])
        elif idx == 34:
            # Motor Driver
            gidx = 'Motor'
            gi = 0
            self.data_group[gidx].setData(gi, data)
        elif idx == 35:
            # Motor Display
            self.data_group['Motor'].setButtonData('Display', data[0])

        # Motor Control PID
        elif idx == 36:
            # PID KP
            self.pidKPClient = float(data[0])
            self.pidKP.set(data[0])
        elif idx == 37:
            # PID KI 
            self.pidKI.set(data[0])
            self.pidKIClient = float(data[0])
            #print(f"PID KI {self.pidKIClient}")
        elif idx == 38:
            # PID KD 
            self.pidKD.set(data[0])
            self.pidKDClient = float(data[0])
        elif idx == 39:
            # PID SP
            self.pidSP.set(data[0])
            self.pidSPClient = float(data[0])
        elif idx == 40:
            # PID PV
            self.pidPV.set(data[0])
            self.pidPVClient = float(data[0])
        elif idx == 41:
            # PID Output
            self.pidOutput.set(data[0])
            self.pidOutputClient = float(data[0])

        # Speed Control PID
        elif idx == 42:
            # PID KP 
            #print(f"PID KP2 {data[0]} {float(data[0])}")
            self.pidKP2.set(data[0])
            self.pidKP2Client = float(data[0])
        elif idx == 43:
            # PID KI 
            #print(f"PID KI2 {data[0]} {float(data[0])}")
            self.pidKI2.set(data[0])
            self.pidKI2Client = float(data[0])
        elif idx == 44:
            # PID KD 
            #print(f"PID KD2 {data[0]} {float(data[0])}")
            self.pidKD2.set(data[0])
            self.pidKD2Client = float(data[0])
        elif idx == 45:
            # PID SP
            self.pidSP2.set(data[0])
            self.pidSP2Client = float(data[0])
        elif idx == 46:
            # PID PV
            self.pidPV2.set(data[0])
            self.pidPV2Client = float(data[0])
            #if self.pidPV2Client != 0.0:
            #    print(f"PV2 {self.pidPV2Client}")
        elif idx == 47:
            # PID Output
            self.pidOutput2.set(data[0])
            self.pidOutput2Client = float(data[0])
            #if self.pidOutput2Client != 0.0:
            #    print(f"Out2 {self.pidOutput2Client}")
        elif idx == 48:
            gidx = 'Performance'
            # Performance Measure data arrives together as a list of length 3 :
            #    Roll, Motor Drive, Motor Speed
            self.data_group[gidx].setData(0, data)
        if self.dataplotcnt & 0xff == 0:
            # draw_idle is very slow, so don't call it too often.
            # It can cause a large backlog of notifications, causing the display to be unresponsive
            # to the current value in the most recent notification.
            self.scalePlot()
            self.dataplot_cnv.draw_idle()
        self.dataplotcnt = self.dataplotcnt + 1


    def createWidgetPlot(self, row_num, col_num, row_span):
        # Canvas
        mpl.use("TkAgg")

        # figsize is the Size of the graphical plot in inches 
        self.fig, self.ax = plt.subplots(figsize=(7, 4.0), constrained_layout=True)

        for v in self.data_group.values():
            v.setPlotAxes(self.ax)

        paddingx = 5
        paddingy = 5
        self.dataplot_cnv = FigureCanvasTkAgg(self.fig, master=self)
        self.dataplot_cnv.draw()
        self.dataplot_cnv.get_tk_widget().grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, rowspan=row_span, sticky=tk.N)

    def createVisualizationWidget(self, row_num, col_num, row_span):
        app = ShaderFrame(self, width=384, height=384)
        paddingx = 5
        paddingy = 5
        app.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, rowspan=row_span)
        # app.after(100, app.printContext)
        # callback delay in ms to redraw procedure
        app.animate = 1000 // 60

    def createWidgets(self):
        row_num = 0
        col_num = 0

        paddingx = 5
        paddingy = 5
        data_label_width = 9
        data_label_width_lag = 14
        data_label_width_sensors = 17
        self.spinbox_width = 10

        # 0,0
        # Menu
        self.menuFrame = tk.Frame(self)
        self.menuFrame.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)
        self.connect_button = tk.Checkbutton(self.menuFrame, text="Connect", command=self.connectButton, variable=self.connected, onvalue=True, offvalue=False)
        self.connect_button.grid(column=0, row=0)
        tk.Button(self.menuFrame, text="Quit", command=self.appExit).grid(column=1, row=0)
        tk.Button(self.menuFrame, text="Scale Plot", command=self.scalePlot).grid(column=2, row=0)
        tk.Button(self.menuFrame, text="Clear Plot", command=self.clearPlot).grid(column=3, row=0)

        # IMU Calibration
        col_num = 0
        row_num += 1
        data_row_num = row_num

        lf = tk.LabelFrame(self, text="IMU Calibration")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, sticky=tk.W)

        self.cb = []
        self.calibrateNormalizedButton = tk.Radiobutton(lf, text="Normalized", command=self.calibrateButton, variable=self.calibrate, value=0)
        self.cb.append(self.calibrateNormalizedButton)
        self.cb.append(tk.Radiobutton(lf, text="Zero Offset", command=self.calibrateButton, variable=self.calibrate, value=1))
        self.cb.append(tk.Radiobutton(lf, text="Magnetometer", command=self.calibrateButton, variable=self.calibrate, value=2))
        self.cb.append(tk.Radiobutton(lf, text="Gyroscope", command=self.calibrateButton, variable=self.calibrate, value=3))
        for cb in self.cb:
            cb.pack(anchor="w")

        tk.Button(lf, text="Reset", command=self.calibrateResetButton).pack(anchor="w")
        tk.Button(lf, text="Save", command=self.calibrateSaveButton).pack(anchor="w")

        lf = tk.LabelFrame(self, text="IMU Settings")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, sticky=tk.E)
        #tk.Label(lf, text="Gyroscope Correction").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        #tk.Spinbox(lf, text="Spinbox", command=self.gyroCorrectionSelect, from_=1, to_=9999, increment=1, textvariable=self.gyroCorrection, width=spinbox_width).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)
        tk.Checkbutton(lf, text="Magnetometer Stability", command=self.magnetometerStabilityButton, variable=self.magnetometerStability).grid(column=0, row=row_num, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Checkbutton(lf, text="Gyroscope Enable", command=self.gyroscopeEnableButton, variable=self.gyroscopeEnable).grid(column=0, row=row_num+1, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Checkbutton(lf, text="Uncalibrated Display", command=self.uncalibratedDisplayButton, variable=self.uncalibratedDisplay).grid(column=0, row=row_num+2, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Checkbutton(lf, text="Settings Display", command=self.settingsDisplayButton, variable=self.settingsDisplay).grid(column=0, row=row_num+3, padx=paddingx, pady=paddingy, sticky=tk.W)

        # Quaternion Data
        col_num += 1
        self.data_group['Quaternion'] = AHRSDataFrame(self, "Quaternion", row_num, col_num, data_names=["Q0", "Q1", "Q2", "Q3"], check_button_names=["Display"], data_label_width=8, sticky=tk.W)

        # Euler Angles
        self.data_group['Euler Angles'] = AHRSDataFrame(self, "Euler Angles", row_num, col_num, data_names=["Roll", "Pitch", "Yaw"], sticky=tk.NE)
        # Performance measurement -- Roll, Motor Drive value, Motor Speed all together
        self.data_group['Performance'] = AHRSDataFrame(self, "Performance", row_num, col_num, data_label_width=data_label_width_lag, data_names=[""], data_plot_names=["Roll", "Motor", "Speed"], sticky=tk.SE)

        # Controls
        col_num = 0
        row_num += 1

        lf = tk.LabelFrame(self, text="AHRS Settings")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        self.algorithmFrame = tk.Frame(lf)
        self.algorithmFrame.grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(self.algorithmFrame, text="Algorithm").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Radiobutton(self.algorithmFrame, text="Mahony", command=self.AHRSAlgorithmSelect, variable=self.AHRSalgorithm, value=0).grid(column=1, row=0)
        tk.Radiobutton(self.algorithmFrame, text="Madgwick", command=self.AHRSAlgorithmSelect, variable=self.AHRSalgorithm, value=1).grid(column=2, row=0)
        tk.Radiobutton(self.algorithmFrame, text="Simple", command=self.AHRSAlgorithmSelect, variable=self.AHRSalgorithm, value=2).grid(column=3, row=0)

        self.ahrsSettingsFrame = tk.Frame(lf)
        self.ahrsSettingsFrame.grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(self.ahrsSettingsFrame, text="Proportional Gain").grid(column=0, row=0, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.proportionalGainSelect, from_=0.0 , to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKp, width=self.spinbox_width).grid(column=1, row=0, padx=paddingx, pady=paddingy)

        tk.Label(self.ahrsSettingsFrame, text="Integral Gain").grid(column=0, row=1, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.integralGainSelect, from_=0.0, to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKi, width=self.spinbox_width).grid(column=1, row=1, padx=paddingx, pady=paddingy)

        tk.Label(self.ahrsSettingsFrame, text="Sample Frequency").grid(column=0, row=2, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.sampleFrequencySelect, from_=0.0, to_=1600.0, increment=32.0, format="%4.1f", textvariable=self.sampleFreq, width=self.spinbox_width).grid(column=1, row=2, padx=paddingx, pady=paddingy)

        tk.Label(self.ahrsSettingsFrame, text="Beta Gain").grid(column=0, row=3, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.betaGainSelect, from_=0.0, to_=5.0, increment=0.1, format="%1.2f", textvariable=self.betaGain, width=self.spinbox_width).grid(column=1, row=3, padx=paddingx, pady=paddingy)


        # Accelerometer Data
        col_num += 1
        scaleAccel = []
        scaleAccel.append(AHRSScale("Noise Threshold", self.accelNoiseThresholdSelect, self.accelNoiseThreshold, tk.HORIZONTAL, 0.0, 2.0, 0.1))
        self.data_group['Accelerometer'] = AHRSDataFrame(self, "Accelerometer", row_num, col_num, data_names=["Normalized", "Calibrated", "Uncalibrated", "Min Threshold"], scales=scaleAccel, data_label_width=data_label_width_sensors, check_button_names=['Hold', 'Ideal', 'Display'])

        # Magnetometer Data
        col_num = 0
        row_num += 1
        self.data_group['Magnetometer'] = AHRSDataFrame(self, "Magnetometer", row_num, col_num, data_names=["Normalized", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width=data_label_width_sensors, check_button_names=['Hold', 'Ideal', 'Display'])

        # Gyroscope Data
        col_num += 1
        self.data_group['Gyroscope'] = AHRSDataFrame(self, "Gyroscope", row_num, col_num, data_names=["Normalized X", "Normalized Y", "Normalized Z", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width=data_label_width_sensors, check_button_names=['Hold', 'Ideal', 'Display'])

        # Measured Output Data Rate
        #col_num = col_num + 1
        #self.data_group['ODR'] = AHRSDataFrame(self, "ODR", ["Accelerometer", "Gyroscope", "Magnetometer"], 8, ['Ideal', 'Display'], row_num, col_num, sticky=tk.NW)

        # PID Motor Control
        col_num = 0
        row_num += 1
        scaleMotor = []
        spinboxMotor = []
        buttonMotor = []
        scaleMotor.append(AHRSScale("Proportional", self.pidKPSelect, self.pidKP, tk.HORIZONTAL, 100.0, 800.0, 2.0))
        scaleMotor.append(AHRSScale("Integral", self.pidKISelect, self.pidKI, tk.HORIZONTAL, 0.0, 400.0, 2.0))
        scaleMotor.append(AHRSScale("Derivative", self.pidKDSelect, self.pidKD, tk.HORIZONTAL, 0.0, 200.0, 2.0))
        spinboxMotor.append(AHRSSpinbox("SetPoint", self.pidSPSelect, self.pidSP, -2.0, 2.0, 0.05, '%1.3f'))
        spinboxMotor.append(AHRSSpinbox("PWM Clock Scale", self.pwmClockSelect, self.pwmClock, 0, 7, 1, '%1.1f'))
        buttonMotor.append(AHRSButton(self, "Save", command="PID_PARAM_SAVE"))
        buttonMotor.append(AHRSButton(self, "Erase", command="PID_PARAM_ERASE"))
        self.data_group['Motor'] = AHRSDataFrame(self, "Motor", row_num, col_num, data_names=["Driver"], scales=scaleMotor, spinboxes=spinboxMotor, check_button_names=['Enabled', 'Display'], check_button_cmds=["MOTOR_DRIVER_TOGGLE_POWER", "MOTOR_DRIVER_TOGGLE_DISPLAY"], button_stack="horizontal", buttons=buttonMotor, data_label_width=8)

        # Speed Control PID
        col_num += 1
        scaleSpeed = []
        spinboxSpeed = []
        buttonSpeed = []
        scaleSpeed.append(AHRSScale("Proportional", self.pidKP2Select, self.pidKP2, tk.HORIZONTAL, 0.0, 0.5, 0.005))
        scaleSpeed.append(AHRSScale("Integral", self.pidKI2Select, self.pidKI2, tk.HORIZONTAL, 0.0, 0.5, 0.005))
        scaleSpeed.append(AHRSScale("Derivative", self.pidKD2Select, self.pidKD2, tk.HORIZONTAL, 0.0, 0.5, 0.005))
        spinboxSpeed.append(AHRSSpinbox("SetPoint", self.pidSP2Select, self.pidSP2, -1.0, 1.0, 0.05, '%1.2f'))
        buttonSpeed.append(AHRSButton(self, "Save", command="PID2_PARAM_SAVE"))
        buttonSpeed.append(AHRSButton(self, "Erase", command="PID2_PARAM_ERASE"))
        self.data_group['Speed'] = AHRSDataFrame(self, "Speed", row_num, col_num, data_names=None, scales=scaleSpeed, spinboxes=spinboxSpeed, check_button_names=None, check_button_cmds=None, button_stack="horizontal", buttons=buttonSpeed, data_label_width=8)

        # Data Plot
        col_num += 1
        row_num = 1 
        row_span = 2
        self.createWidgetPlot(data_row_num, col_num, row_span)
        data_row_num = data_row_num + 2
        self.createVisualizationWidget(data_row_num, col_num, row_span)

    def magnetometerStabilityButton(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            self.writeCmdStr('IMU_MAGNETOMETER_STABILITY_TOGGLE')

    def gyroscopeEnableButton(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            self.writeCmdStr('IMU_GYROSCOPE_ENABLE_TOGGLE')

    def uncalibratedDisplayButton(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            self.writeCmdStr('IMU_UNCALIBRATED_DISPLAY_TOGGLE')

    def settingsDisplayButton(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            self.writeCmdStr('IMU_SETTINGS_DISPLAY_TOGGLE')

    def proportionalGainSelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            print("Proportional Gain Client %f Server %f" % ( self.twoKpClient, self.twoKp.get()) )
            if self.twoKpClient < self.twoKp.get():
                # Send up
                print("Prop Gain Up from %f" % ( self.twoKpClient) )
                self.writeCmdStr('IMU_AHRS_PROP_GAIN_UP')
            elif self.twoKpClient > self.twoKp.get():
                # Send down
                print("Prop Gain Down from %f" % ( self.twoKpClient) )
                self.writeCmdStr('IMU_AHRS_PROP_GAIN_DOWN')

    def integralGainSelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            print("Integral Gain %f" % ( self.twoKi.get()) )
            if self.twoKiClient < self.twoKi.get():
                # Send up
                print("Integral Gain Up from %f" % ( self.twoKiClient) )
                self.writeCmdStr('IMU_AHRS_INTEG_GAIN_UP')
            elif self.twoKiClient > self.twoKi.get():
                # Send down
                print("Integral Gain Down from %f" % ( self.twoKiClient) )
                self.writeCmdStr('IMU_AHRS_INTEG_GAIN_DOWN')

    def sampleFrequencySelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            print("Sample Frequency %f" % ( self.sampleFreq.get()) )
            if self.sampleFreqClient < self.sampleFreq.get():
                # Send up
                print("Sample Frequency Up from %f" % ( self.sampleFreqClient ) )
                self.writeCmdStr('IMU_AHRS_SAMPLE_FREQ_UP')
            elif self.sampleFreqClient > self.sampleFreq.get():
                # Send down
                print("Sample Frequency Down from %f" % ( self.sampleFreqClient) )
                self.writeCmdStr('IMU_AHRS_SAMPLE_FREQ_DOWN')

    def gyroCorrectionSelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            print("Gyroscope Correction %f" % ( self.gyroCorrection.get()) )
            if self.gyroCorrectionClient < self.gyroCorrection.get():
                # Send up
                print("Gyroscope Correction Up from %f" % ( self.gyroCorrectionClient ) )
                self.writeCmdStr('IMU_GYROSCOPE_CORRECTION_UP')
            elif self.gyroCorrectionClient > self.gyroCorrection.get():
                # Send down
                print("Gyroscope Correction Down from %f" % ( self.gyroCorrectionClient ) )
                self.writeCmdStr('IMU_GYROSCOPE_CORRECTION_DOWN')

    def betaGainSelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            print("Beta Gain %f" % ( self.betaGain.get()) )
            if self.betaGainClient < self.betaGain.get():
                # Send up
                print("Beta Gain Up from %f" % ( self.betaGainClient) )
                self.writeCmdStr('IMU_AHRS_BETA_GAIN_UP')
            elif self.betaGainClient > self.betaGain.get():
                # Send down
                print("Beta Gain Down from %f" % ( self.betaGainClient) )
                self.writeCmdStr('IMU_AHRS_BETA_GAIN_DOWN')

    def AHRSAlgorithmSelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        elif self.AHRSalgorithmClient != self.AHRSalgorithm.get():
            self.writeCmdStr('IMU_AHRS_ALGORITHM_TOGGLE')

    def calibrateResetButton(self):
        print("Calibrate Reset")
        self.writeCmdStr("IMU_SENSOR_CALIBRATE_RESET")
        self.calibrateNormalizedButton.invoke()

    def calibrateSaveButton(self):
        print("Calibrate Save")
        self.writeCmdStr("IMU_SENSOR_CALIBRATE_SAVE")

    def pidParamSaveButton(self):
        print("PID Param Save")
        self.writeCmdStr("PID_PARAM_SAVE")

    # Accelerometer Noise Threshold Multiplier
    def accelNoiseThresholdSelect(self, value):
        if not self.connected.get():
            print("Not connected to AHRS")
        elif self.calibrate.get() != 1:
            print("Not in Zero Offset calibration mode")
        else:
            while self.accelNoiseThresholdClient != self.accelNoiseThreshold.get() and self.calibrate.get() == 1:
                if self.accelNoiseThresholdClient < self.accelNoiseThreshold.get():
                    # Send up
                    print("Accel Noise Threshold Up from %f" % ( self.accelNoiseThresholdClient) )
                    self.writeCmdStr('IMU_SELECT_ACCELEROMETER')
                    self.writeCmdStr('NOISE_THRESHOLD_UP')
                elif self.accelNoiseThresholdClient > self.accelNoiseThreshold.get():
                    # Send down
                    print("Accel Noise Threshold Down from %f" % ( self.accelNoiseThresholdClient) )
                    self.writeCmdStr('IMU_SELECT_ACCELEROMETER')
                    self.writeCmdStr('NOISE_THRESHOLD_DOWN')


    # Motor PID Control
    def pidKPSelect(self, value):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidKPClient < self.pidKP.get():
                # Send up
                print("PID KP Up %f from Client %f" % ( self.pidKP.get(), self.pidKPClient) )
                self.writeCmdStr('PID_KP_UP')
            elif self.pidKPClient > self.pidKP.get():
                # Send down
                print("PID KP Down %f from Client %f" % ( self.pidKP.get(), self.pidKPClient) )
                self.writeCmdStr('PID_KP_DOWN')

    def pidKISelect(self, value):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidKIClient < self.pidKI.get():
                # Send up
                print("PID KI Up from %f" % ( self.pidKIClient) )
                self.writeCmdStr('PID_KI_UP')
            elif self.pidKIClient > self.pidKI.get():
                # Send down
                print("PID KI Down from %f" % ( self.pidKIClient) )
                self.writeCmdStr('PID_KI_DOWN')

    def pidKDSelect(self, value):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidKDClient < self.pidKD.get():
                # Send up
                print("PID KD Up from %f" % ( self.pidKDClient) )
                self.writeCmdStr('PID_KD_UP')
            elif self.pidKDClient > self.pidKD.get():
                # Send down
                print("PID KD Down from %f" % ( self.pidKDClient) )
                self.writeCmdStr('PID_KD_DOWN')
 
    def pidSPSelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidSPClient < self.pidSP.get():
                # Send up
                print("PID SP Up from %f" % ( self.pidSPClient) )
                self.writeCmdStr('PID_SP_UP')
            elif self.pidSPClient > self.pidSP.get():
                # Send down
                print("PID SP Down from %f" % ( self.pidSPClient) )
                self.writeCmdStr('PID_SP_DOWN')

    # Motor PWM Clock Control
    def pwmClockSelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pwmClockClient < self.pwmClock.get():
                # Send up
                print("PWM Clock Up from %d" % ( self.pwmClockClient) )
                self.writeCmdStr('PWM_CLOCK_UP')
            elif self.pwmClockClient > self.pwmClock.get():
                # Send down
                print("PWM Clock Down from %d" % ( self.pwmClockClient) )
                self.writeCmdStr('PWM_CLOCK_DOWN')

    # Speed PID Control
    def pidKP2Select(self, value):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidKP2Client < self.pidKP2.get():
                # Send up
                #print("PID KP2 Up from %f" % ( self.pidKP2Client) )
                self.writeCmdStr('PID2_KP_UP')
            elif self.pidKP2Client > self.pidKP2.get():
                # Send down
                #print("PID KP2 Down from %f" % ( self.pidKP2Client) )
                self.writeCmdStr('PID2_KP_DOWN')

    def pidKI2Select(self, value):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidKI2Client < self.pidKI2.get():
                # Send up
                #print("PID2 KI Up from %f" % ( self.pidKI2Client) )
                self.writeCmdStr('PID2_KI_UP')
            elif self.pidKI2Client > self.pidKI2.get():
                # Send down
                #print("PID2 KI Down from %f" % ( self.pidKI2Client) )
                self.writeCmdStr('PID2_KI_DOWN')

    def pidKD2Select(self, value):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidKD2Client < self.pidKD2.get():
                # Send up
                #print("PID2 KD Up from %f" % ( self.pidKD2Client) )
                self.writeCmdStr('PID2_KD_UP')
            elif self.pidKD2Client > self.pidKD2.get():
                # Send down
                #print("PID2 KD Down from %f" % ( self.pidKD2Client) )
                self.writeCmdStr('PID2_KD_DOWN')
 
    def pidSP2Select(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            if self.pidSP2Client < self.pidSP2.get():
                # Send up
                print("PID2 SP Up from %f" % ( self.pidSP2Client) )
                self.writeCmdStr('PID2_SP_UP')
            elif self.pidSP2Client > self.pidSP2.get():
                # Send down
                print("PID2 SP Down from %f" % ( self.pidSP2Client) )
                self.writeCmdStr('PID2_SP_DOWN')

    def calibrateButton(self):
        if self.connected.get():
            #print("Calibrate Button %d" % ( self.calibrate.get() ))
            if self.calibrate.get() == 0:
                self.writeCmdStr('IMU_SENSOR_CALIBRATE_NORMALIZED')
            elif self.calibrate.get() == 1:
                self.writeCmdStr('IMU_SENSOR_CALIBRATE_ZERO_OFFSET')
            elif self.calibrate.get() == 2:
                self.writeCmdStr('IMU_SENSOR_CALIBRATE_MAGNETOMETER')
            elif self.calibrate.get() == 3:
                self.writeCmdStr('IMU_SENSOR_CALIBRATE_GYROSCOPE')

    def connectButton(self):
        if self.connected.get():
            self.stopScanPassive()
            self.connect_thread = threading.Thread(target=self.connectPeripheral)
            self.connect_thread.start()
        else:
            self.disconnect()
            self.startScanPassive()

    def getBLEState(self):
        if self.peripheral is not None:
            try:
                ble_state = self.peripheral.getState()
                print("Current State %s" % ( ble_state ))
            except BTLEException as e:
                self.connected.set(False)
                print(e)
                return
            return ble_state
        return

    def scanForAHRS(self):
        scan_delegate = ScanDelegateFindAHRS()
        scanner = Scanner().withDelegate(scan_delegate)
        scanner.clear()
        scanner.start(passive=False)
        timeout_counter = 24
        while self.dev_addr is None and timeout_counter > 0:
            scanner.process(5)
            self.dev_addr = scan_delegate.getAHRSAddr()
            timeout_counter = timeout_counter - 1
        scanner.stop()
        return self.dev_addr

    def connectPeripheral(self):
        print("Connect...")
        self.peripheral = None
        # USB-powered device.
        # dev_addr = 'CC:43:80:8D:F8:46'
        # battery-powered device.
        # dev_addr = 'F1:68:47:7C:AD:E3'
        dev_addr = self.scanForAHRS()
        if dev_addr == None:
            print("Could not find AHRS")
            self.connected.set(False)
            return

        connected = False
        connectCount = 0
        connectCountLimit = 30
        # FIXME: temporary workaround for intermittent connection failures
        while not connected:
            try:
                print(f"Trying connectCount {connectCount}")
                self.peripheral = Peripheral(deviceAddr = dev_addr, addrType = 'random').withDelegate(NotifyDelegate())
                connected = True
            except BTLEException as e:
                print(f"Failed connectCount {connectCount}")
                connectCount += 1
                if connectCount >= connectCountLimit:
                    print(e)
                    self.connected.set(False)
                    self.startScanPassive()
                    return

        print("Connected to %s after %d failures" % ( dev_addr, connectCount ))
        # Nordic UART Service UUID.  Vendor-specific. 
        # Value is defined Inside ble_nus_c.h as NUS_BASE_UUID
        srv = self.peripheral.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
        ch = srv.getCharacteristics()
        # Enable notifications on TX characteristic
        for c in ch:
            if 'NOTIFY' in c.propertiesToString():
                print(f"Enabling notifications on {c}")
                d = c.getDescriptors()[0]
                val = d.read()
                #print("    Value:  ", binascii.b2a_hex(val).decode('utf-8'))
                d.write(b"\x01\x00",withResponse=True)
                val = d.read()
                #print("    Value:  ", binascii.b2a_hex(val).decode('utf-8'))
        print("Wait for notifications")
        while self.connected.get():
            try:
                self.peripheral.waitForNotifications(0.1)
            except:
                pass
        self.peripheral.disconnect()
        self.peripheral = None
        print("Disconnected from %s" % ( dev_addr) )

    def writeCmdStr(self,cmd):
        try:
            print("Sending command %s" % ( cmd ))
            cmdInt = self.commandDict[cmd]
            self.writeCmdInt(cmdInt)
        except:
            print("ERROR: Trouble sending command %s" % (cmd))
            self.disconnect()

    def writeCmdInt(self,cmd):
        self.writeCmd(cmd.to_bytes(1, byteorder=sys.byteorder))

    def writeCmd(self,cmd):
        try:
            print(f'writeCmd0 {cmd}')
            srv = self.peripheral.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
            print(f'writeCmd1 {cmd}')
            uuid_write = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
            uart_write = srv.getCharacteristics(forUUID = uuid_write)
            print(f'writeCmd2 {cmd} {uart_write[0]} {type(uart_write)} {type(uart_write[0])}')
            uart_write[0].write(cmd, withResponse=True)
            print(f'writeCmd3 {cmd} {uart_write[0]} {type(uart_write)} {type(uart_write[0])}')
        except BTLEException as e:
            print(f'writeCmd4 {cmd}')
            self.disconnect()

    def disconnect(self):
        print("Disconnect...")
        # disable delegate from flooding app with notifications
        if self.peripheral is not None:
            self.peripheral.setDelegate(None)
        # if already connected, set connect button variable
        self.connected.set(False)

    def appExit(self):
        if self.connected.get():
            self.connect_button.invoke()
        if self.peripheral is None:
            self.stopScanPassive()
            #self.scanner.stop()
            self.quit()
            sys.exit(0)

class NotifyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleNotification(self, cHandle, data):
        str_data = str(data, encoding='utf-8').split()
        #print("Notif: %s" % ( str_data ))
        try:
            idx = int(str_data[0])
            console.setAHRSData( idx, str_data[1:] )
        except:
            raise
            # print("Notif: %s" % ( str_data ))


console = AHRSConsole()
console.master.title('AHRS Console')
console.mainloop()

