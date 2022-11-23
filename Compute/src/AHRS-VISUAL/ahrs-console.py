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

# Avoiding glitches in pyopengl-3.0.x and python3.4
def bytestr(s):
    return s.encode("utf-8") + b"\000"

vshader_source = """
#version 140
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


def rot(a, b, c):
    s = np.sin(a)
    c = np.cos(a)
    am = np.array(((c, s, 0, 0), (-s, c, 0, 0), (0, 0, 1, 0), (0, 0, 0, 1)), np.float32)
    s = np.sin(b)
    c = np.cos(b)
    bm = np.array(((c, 0, s, 0), (0, 1, 0, 0), (-s, 0, c, 0), (0, 0, 0, 1)), np.float32)
    s = np.sin(c)
    c = np.cos(c)
    cm = np.array(((1, 0, 0, 0), (0, c, s, 0), (0, -s, c, 0), (0, 0, 0, 1)), np.float32)
    return np.dot(np.dot(am, bm), cm)


class ShaderFrame(pyopengltk.OpenGLFrame):

    def initgl(self):

        #print("visualizer.init")
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

        self.start = time.time()


    def redraw(self):
        """Render a single frame"""
        GL.glClear(GL.GL_COLOR_BUFFER_BIT | GL.GL_DEPTH_BUFFER_BIT)
        GL.glUseProgram (self.shader)
        tm = time.time() - self.start
        self.mvp = np.eye(4, dtype=np.float32)

        t = time.time()-self.start
        s = 2.
        self.mvp = rot(t*s/5., t*s/6., t*s/7.)

        GL.glBindBuffer(GL.GL_ARRAY_BUFFER, self.buffers[0])
        GL.glUniformMatrix4fv(self.unif_mvp, 1, GL.GL_FALSE, self.mvp)

        GL.glFrontFace(GL.GL_CCW)
        GL.glCullFace(GL.GL_BACK)
        GL.glEnable(GL.GL_CULL_FACE)

        start_pos = 0
        for i in range(6):
            GL.glDrawArrays( GL.GL_TRIANGLE_FAN, start_pos, 4)
            start_pos = start_pos + 4


class ScanDelegate(DefaultDelegate):
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

class AHRSDataFrame():
    def __init__(self, master, data_group_name, data_names, data_label_width, check_button_names, row, column, data_hold=False, sticky=None):
        # Save master object 
        self.master = master
        paddingx = 2
        paddingy = 2
        lf = tk.LabelFrame(master, text=data_group_name)
        if sticky:
            lf.grid(column=column, row=row, padx=paddingx, pady=paddingy, sticky=sticky)
        else:
            lf.grid(column=column, row=row, padx=paddingx, pady=paddingy)

        self.dataItems = []
        self.data_group_name = data_group_name
        if data_names:
            if check_button_names:
                # create separate frames for data and check buttons
                self.dataFrame = tk.Frame(lf)
                self.dataFrame.grid(column=column, row=row, padx=paddingx, pady=paddingy)
            else:
                self.dataFrame = lf

            row_first = row
            for data_name in data_names:
                if isinstance(data_name, (list)):
                    for dat in data_name:
                        row = row + 1
                        self.dataItems.append(AHRSDataItem(self.dataFrame, dat, data_label_width, row, column))
                    row = row_first
                    column = column + 2
                else:
                    row = row + 1
                    self.dataItems.append(AHRSDataItem(self.dataFrame, data_name, data_label_width, row, column))

        if check_button_names:
            self.checkButtonFrame = tk.Frame(lf)
            self.checkButtonFrame.grid(column=column, row=row+1, padx=paddingx, pady=paddingy)
            self.checkButtonData = dict()
            brow = bcolumn = 0
            bcolumn_start = bcolumn
            bcolumn_last = bcolumn + 3
            anch = tk.CENTER
            for name in check_button_names:
                self.checkButtonData[name] = tk.StringVar()
                self.checkButtonData[name].set(0)
                def buttonHandler(self=self, button_name=name):
                    return self._buttonHandler(button_name)
                tk.Checkbutton(self.checkButtonFrame, text=name, command=buttonHandler, variable=self.checkButtonData[name], indicatoron=1, anchor=anch).grid(column=bcolumn, row=brow, padx=paddingx, pady=paddingy)
                bcolumn = bcolumn + 1
                if bcolumn == bcolumn_last:
                    brow = brow + 1
                    bcolumn = bcolumn_start

    def _buttonHandler(self, button_name):
        if not self.master.connected.get():
            print("Not connected to AHRS")
        else:
            self.master.writeCmdStr('IMU_SELECT_' + self.data_group_name.upper() )
            self.master.writeCmdStr('IMU_SENSOR_DATA_' + button_name.upper() + '_TOGGLE')

    def setButtonData(self, button_name, data):
        self.checkButtonData[button_name].set(data)

    def setData(self, idx, data):
        self.dataItems[idx].setData(data)

    def setupPlot(self, ax):
        # setup data plot
        for dItem in self.dataItems:
            dItem.setupPlot(ax)

class AHRSDataItem():
    def __init__(self, master, data_name, data_label_width, row, column):
        self.data_name = data_name
        self.data = tk.StringVar()
        self.data.set(0)
        paddingx = 5
        paddingy = 5
        self.xpoints = 500
        self.dataFrame = master
        col_start=column
        row_start=row
        tk.Label(self.dataFrame, text=data_name, justify=tk.LEFT).grid(column=col_start, row=row_start, padx=paddingx, pady=paddingy, sticky=tk.W)
        self.data_label = tk.Label(self.dataFrame, relief=tk.SUNKEN, textvariable=self.data, width=data_label_width)
        self.data_label.grid(column=col_start+1, row=row_start, padx=paddingx, pady=paddingy)
        def handler(event, self=self):
            return self._labelHandler(event)
        self.data_label.bind('<Button-1>', handler)
        self.data_label.bind('<Button-3>', handler)
        self.dataplot = np.zeros(self.xpoints)
        self.dataplotidx = 0
        self.line = None

    def _labelHandler(self, event):
        print("Label button ", self.data_name)
        print(event.type)
        print("event num %d" % ( event.num) )

    def setData(self, data):
        self.data.set(data)
        #print("DataItem %s data = %s" % ( self.data_name, self.data.get() ))
        if self.line:
            if isinstance(data, (list)):
                self.dataplot[self.dataplotidx] = float(data[0])
                self.line.set_data(np.arange(self.xpoints), self.dataplot)
            else:
                self.dataplot[self.dataplotidx] = float(data)
                self.line.set_data(np.arange(self.xpoints), self.dataplot)
            self.dataplotidx = ( self.dataplotidx + 1 ) % self.xpoints
    def getData(self):
        return self.data
    def setupPlot(self, ax):
        self.line, = ax.plot(np.arange(self.xpoints), self.dataplot, label=self.data_name)

class AHRSConsole(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()

        self.connected = tk.BooleanVar()
        self.connected.set(False)
        self.connect_thread = None
        self.connect_button = None

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
        self.commandDict['IMU_AHRS_INPUT_TOGGLE'] = 11
        self.commandDict['IMU_AHRS_YAW_TOGGLE'] = 12
        self.commandDict['IMU_AHRS_PITCH_TOGGLE'] = 13
        self.commandDict['IMU_AHRS_ROLL_TOGGLE'] = 14
        self.commandDict['IMU_SENSOR_DATA_HOLD_TOGGLE'] = 15 
        self.commandDict['IMU_SENSOR_DATA_IDEAL_TOGGLE'] = 16
        self.commandDict['IMU_SENSOR_DATA_FIXED_TOGGLE'] = 17
        self.commandDict['IMU_SENSOR_DATA_DISPLAY_TOGGLE'] = 18
        self.commandDict['IMU_AHRS_PROP_GAIN_UP'] = 19
        self.commandDict['IMU_AHRS_PROP_GAIN_DOWN'] = 20
        self.commandDict['IMU_AHRS_INTEG_GAIN_UP'] = 21
        self.commandDict['IMU_AHRS_INTEG_GAIN_DOWN'] = 22
        self.commandDict['IMU_AHRS_SAMPLE_FREQ_UP'] = 23
        self.commandDict['IMU_AHRS_SAMPLE_FREQ_DOWN'] = 24
        self.commandDict['IMU_GYROSCOPE_CORRECTION_UP'] = 25
        self.commandDict['IMU_GYROSCOPE_CORRECTION_DOWN'] = 26
        self.commandDict['IMU_MAGNETOMETER_STABILITY_TOGGLE'] = 27
        self.commandDict['IMU_AHRS_BETA_GAIN_UP'] = 28
        self.commandDict['IMU_AHRS_BETA_GAIN_DOWN'] = 29
        self.commandDict['IMU_AHRS_ALGORITHM_TOGGLE'] = 30
        self.commandDict['IMU_GYROSCOPE_ENABLE_TOGGLE'] = 31
        self.commandDict['IMU_UNCALIBRATED_DISPLAY_TOGGLE'] = 32
        self.commandDict['IMU_SETTINGS_DISPLAY_TOGGLE'] = 33
        self.commandDict['IMU_SELECT_ODR'] = 34 

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

    def scalePlot(self):
        self.ax.relim()
        self.ax.autoscale_view(tight=False, scaley=True, scalex=False)
        self.dataplot_cnv.draw_idle()

    def setAHRSData(self, idx, data):
        # print("NOTIF %d" % ( idx ))
        if idx == 0:
            # AHRS
            gidx = 'Euler Angles'
            for gi in range(3):
                self.data_group[gidx].setData(gi, data[gi])
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
            # print("BIT FLAGS %s" % ( hex(bit_flags)))
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
            if bit_flags & 0x4000:
                self.data_group['ODR'].setButtonData('Ideal', 1)
            else:
                self.data_group['ODR'].setButtonData('Ideal', 0)
            if bit_flags & 0x8000:
                self.data_group['ODR'].setButtonData('Display', 1)
            else:
                self.data_group['ODR'].setButtonData('Display', 0)

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
            self.data_group[gidx].setData(gi, data)
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

        paddingx = 5
        paddingy = 5
        self.fig, self.ax = plt.subplots(figsize=(6, 4.0), constrained_layout=True)

        # setup initial data plot
        self.data_group['Euler Angles'].setupPlot(self.ax)
        #self.data_group['Quaternion'].setupPlot(self.ax)
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value');
        self.ax.legend()

        self.ax.relim()
        self.ax.autoscale_view(tight=False, scaley=True, scalex=False)

        self.dataplot_cnv = FigureCanvasTkAgg(self.fig, master=self)
        self.dataplot_cnv.draw()
        self.dataplot_cnv.get_tk_widget().grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, rowspan=row_span, sticky=tk.N)

    def visualizationWidgetInit(self, args):
        print("init %s %s %s" % ( 'ahrs-console.py', args, type(args) ) )
        if self.visualizerw: 
            visualizer.init(self.visualizerw)
        else:
            self.visualizerw_doinit = True

    def visualizationWidgetReshape(self, foo):
        print("reshape %s %s" % ( 'ahrs-console.py' , foo) )
        if self.visualizerw: 
            visualizer.reshape(self.visualizerw)
        else:
            self.visualizerw_doreshape = True

    def visualizationWidgetZap(self, foo):
        print("zap %s" % ( foo) )
        visualizer.zap(self.visualizerw)

    def visualizationWidgetIdle(self, foo):
        #print("idle %s %s" % ( self.visualizerw , type(self.visualizerw) ) )
        visualizer.idle(self.visualizerw)

    def visualizationWidgetDraw(self, foo):
        #print("draw %s" % ( foo) )
        visualizer.draw(self.visualizerw)

    def render(self, foo):
        print("render %s" % ( foo) )

    def createVisualizationWidget(self, row_num, col_num, row_span):
        app = ShaderFrame(self, width=384, height=384)
        paddingx = 5
        paddingy = 5
        app.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, rowspan=row_span)
        app.after(100, app.printContext)
        app.animate = 1000 // 60


    def createWidgets(self):
        row_num = 0
        col_num = 0

        paddingx = 5
        paddingy = 5
        data_label_width = 9
        data_label_width_sensors = 17
        spinbox_width = 10

        # 0,0
        # Menu
        self.menuFrame = tk.Frame(self)
        self.menuFrame.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)
        self.connect_button = tk.Checkbutton(self.menuFrame, text="Connect", command=self.connectButton, variable=self.connected, onvalue=True, offvalue=False)
        self.connect_button.grid(column=0, row=0)
        tk.Button(self.menuFrame, text="Quit", command=self.appExit).grid(column=1, row=0)
        tk.Button(self.menuFrame, text="Scale", command=self.scalePlot).grid(column=2, row=0)

        # 1,0
        # Calibration
        col_num = 0
        row_num = row_num + 1
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

        # Euler Angles
        col_num = col_num + 1
        #self.data_group['Euler Angles'] = AHRSDataFrame(self, "Euler Angles", [["Roll", "Pitch", "Yaw"], ["Roll-Local", "Pitch-Local", "Yaw-Local"]], 8, None, row_num, col_num)
        self.data_group['Euler Angles'] = AHRSDataFrame(self, "Euler Angles", ["Roll", "Pitch", "Yaw"], data_label_width, None, row_num, col_num, sticky=tk.W)

        # Quaternion Data
        #col_num = col_num + 1
        self.data_group['Quaternion'] = AHRSDataFrame(self, "Quaternion", ["Q0", "Q1", "Q2", "Q3"], data_label_width, ["Display"], row_num, col_num, sticky=tk.E)

        #self.data_group['Euler Angles Local'] = AHRSDataFrame(self, "Euler Angles Local", ["Roll", "Pitch", "Yaw"], 8, None, row_num, col_num)

        col_num = 0
        row_num = row_num + 1

        lf = tk.LabelFrame(self, text="IMU Settings")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)
        #tk.Label(lf, text="Gyroscope Correction").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        #tk.Spinbox(lf, text="Spinbox", command=self.gyroCorrectionSelect, from_=1, to_=9999, increment=1, textvariable=self.gyroCorrection, width=spinbox_width).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)
        tk.Checkbutton(lf, text="Magnetometer Stability", command=self.magnetometerStabilityButton, variable=self.magnetometerStability).grid(column=0, row=row_num, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Checkbutton(lf, text="Gyroscope Enable", command=self.gyroscopeEnableButton, variable=self.gyroscopeEnable).grid(column=0, row=row_num+1, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Checkbutton(lf, text="Uncalibrated Display", command=self.uncalibratedDisplayButton, variable=self.uncalibratedDisplay).grid(column=0, row=row_num+2, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Checkbutton(lf, text="Settings Display", command=self.settingsDisplayButton, variable=self.settingsDisplay).grid(column=0, row=row_num+3, padx=paddingx, pady=paddingy, sticky=tk.W)

        col_num = col_num + 1

        # 2,0
        # Controls
        lf = tk.LabelFrame(self, text="AHRS Settings")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        self.algorithmFrame = tk.Frame(lf)
        self.algorithmFrame.grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(self.algorithmFrame, text="Algorithm").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Radiobutton(self.algorithmFrame, text="Mahony", command=self.AHRSAlgorithmSelect, variable=self.AHRSalgorithm, value=0).grid(column=1, row=0)
        tk.Radiobutton(self.algorithmFrame, text="Madgwick", command=self.AHRSAlgorithmSelect, variable=self.AHRSalgorithm, value=1).grid(column=2, row=0)

        self.ahrsSettingsFrame = tk.Frame(lf)
        self.ahrsSettingsFrame.grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(self.ahrsSettingsFrame, text="Proportional Gain").grid(column=0, row=0, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.proportionalGainSelect, from_=0.0 , to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKp, width=spinbox_width).grid(column=1, row=0, padx=paddingx, pady=paddingy)

        tk.Label(self.ahrsSettingsFrame, text="Integral Gain").grid(column=0, row=1, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.integralGainSelect, from_=0.0, to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKi, width=spinbox_width).grid(column=1, row=1, padx=paddingx, pady=paddingy)

        tk.Label(self.ahrsSettingsFrame, text="Sample Frequency").grid(column=0, row=2, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.sampleFrequencySelect, from_=0.0, to_=1600.0, increment=32.0, format="%4.1f", textvariable=self.sampleFreq, width=spinbox_width).grid(column=1, row=2, padx=paddingx, pady=paddingy)

        tk.Label(self.ahrsSettingsFrame, text="Beta Gain").grid(column=0, row=3, padx=paddingx, pady=paddingy, sticky=tk.W)
        tk.Spinbox(self.ahrsSettingsFrame, text="Spinbox", command=self.betaGainSelect, from_=0.0, to_=5.0, increment=0.1, format="%1.2f", textvariable=self.betaGain, width=spinbox_width).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        col_num = 0
        row_num = row_num + 1

        # Accelerometer Data
        self.data_group['Accelerometer'] = AHRSDataFrame(self, "Accelerometer", ["Normalized", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width_sensors, ['Hold', 'Ideal', 'Display'], row_num, col_num)

        # Magnetometer Data
        col_num = col_num + 1
        self.data_group['Magnetometer'] = AHRSDataFrame(self, "Magnetometer", ["Normalized", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width_sensors, ['Hold', 'Ideal', 'Display'], row_num, col_num)

        
        # Gyroscope Data
        row_num = row_num + 1
        col_num = 0
        self.data_group['Gyroscope'] = AHRSDataFrame(self, "Gyroscope", ["Normalized X", "Normalized Y", "Normalized Z", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width_sensors, ['Hold', 'Ideal', 'Display'], row_num, col_num)


        # Measured Output Data Rate
        col_num = col_num + 1
        self.data_group['ODR'] = AHRSDataFrame(self, "ODR", ["Accelerometer", "Gyroscope", "Magnetometer"], 8, ['Ideal', 'Display'], row_num, col_num, sticky=tk.NW)

        # Data Plot
        col_num = col_num + 1
        row_span=2
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
            print("Proportional Gain %f" % ( self.twoKp.get()) )
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
            self.connect_thread = threading.Thread(target=self.connectPeripheral)
            self.connect_thread.start()
        else:
            self.disconnect()

    def getBLEState(self):
        if self.peripheral:
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
        scan_delegate = ScanDelegate()
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
            return

        try:
            self.peripheral = Peripheral(deviceAddr = dev_addr, addrType = 'random').withDelegate(NotifyDelegate())
        except BTLEException as e:
            self.connected.set(False)
            print(e)
            return

        print("Connected to %s" % ( dev_addr ))
        # Nordic UART Service UUID.  Vendor-specific. 
        # Value is defined Inside ble_nus_c.h as NUS_BASE_UUID
        srv = self.peripheral.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
        ch = srv.getCharacteristics()
        # Enable notifications
        for c in ch:
            for d in c.getDescriptors():
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
            srv = self.peripheral.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
            uuid_write = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
            uart_write = srv.getCharacteristics(forUUID = uuid_write)
            uart_write[0].write(cmd, withResponse=True)
        except BTLEException as e:
            self.disconnect()

    def disconnect(self):
        print("Disconnect...")
        # disable delegate from flooding app with notifications
        self.peripheral.setDelegate(None)
        # if already connected, set connect button variable
        self.connected.set(False)

    def appExit(self):
        if self.connected.get():
            self.connect_button.invoke()
        if self.peripheral is None:
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


