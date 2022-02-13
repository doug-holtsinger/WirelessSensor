#!/usr/bin/env python3

""" Provide control of the Wireless Sensor device using a GUI.
"""

import sys
import binascii
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, BTLEException

import tkinter as tk
import threading
import matplotlib as mpl
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import numpy as np

class GUIApplication(tk.Frame):
    def __init__(self, master=None):
        tk.Frame.__init__(self, master)
        self.grid()

        self.connected = tk.BooleanVar()
        self.connected.set(False)
        self.connect_thread = None
        self.connect_button = None

        self.calibrate = tk.IntVar()
        self.calibrate.set(0)
        self.calibrate_prev = 0

        self.twoKp = tk.DoubleVar()
        self.twoKi = tk.DoubleVar()
        self.sampleFreq = tk.DoubleVar()
        self.gyroSens = tk.IntVar()

        self.resetCalibration()
        self.xpoints = 1500

        self.ahrs_data = []
        for i in range(17):
            self.ahrs_data.append(tk.StringVar())
        for i in range(17):
            self.ahrs_data[i].set(0)

        self.dataplot_cnv = None
        self.line = []
        self.euler_angles = []
        self.dataplot = []
        self.dataplotidx = 0
        for i in range(3):
            self.line.append(None)
            self.euler_angles.append(tk.StringVar())
            self.dataplot.append(np.zeros(self.xpoints))
        for i in range(3):
            self.euler_angles[i].set(0)

        self.createWidgets()
        self.peripheral = None

    def resetCalibration(self):
        self.twoKp.set(1.0)
        self.twoKpSave = 1.0
        self.twoKi.set(0.0)
        self.twoKiSave = 0.0
        self.sampleFreq.set(416.0)
        self.sampleFreqSave = 416.0
        self.gyroSens.set(16)
        self.gyroSensSave = 16

    def scalePlot(self):
        self.ax.relim()
        self.ax.autoscale_view(tight=False, scaley=True, scalex=False)
        self.dataplot_cnv.draw_idle()

    def setAHRSData(self, idx, data):
        #if (self.num_notifications & 0xF) == 0:
        #    print("num notif %d" % ( self.num_notifications ))
        #if idx >= 0 and idx <= 2:
        #    print("Data idx %d %s" % ( idx , data ))
        if idx >= 0 and idx <= 16:
            self.ahrs_data[idx].set(data)
        if idx == 0 and self.dataplot_cnv is not None:
            # print("Data idx %d %s" % ( idx , data ))
            for i in range(3):
                self.euler_angles[i].set(data[i])
                self.dataplot[i][self.dataplotidx] = float(data[i])
                self.line[i].set_data(np.arange(self.xpoints), self.dataplot[i])
            if self.dataplotidx == 0:
                self.scalePlot()
            self.dataplotidx = ( self.dataplotidx + 1 ) % self.xpoints
            self.num_notifications = self.num_notifications + 1
            if self.num_notifications & 0x1f == 0:
                # draw_idle is very slow.
                self.dataplot_cnv.draw_idle()

    def createWidgetPlot(self, row_num, col_num, row_span):
        # Canvas
        mpl.use("TkAgg")

        paddingx = 5
        paddingy = 5
        self.fig, self.ax = plt.subplots(figsize=(10, 5.0), constrained_layout=True)
        for i in range(3):
            if i == 0:
                l = 'Roll'
            if i == 1:
                l = 'Pitch'
            if i == 2:
                l = 'Yaw'
            self.line[i], = self.ax.plot(np.arange(self.xpoints), self.dataplot[i], label=l)
        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value');
        self.ax.legend()

        #self.ax.set_ylim(bottom=-90.0, top=90.0)
        self.ax.relim()
        self.ax.autoscale_view(tight=False, scaley=True, scalex=False)

        #self.ax.autoscale(enable=True, axis='both')
        # self.ax.set_autoscaley_on(True)
        # self.ax.set_autoscaley_on(True)
        # self.ax.autoscale(enable=True, axis='both')
        # self.ax.set_ylim(auto=True)

        # print( self.ax.format_ydata(0.1) )
        # print( self.ax.get_children() )

        self.dataplot_cnv = FigureCanvasTkAgg(self.fig, master=self)
        self.dataplot_cnv.draw()
        # cnv.get_tk_widget().grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, rowspan=last_ctrl_row_num - first_ctrl_row_num + 1)
        self.dataplot_cnv.get_tk_widget().grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, rowspan=row_span)

    def createWidgets(self):
        row_num = 0
        col_num = 0

        paddingx = 5
        paddingy = 5

        self.connect_button = tk.Checkbutton(self, text="Connect", command=self.connectButton, variable=self.connected, onvalue=True, offvalue=False)
        self.connect_button.grid(column=col_num, row=row_num)
        col_num = col_num + 1
        tk.Button(self, text="Quit", command=self.appExit).grid(column=col_num, row=row_num)
        col_num = col_num + 1
        tk.Button(self, text="Scale", command=self.scalePlot).grid(column=col_num, row=row_num)

        row_num = row_num + 1
        data_row_num = row_num
        col_num = 0

        lf = tk.LabelFrame(self, text="Calibration")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)
        first_ctrl_row_num = row_num

        self.cb = []
        self.cb.append(tk.Radiobutton(lf, text="Normalized", command=self.calibrateButton, variable=self.calibrate, value=0))
        self.cb.append(tk.Radiobutton(lf, text="Zero Offset", command=self.calibrateButton, variable=self.calibrate, value=1))
        self.cb.append(tk.Radiobutton(lf, text="Magnetometer", command=self.calibrateButton, variable=self.calibrate, value=2))
        for cb in self.cb:
            cb.pack(anchor="w")

        tk.Button(lf, text="Reset", command=self.calibrateResetButton).pack(anchor="w")

        # Euler Angles
        col_num = col_num + 1
        lf2 = tk.LabelFrame(self, text="Euler Angles")
        lf2.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf2, text="Roll", justify=tk.LEFT, padx=20).grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf2, relief=tk.SUNKEN, textvariable=self.euler_angles[0]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf2, text="Pitch", justify=tk.LEFT, padx=20).grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf2, relief=tk.SUNKEN, textvariable=self.euler_angles[1]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf2, text="Yaw", justify=tk.LEFT, padx=20).grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf2, relief=tk.SUNKEN, textvariable=self.euler_angles[2]).grid(column=1, row=2, padx=paddingx, pady=paddingy)

        col_num = 0
        row_num = row_num + 1

        tk.Label(self, text="Proportional Gain").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", command=self.proportionalGain, from_=0.0 , to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKp).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(self, text="Integral Gain").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", command=self.integralGain, from_=0.0, to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKi).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(self, text="Sample Frequency").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", command=self.sampleFrequency, from_=0.0, to_=1600.0, increment=32.0, format="%4.1f", textvariable=self.sampleFreq).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(self, text="Gyroscope Sensitivity").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", command=self.gyroSensitivity, from_=1, to_=24, increment=1, textvariable=self.gyroSens).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        # Accelerometer Data
        row_num = row_num + 1
        lf3 = tk.LabelFrame(self, text="Accelerometer")
        lf3.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf3, text="Normalized").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[1]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf3, text="Calibrated").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[2]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf3, text="Uncalibrated").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[3]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf3, text="Min Threshold").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[4]).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        # Gyroscope Data
        col_num = col_num + 1
        lf4 = tk.LabelFrame(self, text="Gyroscope")
        lf4.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf4, text="Normalized").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[5]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf4, text="Calibrated").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[6]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf4, text="Uncalibrated").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[7]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf4, text="Min Threshold").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[8]).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        # Magnetometer Data
        row_num = row_num + 1
        col_num = col_num - 1
        lf5 = tk.LabelFrame(self, text="Magnetometer")
        lf5.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf5, text="Normalized").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[9]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf5, text="Calibrated").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[10]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf5, text="Uncalibrated").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[11]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf5, text="Min Threshold").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[12]).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        # Quaternion Data
        col_num = col_num + 1
        last_ctrl_row_num = row_num
        lf6 = tk.LabelFrame(self, text="Quaternion")
        lf6.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf6, text="Q0").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[13]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf6, text="Q1").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[14]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf6, text="Q2").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[15]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf6, text="Q3").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[16]).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        col_num = col_num + 1
        row_span=last_ctrl_row_num - first_ctrl_row_num + 1
        self.createWidgetPlot(data_row_num, col_num, row_span)

    def proportionalGain(self):
        print("Proportional Gain %f" % ( self.twoKp.get()) )
        if self.twoKpSave < self.twoKp.get():
            # Send up
            print("Prop Gain Up from %f" % ( self.twoKpSave) )
            self.writeCmd(b"h")
            self.twoKpSave = self.twoKpSave + 0.1
        else:
            # Send down
            print("Prop Gain Down from %f" % ( self.twoKpSave) )
            self.writeCmd(b"j")
            self.twoKpSave = self.twoKpSave - 0.1
        print("twoKp = %f" % ( self.twoKpSave ))

    def integralGain(self):
        print("Integral Gain %f" % ( self.twoKi.get()) )
        if self.twoKiSave < self.twoKi.get():
            # Send up
            print("Integral Gain Up from %f" % ( self.twoKiSave) )
            self.writeCmd(b"l")
            self.twoKiSave = self.twoKiSave + 0.1
        else:
            # Send down
            print("Integral Gain Down from %f" % ( self.twoKiSave) )
            self.writeCmd(b"n")
            self.twoKiSave = self.twoKiSave - 0.1
        print("twoKi = %f" % ( self.twoKiSave ))

    def sampleFrequency(self):
        print("Sample Frequency %f" % ( self.sampleFreq.get()) )
        if self.sampleFreqSave < self.sampleFreq.get():
            # Send up
            print("Sample Frequency Up from %f" % ( self.sampleFreqSave ) )
            self.writeCmd(b"s")
            self.sampleFreqSave = self.sampleFreqSave + 32.0
        else:
            # Send down
            print("Sample Frequency Down from %f" % ( self.sampleFreqSave) )
            self.writeCmd(b"o")
            self.sampleFreqSave = self.sampleFreqSave - 32.0
        print("sampleFreq = %f" % ( self.sampleFreqSave ))

    def gyroSensitivity(self):
        print("Gyroscope Sensitivity %d" % ( self.gyroSens.get()) )
        if self.gyroSensSave < self.gyroSens.get():
            # Send up
            print("Gyroscope Sensitivity Up from %f" % ( self.gyroSensSave ) )
            self.writeCmd(b"u")
            self.gyroSensSave = self.gyroSensSave + 1
        else:
            # Send down
            print("Gyroscope Sensitivity Down from %f" % ( self.gyroSensSave ) )
            self.writeCmd(b"t")
            self.gyroSensSave = self.gyroSensSave - 1
        print("gyroSens = %d" % ( self.gyroSensSave ))

    def calibrateResetButton(self):
        print("Calibrate Reset")
        self.writeCmd(b"e")
        self.resetCalibration()

    def calibrateButton(self):
        if self.connected.get():
            print("Calibrate Button %d" % ( self.calibrate.get() ))
            while self.calibrate_prev != self.calibrate.get():
                self.writeCmd(b"c")
                self.calibrate_prev = ( self.calibrate_prev + 1 ) % 3
        else:
            self.calibrate.set(self.calibrate_prev)

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

    def connectPeripheral(self):
        print("Connect...")
        # FIXME
        # USB-powered device.
        dev_addr = 'CC:43:80:8D:F8:46'
        # battery-powered device.
        # dev_addr = 'F1:68:47:7C:AD:E3'
        self.peripheral = None
        self.num_notifications = 0

        try:
            self.peripheral = Peripheral(deviceAddr = dev_addr, addrType = 'random').withDelegate(NotifyDelegate())
        except BTLEException as e:
            self.connected.set(False)
            print(e)
            return

        print("Connected")
        srv = self.peripheral.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
        ch = srv.getCharacteristics()
        for c in ch:
            for d in c.getDescriptors():
                val = d.read()
                print("    Value:  ", binascii.b2a_hex(val).decode('utf-8'))
                d.write(b"\x01\x00",withResponse=True)
                val = d.read()
                print("    Value:  ", binascii.b2a_hex(val).decode('utf-8'))
        print("Wait for notifications")
        while self.connected.get():
            try:
                self.peripheral.waitForNotifications(0.1)
            except:
                pass
        self.peripheral.disconnect()
        self.peripheral = None
        print("Disconnected")

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
            app.setAHRSData( idx, str_data[1:] )
        except:
            # raise
            print("Notif: %s" % ( str_data ))


app = GUIApplication()
app.master.title('AHRS Console')
app.mainloop()


