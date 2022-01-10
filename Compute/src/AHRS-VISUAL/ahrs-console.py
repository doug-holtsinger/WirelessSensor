#!/usr/bin/env python3

""" Provide control of the Wireless Sensor device using a GUI.
"""

import sys
import binascii
from bluepy.btle import Scanner, DefaultDelegate, Peripheral, BTLEException

import tkinter as tk
import threading

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
        self.twoKp.set(1.0)
        self.twoKi = tk.DoubleVar()
        self.twoKi.set(0.0)
        self.sampleFreq = tk.DoubleVar()
        self.sampleFreq.set(416.0)
        self.gyroscope_sensitivity = tk.IntVar()
        self.gyroscope_sensitivity.set(16)
        self.ahrs_data = []
        for i in range(19):
            self.ahrs_data.append(tk.StringVar())
        for i in range(19):
            self.ahrs_data[i].set(0)

        self.createWidgets()
        self.peripheral = None

    def setAHRSData(self, idx, data):
        if idx >= 0 and idx <= 18:
            # print("Set AHRS Data %d = %s" % ( idx, data ))
            self.ahrs_data[idx].set(data)

    def createWidgets(self):
        row_num = 0
        col_num = 0

        paddingx = 5
        paddingy = 5

        self.connect_button = tk.Checkbutton(self, text="Connect", command=self.connectButton, variable=self.connected, onvalue=True, offvalue=False)
        self.connect_button.grid(column=col_num, row=row_num)
        col_num = col_num + 1
        tk.Button(self, text="Quit", command=self.appExit).grid(column=col_num, row=row_num)

        row_num = row_num + 1
        col_num = 0

        lf = tk.LabelFrame(self, text="Calibration")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

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
        tk.Label(lf2, relief=tk.SUNKEN, textvariable=self.ahrs_data[0]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf2, text="Pitch", justify=tk.LEFT, padx=20).grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf2, relief=tk.SUNKEN, textvariable=self.ahrs_data[1]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf2, text="Yaw", justify=tk.LEFT, padx=20).grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf2, relief=tk.SUNKEN, textvariable=self.ahrs_data[2]).grid(column=1, row=2, padx=paddingx, pady=paddingy)


        col_num = 0
        row_num = row_num + 1

        tk.Label(self, text="Proportional Gain").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", from_=0.0 , to_=5.0, increment=0.05, format="%1.2f", textvariable=self.twoKp).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(self, text="Integral Gain").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", from_=0.0, to_=5.0, increment=0.05, format="%1.2f", textvariable=self.twoKi).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(self, text="Sample Frequency").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", from_=0.0, to_=1600.0, increment=5.0, format="%4.1f", textvariable=self.sampleFreq).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(self, text="Gyroscope Sensitivity").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(self, text="Spinbox", from_=1, to_=24, increment=1, textvariable=self.gyroscope_sensitivity).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)


        # Accelerometer Data
        row_num = row_num + 1
        lf3 = tk.LabelFrame(self, text="Accelerometer")
        lf3.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf3, text="Normalized").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[3]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf3, text="Calibrated").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[4]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf3, text="Uncalibrated").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[5]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf3, text="Min Threshold").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf3, relief=tk.SUNKEN, textvariable=self.ahrs_data[6]).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        # Gyroscope Data
        col_num = col_num + 1
        lf4 = tk.LabelFrame(self, text="Gyroscope")
        lf4.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf4, text="Normalized").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[7]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf4, text="Calibrated").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[8]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf4, text="Uncalibrated").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[9]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf4, text="Min Threshold").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf4, relief=tk.SUNKEN, textvariable=self.ahrs_data[10]).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        # Magnetometer Data
        row_num = row_num + 1
        col_num = col_num - 1
        lf5 = tk.LabelFrame(self, text="Magnetometer")
        lf5.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf5, text="Normalized").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[11]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf5, text="Calibrated").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[12]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf5, text="Uncalibrated").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[13]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf5, text="Min Threshold").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf5, relief=tk.SUNKEN, textvariable=self.ahrs_data[14]).grid(column=1, row=3, padx=paddingx, pady=paddingy)

        # Quaternion Data
        col_num = col_num + 1
        lf6 = tk.LabelFrame(self, text="Quaternion")
        lf6.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf6, text="Q0").grid(column=0, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[15]).grid(column=1, row=0, padx=paddingx, pady=paddingy)
        tk.Label(lf6, text="Q1").grid(column=0, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[16]).grid(column=1, row=1, padx=paddingx, pady=paddingy)
        tk.Label(lf6, text="Q2").grid(column=0, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[17]).grid(column=1, row=2, padx=paddingx, pady=paddingy)
        tk.Label(lf6, text="Q3").grid(column=0, row=3, padx=paddingx, pady=paddingy)
        tk.Label(lf6, relief=tk.SUNKEN, textvariable=self.ahrs_data[18]).grid(column=1, row=3, padx=paddingx, pady=paddingy)


    def calibrateResetButton(self):
        print("Calibrate Reset")

    def calibrateButton(self):
        if self.connected.get():
            print("Calibrate Button %d" % ( self.calibrate.get() ))
            while self.calibrate_prev != self.calibrate.get():
                self.writeCmd(b"\x63")
                self.calibrate_prev = ( self.calibrate_prev + 1 ) % 3
        else:
            self.calibrate.set(self.calibrate_prev)

    def connectButton(self):
        if self.connected.get():
            self.connect_thread = threading.Thread(target=self.connectPeripheral)
            self.connect_thread.start()
        else:
            self.disconnect()

    def connectPeripheral(self):
        print("Connect...")
        # battery-powered device.
        dev_addr = 'F1:68:47:7C:AD:E3'
        # USB-powered device.
        dev_addr = 'CC:43:80:8D:F8:46'
        self.peripheral = None

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
        # 'a'
        #self.writeCmd(b"\x61")
        #self.writeCmd(b"\x67")
        #self.writeCmd(b"\x6D")
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
        self.connected.set(False)

    def appExit(self):
        if self.connected.get():
            self.connect_button.invoke()
        if self.peripheral is None:
            sys.exit(0)

class NotifyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleNotification(self, cHandle, data):
        str_data = str(data, encoding='utf-8').split()
        print("Notification:", str_data )
        if str_data[0] != 'Calibrate':
            idx = int(str_data[0])
            app.setAHRSData( idx, str_data[1:] )

app = GUIApplication()
app.master.title('AHRS Command')
app.mainloop()

