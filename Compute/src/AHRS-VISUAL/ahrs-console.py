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
        self.twoKp = tk.DoubleVar()
        self.twoKp.set(1.0)
        self.twoKi = tk.DoubleVar()
        self.twoKi.set(0.0)
        self.sampleFreq = tk.DoubleVar()
        self.sampleFreq.set(416.0)
        self.gyroscope_sensitivity = tk.IntVar()
        self.gyroscope_sensitivity.set(16)
        self.createWidgets()
        self.peripheral = None

    def createWidgets(self):
        row_num = 0
        col_num = 0

        self.connect_button = tk.Checkbutton(self, text="Connect", command=self.connectButton, variable=self.connected, onvalue=True, offvalue=False)
        self.connect_button.grid(column=col_num, row=row_num)
        col_num = col_num + 1
        tk.Button(self, text="Quit", command=self.appExit).grid(column=col_num, row=row_num)

        row_num = row_num + 1
        col_num = 0

        lf = tk.LabelFrame(self, text="Calibration")
        lf.grid(column=col_num, row=row_num)
        tk.Radiobutton(lf, text="Normal", variable=self.calibrate, value=0).pack(anchor="w")
        tk.Radiobutton(lf, text="Zero Offset", variable=self.calibrate, value=1).pack(anchor="w")
        tk.Radiobutton(lf, text="Magnetometer", variable=self.calibrate, value=2).pack(anchor="w")
        tk.Button(lf, text="Reset", command=self.calibrateResetButton).pack(anchor="w")

        col_num = 0
        row_num = row_num + 1

        tk.Label(self, text="Proportional Gain").grid(column=0, row=row_num, padx=10, pady=10)
        tk.Spinbox(self, text="Spinbox", from_=0.0 , to_=5.0, increment=0.05, format="%1.2f", textvariable=self.twoKp).grid(column=1, row=row_num, padx=10, pady=10)

        row_num = row_num + 1
        tk.Label(self, text="Integral Gain").grid(column=0, row=row_num, padx=10, pady=10)
        tk.Spinbox(self, text="Spinbox", from_=0.0, to_=5.0, increment=0.05, format="%1.2f", textvariable=self.twoKi).grid(column=1, row=row_num, padx=10, pady=10)

        row_num = row_num + 1
        tk.Label(self, text="Sample Frequency").grid(column=0, row=row_num, padx=10, pady=10)
        tk.Spinbox(self, text="Spinbox", from_=0.0, to_=1600.0, increment=5.0, format="%4.1f", textvariable=self.sampleFreq).grid(column=1, row=row_num, padx=10, pady=10)

        row_num = row_num + 1
        tk.Label(self, text="Gyroscope Sensitivity").grid(column=0, row=row_num, padx=10, pady=10)
        tk.Spinbox(self, text="Spinbox", from_=1, to_=24, increment=1, textvariable=self.gyroscope_sensitivity).grid(column=1, row=row_num, padx=10, pady=10)

    def calibrateResetButton(self):
        print("Calibrate Reset")

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

        self.connect_button.state = tk.DISABLED
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
        timeout = 30
        # 'a'
        self.writeCmd(b"\x61")
        self.writeCmd(b"\x67")
        #self.writeCmd(b"\x6D")
        print("Wait for Notifications")
        self.connect_button.state = tk.NORMAL
        while self.connected.get():
            try:
                self.peripheral.waitForNotifications(timeout)
            except: 
                pass
        print("connectPeripheral: End")

    def writeCmd(self,cmd):
        try:
            srv = self.peripheral.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
            uuid_write = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
            uart_write = srv.getCharacteristics(forUUID = uuid_write)
            uart_write[0].write(cmd, withResponse=True)
        except BTLEException as e:
            self.disconnect()

    def disconnect(self):
        print("Disconnect")
        self.connected.set(False)
        if self.peripheral is not None:
            self.peripheral.disconnect()
            self.peripheral = None

    def appExit(self):
        print("appExit")
        self.disconnect()
        print("appExit: try exit")
        sys.exit(0)

class NotifyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleNotification(self, cHandle, data):
        #print("Notification:", cHandle, "sent data", binascii.b2a_hex(data))
        print("Notification:", cHandle, "len", len(data), "data", str(data))

app = GUIApplication()
app.master.title('AHRS Command')
app.mainloop()

