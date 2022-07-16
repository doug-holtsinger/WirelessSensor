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
    def __init__(self, master, data_group_name, data_names, data_label_width, row, column):
        paddingx = 5
        paddingy = 5
        lf = tk.LabelFrame(master, text=data_group_name)
        lf.grid(column=column, row=row, padx=paddingx, pady=paddingy)
        self.dataItems = []
        self.data_group_name = data_group_name
        for data_name in data_names:
            row = row + 1
            self.dataItems.append(AHRSDataItem(lf, data_name, data_label_width, row, column))

    def setData(self, idx, data):
        #print("DataFrame setData %s %d data = %s" % ( self.data_group_name, idx, data ))
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
        self.xpoints = 1500
        tk.Label(master, text=data_name, justify=tk.LEFT, padx=20).grid(column=column, row=row, padx=paddingx, pady=paddingy)
        self.data_label = tk.Label(master, relief=tk.SUNKEN, textvariable=self.data, width=data_label_width)
        self.data_label.grid(column=column+1, row=row, padx=paddingx, pady=paddingy)
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
        self.gyroSens = tk.IntVar()
        self.magnetometerStability = tk.IntVar()
        self.AHRSalgorithm = tk.IntVar()
        self.betaGain = tk.DoubleVar()

        self.resetAHRSSettings()
        self.xpoints = 1500

        self.data_group = []
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
        self.commandDict['IMU_SELECT_AHRS'] = 4
        self.commandDict['IMU_SENSOR_CALIBRATE_NORMALIZED'] = 5
        self.commandDict['IMU_SENSOR_CALIBRATE_ZERO_OFFSET'] = 6
        self.commandDict['IMU_SENSOR_CALIBRATE_MAGNETOMETER'] = 7
        self.commandDict['IMU_SENSOR_CALIBRATE_RESET'] = 8
        self.commandDict['IMU_SENSOR_CALIBRATE_SAVE'] = 9
        self.commandDict['IMU_AHRS_INPUT_TOGGLE'] = 10 
        self.commandDict['IMU_AHRS_YAW_TOGGLE'] = 11 
        self.commandDict['IMU_AHRS_PITCH_TOGGLE'] = 12
        self.commandDict['IMU_AHRS_ROLL_TOGGLE'] = 13
        self.commandDict['IMU_SENSOR_DATA_ZERO'] = 14 
        self.commandDict['IMU_SENSOR_DATA_IDEAL_TOGGLE'] = 15 
        self.commandDict['IMU_SENSOR_DATA_FIXED_TOGGLE'] = 16
        self.commandDict['IMU_AHRS_PROP_GAIN_UP'] = 17
        self.commandDict['IMU_AHRS_PROP_GAIN_DOWN'] = 18
        self.commandDict['IMU_AHRS_INTEG_GAIN_UP'] = 19
        self.commandDict['IMU_AHRS_INTEG_GAIN_DOWN'] = 20
        self.commandDict['IMU_AHRS_SAMPLE_FREQ_UP'] = 21
        self.commandDict['IMU_AHRS_SAMPLE_FREQ_DOWN'] = 22
        self.commandDict['IMU_GYROSCOPE_SENSITIVITY_UP'] = 23
        self.commandDict['IMU_GYROSCOPE_SENSITIVITY_DOWN'] = 24
        self.commandDict['IMU_MAGNETOMETER_STABILITY_TOGGLE'] = 25
        self.commandDict['IMU_AHRS_BETA_GAIN_UP'] = 26
        self.commandDict['IMU_AHRS_BETA_GAIN_DOWN'] = 27
        self.commandDict['IMU_AHRS_ALGORITHM_TOGGLE'] = 28

    def resetAHRSSettings(self):
        self.twoKp.set(0.0)
        self.twoKpClient = 0.0
        self.twoKi.set(0.0)
        self.twoKiClient = 0.0
        self.sampleFreq.set(0.0)
        self.sampleFreqClient = 0.0
        self.gyroSens.set(0)
        self.gyroSensClient = 0
        self.AHRSalgorithm.set(0)
        self.AHRSalgorithmClient = 0
        self.betaGain.set(0.0)
        self.betaGainClient = 0.0

    def scalePlot(self):
        self.ax.relim()
        self.ax.autoscale_view(tight=False, scaley=True, scalex=False)
        self.dataplot_cnv.draw_idle()

    def setAHRSData(self, idx, data):
        if idx == 0:
            # AHRS
            gidx = 0
            for gi in range(3):
                self.data_group[gidx].setData(gi, data[gi])
        elif idx <= 12:
            # Accelerometer, Magnetometer, Quaternion
            # gidx 1 to 3
            gidx = int((idx + 3) / 4)
            # gi 0 to 3
            gi = (idx + 3) & 0x3
            self.data_group[gidx].setData(gi, data)
        elif idx <= 18:
            # Gyroscope
            gidx = 4
            gi = idx - 13
            self.data_group[gidx].setData(gi, data)
        elif idx == 19:
            # Gyro sensitivity
            self.gyroSens.set(data[0])
            self.gyroSensClient = int(data[0])
        elif idx == 20:
            # Magnetometer stability
            self.magnetometerStability.set(data[0]) 
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
        self.fig, self.ax = plt.subplots(figsize=(10, 5.0), constrained_layout=True)

        # setup initial data plot
        self.data_group[0].setupPlot(self.ax)
        # self.data_group[1].setupPlot(self.ax)

        self.ax.set_xlabel('Time')
        self.ax.set_ylabel('Value');
        self.ax.legend()

        self.ax.relim()
        self.ax.autoscale_view(tight=False, scaley=True, scalex=False)

        self.dataplot_cnv = FigureCanvasTkAgg(self.fig, master=self)
        self.dataplot_cnv.draw()
        self.dataplot_cnv.get_tk_widget().grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy, rowspan=row_span)


    def createWidgets(self):
        row_num = 0
        col_num = 0

        paddingx = 5
        paddingy = 5
        data_label_width = 17

        # 0,0
        # Menu
        self.connect_button = tk.Checkbutton(self, text="Connect", command=self.connectButton, variable=self.connected, onvalue=True, offvalue=False)
        self.connect_button.grid(column=col_num, row=row_num)
        col_num = col_num + 1
        tk.Button(self, text="Quit", command=self.appExit).grid(column=col_num, row=row_num)
        col_num = col_num + 1
        tk.Button(self, text="Scale", command=self.scalePlot).grid(column=col_num, row=row_num)

        # 1,0
        # Calibration
        col_num = 0
        row_num = row_num + 1
        data_row_num = row_num

        lf = tk.LabelFrame(self, text="IMU Calibration")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)
        first_ctrl_row_num = row_num

        self.cb = []
        self.calibrateNormalizedButton = tk.Radiobutton(lf, text="Normalized", command=self.calibrateButton, variable=self.calibrate, value=0)
        self.cb.append(self.calibrateNormalizedButton)
        self.cb.append(tk.Radiobutton(lf, text="Zero Offset", command=self.calibrateButton, variable=self.calibrate, value=1))
        self.cb.append(tk.Radiobutton(lf, text="Magnetometer", command=self.calibrateButton, variable=self.calibrate, value=2))
        for cb in self.cb:
            cb.pack(anchor="w")

        tk.Button(lf, text="Reset", command=self.calibrateResetButton).pack(anchor="w")
        tk.Button(lf, text="Save", command=self.calibrateSaveButton).pack(anchor="w")

        # 1,1
        # Euler Angles
        col_num = col_num + 1
        self.data_group.append(AHRSDataFrame(self, "Euler Angles", ["Roll", "Pitch", "Yaw"], data_label_width, row_num, col_num))

        col_num = 0
        row_num = row_num + 1

        lf = tk.LabelFrame(self, text="IMU Settings")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)
        tk.Label(lf, text="Gyroscope Sensitivity").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(lf, text="Spinbox", command=self.gyroSensitivitySelect, from_=1, to_=24, increment=1, textvariable=self.gyroSens).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)
        tk.Checkbutton(lf, text="Magnetometer Stability", command=self.magnetometerStabilityButton, variable=self.magnetometerStability).grid(column=0, row=row_num+1, padx=paddingx, pady=paddingy)

        col_num = col_num + 1

        # 2,0
        # Controls
        lf = tk.LabelFrame(self, text="AHRS Settings")
        lf.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        #tk.Radiobutton(lf, text="Algorithm", command=self.AHRSAlgorithmSelect, textvariable=self.AHRSalgorithm).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        #row_num = row_num + 1
        #lf2 = tk.LabelFrame(self, text="Algorithm")
        #lf2.grid(column=col_num, row=row_num, padx=paddingx, pady=paddingy)

        tk.Label(lf, text="Algorithm").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Radiobutton(lf, text="Mahony", command=self.AHRSAlgorithmSelect, variable=self.AHRSalgorithm, value=0).grid(column=1, row=row_num)
        tk.Radiobutton(lf, text="Madgwick", command=self.AHRSAlgorithmSelect, variable=self.AHRSalgorithm, value=1).grid(column=2, row=row_num)

        row_num = row_num + 1
        tk.Label(lf, text="Proportional Gain").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(lf, text="Spinbox", command=self.proportionalGainSelect, from_=0.0 , to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKp).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(lf, text="Integral Gain").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(lf, text="Spinbox", command=self.integralGainSelect, from_=0.0, to_=5.0, increment=0.1, format="%1.2f", textvariable=self.twoKi).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(lf, text="Sample Frequency").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(lf, text="Spinbox", command=self.sampleFrequencySelect, from_=0.0, to_=1600.0, increment=32.0, format="%4.1f", textvariable=self.sampleFreq).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        row_num = row_num + 1
        tk.Label(lf, text="Beta Gain").grid(column=0, row=row_num, padx=paddingx, pady=paddingy)
        tk.Spinbox(lf, text="Spinbox", command=self.betaGainSelect, from_=0.0, to_=5.0, increment=0.1, format="%1.2f", textvariable=self.betaGain).grid(column=1, row=row_num, padx=paddingx, pady=paddingy)

        col_num = 0
        row_num = row_num + 1

        # Accelerometer Data
        self.data_group.append(AHRSDataFrame(self, "Accelerometer", ["Normalized", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width, row_num, col_num))

        # Magnetometer Data
        col_num = col_num + 1
        self.data_group.append(AHRSDataFrame(self, "Magnetometer", ["Normalized", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width, row_num, col_num))

        # Quaternion Data
        row_num = row_num + 1
        col_num = 0
        self.data_group.append(AHRSDataFrame(self, "Quaternion", ["Q0", "Q1", "Q2", "Q3"], data_label_width, row_num, col_num))

        # Gyroscope Data
        col_num = col_num + 1
        self.data_group.append(AHRSDataFrame(self, "Gyroscope", ["Normalized X", "Normalized Y", "Normalized Z", "Calibrated", "Uncalibrated", "Min Threshold"], data_label_width, row_num, col_num))

        last_ctrl_row_num = row_num

        # 1,2  row_span=4-1 = 3
        col_num = col_num + 1
        row_span=last_ctrl_row_num - first_ctrl_row_num + 1
        self.createWidgetPlot(data_row_num, col_num, row_span)

    def magnetometerStabilityButton(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            self.writeCmdStr('IMU_MAGNETOMETER_STABILITY_TOGGLE')

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

    def gyroSensitivitySelect(self):
        if not self.connected.get():
            print("Not connected to AHRS")
        else:
            print("Gyroscope Sensitivity %d" % ( self.gyroSens.get()) )
            if self.gyroSensClient < self.gyroSens.get():
                # Send up
                print("Gyroscope Sensitivity Up from %f" % ( self.gyroSensClient ) )
                self.writeCmdStr('IMU_GYROSCOPE_SENSITIVITY_UP')
            elif self.gyroSensClient > self.gyroSens.get():
                # Send down
                print("Gyroscope Sensitivity Down from %f" % ( self.gyroSensClient ) )
                self.writeCmdStr('IMU_GYROSCOPE_SENSITIVITY_DOWN')

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


