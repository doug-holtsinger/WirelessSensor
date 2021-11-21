#!/usr/bin/env python

# from datetime import datetime
# import binascii
import sys
from bluepy.btle import Scanner, DefaultDelegate

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.pkt_num = 0

    def handleDiscovery(self, dev, isNewDev, isNewData):
        # print(datetime.now().time(), dev.rawData, dev.addr)
        # if dev.addr == "cc:43:80:8d:f8:46":
        if dev.addr == "f1:68:47:7c:ad:e3":
            manufacturer_data_t = dev.getValueText(255)[4:]
            roll = manufacturer_data_t[0:4]
            pitch = manufacturer_data_t[4:8]
            yaw   = manufacturer_data_t[8:12]
            print roll + ' ' + pitch + ' ' + yaw + ' ' + str(self.pkt_num)
            sys.stdout.flush()
            self.pkt_num = self.pkt_num + 1


scanner = Scanner().withDelegate(ScanDelegate())
scanner.clear()
scanner.start(passive=True)

while True:
    scanner.process(60)
