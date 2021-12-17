#!/usr/bin/env python

""" Print the manufacturing data for our Wireless Sensor device 
    to standard out.  It assumes a manufacturer ID of 0xffff and 
    a length of exactly 16 bytes for the manufacturing data.
"""

import sys
from bluepy.btle import Scanner, DefaultDelegate

class ScanDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
        self.pkt_num = 0

    def handleDiscovery(self, dev, isNewDev, isNewData):
        try:
            manufacturer_data = dev.getValueText(255)
        except BTLEException as e:
            print(e)
            sys.exit(1) 
        if manufacturer_data is not None and len(manufacturer_data) == 16 and manufacturer_data[0:4] == 'ffff': 
            # print("dev.addr %s data %s len %d" % ( dev.addr , manufacturer_data[0:4] , len(manufacturer_data)))
            roll = manufacturer_data[0:4]
            pitch = manufacturer_data[4:8]
            yaw   = manufacturer_data[8:12]
            print roll + ' ' + pitch + ' ' + yaw + ' ' + str(self.pkt_num)
            # print dev.rawData
            sys.stdout.flush()
            self.pkt_num = self.pkt_num + 1


scanner = Scanner().withDelegate(ScanDelegate())
scanner.clear()
scanner.start(passive=True)

while True:
    scanner.process(60)
