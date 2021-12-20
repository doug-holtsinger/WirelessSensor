#!/usr/bin/env python

""" Print the manufacturing data for our Wireless Sensor device 
    to standard out.  It assumes a manufacturer ID of 0xffff and 
    a length of exactly 16 bytes for the manufacturing data.
"""

import sys
import binascii
from bluepy.btle import Scanner, DefaultDelegate, Peripheral


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
            mdev = Peripheral(dev.addr, 'random')
            srv = mdev.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
            ch = srv.getCharacteristics()
            for c in ch:
                for d in c.getDescriptors():
                    #print "  dtype:    " , type(d)
                    #print "    descr:  " , d
                    #print "    UUID :  " , d.uuid
                    val = d.read()
                    print "    Value:  ", binascii.b2a_hex(val).decode('utf-8')
                    # val = binascii.a2b_hex('0101'.encode('utf-8'))
                    d.write(b"\x01\x00",withResponse=True)
                    val = d.read()
                    print "    Value:  ", binascii.b2a_hex(val).decode('utf-8')
            uuid_write = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
            uart_write = srv.getCharacteristics(forUUID = uuid_write)
            uart_write[0].write('m', withResponse=True)
            mdev.waitForNotifications(60)
            mdev.disconnect()

class NotifyDelegate(DefaultDelegate):
    def __init__(self):
        DefaultDelegate.__init__(self)
    def handleNotification(self, cHandle, data):
        #print("Notification:", cHandle, "sent data", binascii.b2a_hex(data))
        print("Notification:", cHandle, "len", len(data), "data", str(data))

if len(sys.argv) == 2:
    cmd = sys.argv[1]
else:
    cmd = None

dev_addr = 'CC:43:80:8D:F8:46'
dev_addr = 'F1:68:47:7C:AD:E3'
mdev = Peripheral(deviceAddr = dev_addr, addrType = 'random').withDelegate(NotifyDelegate())
print("Connected")
srv = mdev.getServiceByUUID('6e400001-b5a3-f393-e0a9-e50e24dcca9e')
ch = srv.getCharacteristics()
for c in ch:
    for d in c.getDescriptors():
        val = d.read()
        print "    Value:  ", binascii.b2a_hex(val).decode('utf-8')
        d.write(b"\x01\x00",withResponse=True)
        val = d.read()
        print "    Value:  ", binascii.b2a_hex(val).decode('utf-8')

timeout = 240
if cmd is None:
    while True:
        mdev.waitForNotifications(timeout)
else:
    if cmd == 'c':
        timeout = 10
    uuid_write = '6e400002-b5a3-f393-e0a9-e50e24dcca9e'
    uart_write = srv.getCharacteristics(forUUID = uuid_write)
    uart_write[0].write(cmd, withResponse=True)
    try:
        mdev.waitForNotifications(timeout)
    except:
        pass
    mdev.disconnect()

sys.exit(0)
