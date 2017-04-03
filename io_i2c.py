#!/usr/bin/env python
import paho.mqtt.client as mqtt
import smbus
bus = smbus.SMBus(1)
# requires RPi_I2C_driver.py
import RPi_I2C_driver
import Adafruit_ADS1x15
from time import *
import logging
import logging.handlers
import time
import datetime
import RPi.GPIO as GPIO

# commands
IO_WC_AMP = 0x01
IO_BAD_AMP = 0x02
IO_WC_LUEFTER = 0x04
IO_GARTEN_LICHT = 0x08
IO_GARTEN_STECKDOSE = 0x10

#my_io = RPi_I2C_driver.i2c_device()

log = logging.getLogger("__name__")
log.setLevel(logging.DEBUG)
handler = logging.handlers.SysLogHandler(address = '/dev/log')

formatter = logging.Formatter('%(module)s.%(funcName)s: %(message)s')
handler.setFormatter(formatter)

log.addHandler(handler)

iLoopTime=5

iUptime=0
iUptimeLoop=60/iLoopTime-1

iADC = [0,0,0,0,0]
iADCValid=0

def l_out(client,x,y,sOut):
    client.publish("keller01/display","%02X,%02X,%s" % (x,y,sOut))

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection. Reconnecting...")
        l_out(client,0,1,'MQTT not connected')
        time.sleep(1)
        client.reconnect()
    else :
        print "Disconnected successfully"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    log.debug('Connected with result code ' + str(rc))
    l_out(client,0,1,'MQTT connected    ')

    client.subscribe("io/wc_amp/#")
    client.subscribe("io/bad_amp/#")
    client.subscribe("io/wc_luefter/#")
    client.subscribe("io/garten/mauerlicht/#")
    client.subscribe("io/garten/steckdose/#")
    client.subscribe("io/wozi/sockel1/#")
    client.subscribe("io/wozi/sockel2/#")
    client.subscribe("io/wozi/sockel3/#")
    client.subscribe("io/wozi/sockel4/#")
    client.subscribe("io/wozi/sockel5/#")
    client.subscribe("io/wozi/sockel6/#")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))
    log.debug('Received: ' + msg.topic + " " + str(msg.payload))

    #print("Gelesen: %X" % io_device_38.read())
    #log.debug('I2C ID 38 Read: $%2X' % io_device_38.read())

    
    io_topic_all = msg.topic
    
    io_topic = msg.topic.split('/')[-1].lower()
    if io_topic == "bad_amp":
        io(client, io_device_38, msg.payload, IO_BAD_AMP)
    if io_topic == "wc_amp":
        io(client, io_device_38, msg.payload, IO_WC_AMP)
    if io_topic == "wc_luefter":
        io(client, io_device_38, msg.payload, IO_WC_LUEFTER)
    if io_topic == "mauerlicht":
        io(client, io_device_38, msg.payload, IO_GARTEN_LICHT)
    if io_topic == "steckdose":
        io(client, io_device_38, msg.payload, IO_GARTEN_STECKDOSE)
    if io_topic == "sockel1":
        dimmer(client, io_device_10, msg.payload, 7)
    if io_topic == "sockel2":
        dimmer(client, io_device_10, msg.payload, 8)
    if io_topic == "sockel3":
        dimmer(client, io_device_10, msg.payload, 9)
    if io_topic == "sockel4":
        dimmer(client, io_device_10, msg.payload, 10)
    if io_topic == "sockel5":
        dimmer(client, io_device_10, msg.payload, 6)
    if io_topic == "sockel6":
        dimmer(client, io_device_10, msg.payload, 11)


def dimmer(client, device, do, channel):
    iA=int(str(do))
    iA=(iA+1)*(iA+1)-1
    print(iA)
    iAlow = iA & 0x7f
    iAhigh = (iA >> 7) & 0x7f
    print("Channel: %d Wertlow: %d Werthigh: %d" % (channel, iAlow, iAhigh))
    bus.write_i2c_block_data(0x10, channel | 0x80 , [iAlow,iAhigh])
    #device.write_block_data(channel | 0x80, [iAlow,iAhigh])
    l_out(client,0,3+(channel-7),'Dimmer Ch%d: 0x%04X' % (channel, iA))

def io(client, device, do, data):
    io_in = device.read()
    #print("Gelesen: %X" % io_in)
    if(do == "ON"):
        io_out = io_in & ~data
    if(do == "OFF"):
        io_out = io_in | data
    print("Setzte: %X" % io_out)
    device.write_cmd(io_out)
    log.debug('I2C ID 38 Write: $%2X' % (io_out))
    l_out(client,20,3,'IIC 0x38: 0x%2X' % (io_out))

def doIfLow(channel):
    global client, device
    io_in = device.read()
    print("I2C Interrupt. 0x%02X gelesen" % (io_in))
    if((io_in & 0x80) == 0):
        client.publish("wc/licht/status", "ON")
    else:
        client.publish("wc/licht/status", "OFF")



client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message

#LCD Init
l_out(client,0,3,'Dimmer Ch0: ------')
l_out(client,20,3,'IIC 0x38: ----')


client.connect("192.168.101.240", 1883, 60)

#I2C Init
io_device_38 = RPi_I2C_driver.i2c_device(0x38)
io_device_10 = RPi_I2C_driver.i2c_device(0x10) #Dimmer
adc = Adafruit_ADS1x15.ADS1115()

#GPIO Init
# Zaehlweise der Pins festlegen
GPIO.setmode(GPIO.BOARD)
# Pin 18 (GPIO 24) als Eingang festlegen
GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_UP)
GPIO.add_event_detect(18, GPIO.FALLING, callback=doIfLow, bouncetime=200)


#client.loop_forever()
client.loop_start()

while True:
    iUptimeLoop=iUptimeLoop+1
    if(iUptimeLoop == 60/iLoopTime):
        iUptimeLoop=0
        iUptime=iUptime+1
        client.publish("net/health/keller01",iUptime)
        h, m = divmod(iUptime,60)
        d, h = divmod(h, 24)
        l_out(client,0,19,'UP:%03dd %02dh %02dm' % (d, h, m))
    time.sleep(iLoopTime)
    #print(iADC)
    iADC = [iADC[-1]] + iADC[:-1]
    iADC[0] = adc.read_adc(0,1)
    #print(iADC)
    iA=(2.5-(sum(iADC)/len(iADC))*(4.096/32767))*0.185
    #print(iA)
    if(iADCValid<len(iADC)+1):
        iADCValid=iADCValid+1
    else:
        client.publish("keller01/adc/0",iA)
