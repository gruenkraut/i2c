import paho.mqtt.client as mqtt
import smbus
bus = smbus.SMBus(1)
import time

iLoopTime=30

iUptime=0
iUptimeLoop=60/iLoopTime-1

def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("Unexpected disconnection. Reconnecting...")
        time.sleep(1)
        client.reconnect()
    else :
        print "Disconnected successfully"


def on_connect(client, userdata, flags, rc):
    print("Connected with result code " + str(rc))
    client.subscribe("io/wohnen/#")

def on_message(client, userdata, msg):
    print(msg.topic + " " + str(msg.payload))

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect("192.168.101.240", 1883, 60)

client.loop_start()

while True:
    iUptimeLoop=iUptimeLoop+1
    print("%d - %d" % (iUptimeLoop,60/iLoopTime))
    if(iUptimeLoop == 60/iLoopTime):
        iUptimeLoop=0
        iUptime=iUptime+1
        client.publish("net/health/wohnen01", "%d" % (iUptime))
        print("Uptime: %d" % iUptime)
    time.sleep(iLoopTime)
    # SI7021 address, 0x40(64)
    #		0xF5(245)	Select Relative Humidity NO HOLD master mode
    bus.write_byte(0x40, 0xF5)
    time.sleep(0.3)
    # SI7021 address, 0x40(64)
    # Read data back, 2 bytes, Humidity MSB first
    data0 = bus.read_byte(0x40)
    data1 = bus.read_byte(0x40)
    # Convert the data
    humidity = ((data0 * 256 + data1) * 125 / 65536.0) - 6
    print "Relative Humidity is : %.1f %%" %humidity
    client.publish("wohnen01/humidity/status", "%0.1f" % humidity)
    time.sleep(0.3)
    # SI7021 address, 0x40(64)
    #		0xF3(243)	Select temperature NO HOLD master mode
    bus.write_byte(0x40, 0xF3)
    time.sleep(0.3)
    # SI7021 address, 0x40(64)
    # Read data back, 2 bytes, Temperature MSB first
    data0 = bus.read_byte(0x40)
    data1 = bus.read_byte(0x40)
    # Convert the data
    cTemp = ((data0 * 256 + data1) * 175.72 / 65536.0) - 46.85
    print "Temperature in Celsius is : %.1f C" %cTemp
    client.publish("wohnen01/temperatur/status", "%0.1f" % cTemp)
