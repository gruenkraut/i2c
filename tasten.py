#!/usr/bin/env python
#coding: utf8 
import time
import RPi.GPIO as GPIO
# Zählweise der Pins festlegen
GPIO.setmode(GPIO.BOARD)
# Pin 18 (GPIO 24) als Eingang festlegen
GPIO.setup(18, GPIO.IN, pull_up_down = GPIO.PUD_DOWN)
# Schleifenzähler
i = 0
# Ereignis-Prozedur für Eingang HIGH
def doIfHigh(channel):
 # Zugriff auf Variable i ermöglichen
 global i
 # Wenn Eingang HIGH ist, Ausgabe im Terminal erzeugen
 print "Eingang HIGH " + str(i)
 # Schleifenzähler erhöhen
 i = i + 1
 # Ereignis deklarieren
GPIO.add_event_detect(18, GPIO.RISING, callback = doIfHigh, bouncetime = 200)
# Eigentlicher Programmablauf
while 1:
 time.sleep(0.1)
