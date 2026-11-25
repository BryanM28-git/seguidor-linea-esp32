# ultrasonido.py — HC-SR05 (ECHO a 3,3 V mediante divisor 1k/2k)
import time
from machine import Pin, time_pulse_us


class HCSR05:
    def __init__(self, trig, echo, max_cm=80):
        self.trig = Pin(trig, Pin.OUT, value=0)
        self.echo = Pin(echo, Pin.IN)
        self.timeout = int(max_cm * 58) + 600   # limita cuánto se bloquea el programa
        self.cm = None

    def medir(self):
        self.trig.value(0)
        time.sleep_us(2)
        self.trig.value(1)
        time.sleep_us(10)
        self.trig.value(0)
        t = time_pulse_us(self.echo, 1, self.timeout)
        self.cm = None if t < 0 else round(t / 58.0, 1)   # None = nada dentro del alcance
        return self.cm
