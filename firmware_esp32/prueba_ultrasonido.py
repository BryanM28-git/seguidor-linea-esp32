# prueba_ultrasonido.py — Prueba solo el HC-SR05, SIN WiFi ni motores.
# En Thonny: pulsa STOP, abre este archivo y pulsa F5. Ctrl+C para terminar.
import time
import config as C
from ultrasonido import HCSR05

if C.PIN_TRIG is None:
    print("En config.py pon:  PIN_TRIG, PIN_ECHO = 2, 34   y vuelve a subir config.py")
else:
    hc = HCSR05(C.PIN_TRIG, C.PIN_ECHO, 200)
    print("Pon la mano o una caja a 10, 20 y 30 cm del sensor.")
    print("Debe parar el robot cuando haya algo a menos de %d cm.\n" % C.DIST_OBSTACULO)
    try:
        while True:
            d = hc.medir()
            if d is None:
                print("  ---  (nada a menos de 2 m, o ECHO mal conectado)")
            else:
                aviso = "  <-- OBSTÁCULO" if d < C.DIST_OBSTACULO else ""
                print("  %6.1f cm %s%s" % (d, "#" * int(min(d, 60) / 2), aviso))
            time.sleep_ms(200)
    except KeyboardInterrupt:
        print("Prueba terminada.")
