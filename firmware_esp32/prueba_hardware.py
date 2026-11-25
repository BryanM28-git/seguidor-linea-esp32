# prueba_hardware.py — Prueba rápida del cableado, SIN WiFi.
# En Thonny: primero sube los archivos del firmware al ESP32, luego abre ESTE archivo y pulsa F5.
# Si main.py ya está corriendo, pulsa antes el botón rojo STOP (o Ctrl+C en la consola).
import time
import config as C
from motores import Motores
from regleta import RegletaRC
from color import TCS3200

mot = Motores(C.PIN_IN1, C.PIN_IN2, C.PIN_IN3, C.PIN_IN4, C.PWM_FREQ,
              C.PWM_MIN, C.INVERTIR_IZQ, C.INVERTIR_DER, C.AJUSTE_RECTO)
mot.parar()
pines = tuple(reversed(C.PIN_QRE)) if C.INVERTIR_REGLETA else C.PIN_QRE
reg = RegletaRC(pines, C.RC_TIMEOUT_US)
tcs = TCS3200(C.PIN_S2, C.PIN_S3, C.PIN_TCS_OUT)

print("\n=== 1. MOTORES: levanta el robot para que las ruedas giren al aire ===")
time.sleep(2)
for nombre, iz, de in (("izquierdo ADELANTE", 60, 0), ("izquierdo ATRÁS", -60, 0),
                       ("derecho ADELANTE", 0, 60), ("derecho ATRÁS", 0, -60),
                       ("los dos ADELANTE", 60, 60)):
    print("  ->", nombre)
    mot.mover(iz, de)
    time.sleep(1.2)
    mot.parar()
    time.sleep(0.5)
print("  Si una rueda giró al revés: pon INVERTIR_IZQ o INVERTIR_DER = True en config.py")

print("\n=== 2. REGLETA y COLOR (Ctrl+C para terminar) ===")
print("  Pon el robot EN EL PISO sobre papel blanco (al aire todo marca 2500, es normal).")
print("  Regleta: tiempo de descarga en µs. Blanco = bajo (~100-600), negro/aire = alto (~1500-2500).")
print("  Pasa una cinta negra bajo cada sensor; el de la IZQUIERDA debe cambiar la PRIMERA columna.")
print("  Color: frecuencias en Hz; más alto = más luz de ese color.\n")
mini = [9999] * 8
maxi = [0] * 8
try:
    while True:
        crudo = reg.leer_crudo()
        for i, v in enumerate(crudo):
            mini[i] = min(mini[i], v)
            maxi[i] = max(maxi[i], v)
        r, g, b, c = tcs.leer()
        barra = "".join("#" if v > 1200 else "." for v in crudo)
        print("[%s] %s | R %5.0f  G %5.0f  B %5.0f  C %5.0f" %
              (barra, " ".join("%4d" % v for v in crudo), r, g, b, c))
        time.sleep_ms(250)
except KeyboardInterrupt:
    mot.parar()
    print("\nResumen regleta (mín / máx de cada sensor):")
    for i in range(8):
        estado = "OK" if maxi[i] - mini[i] > 600 else "NO CAMBIÓ"
        print("  S%d  GPIO%-2d  %4d / %4d  %s" % (i + 1, pines[i], mini[i], maxi[i], estado))
    print("Si todos dicen NO CAMBIÓ: VCC de la regleta a 5 V y revisa su GND.")
    print("Prueba terminada.")
