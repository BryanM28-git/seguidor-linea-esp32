# config.py — Parámetros del robot seguidor de línea (ESP32 DevKit 30 pines, MicroPython)
# Todo lo que se ajusta está aquí. Los demás archivos no deberían necesitar cambios.

# ------------------------------------------------------------------ RED
WIFI_MODO = "STA"            # "STA": se conecta al hotspot del celular/router (recomendado: el PC conserva internet)
                             # "AP" : el robot crea su propia red (el PC pierde internet -> la voz no funciona)
WIFI_SSID = "MiHotspot"
WIFI_PASS = "clave1234"
AP_SSID = "Seguidor-UMNG"
AP_PASS = "robot1234"
HOSTNAME = "seguidor"        # en STA se puede abrir como http://seguidor.local
PUERTO = 80

# ------------------------------------------------------------------ PINES
PIN_QRE = (32, 33, 25, 26, 27, 14, 13, 4)   # regleta RC: D1..D8 en este orden (IR a 3V3 o sin conectar)
INVERTIR_REGLETA = False     # pon True si al tapar el sensor de la IZQUIERDA sube la barra S8 en la app
PIN_IN1, PIN_IN2 = 23, 19                    # L298N motor izquierdo (ENA con jumper puesto)
PIN_IN3, PIN_IN4 = 18, 17                    # L298N motor derecho  (ENB con jumper puesto)
PIN_SDA, PIN_SCL = 21, 22                    # brújula I2C opcional (GY-271); sin ella quedan libres
PIN_S2, PIN_S3, PIN_TCS_OUT = 16, 15, 35     # TCS3200 (S0 -> 3V3, S1 -> GND, OE -> GND)
PIN_TRIG, PIN_ECHO = 2, 34                 # HC-SR05 (ECHO con divisor 1k/2k). Sin sensor: None, None
PIN_BAT = None                               # pon 36 si conectaste el divisor 100k/47k de la batería
PIN_SERVO = None                             # opcional: SG90 que orienta el HC-SR05 en curvas (usa 5)
BAT_DIVISOR = (100 + 47) / 47

# ------------------------------------------------------------------ MOTORES
PWM_FREQ = 1000
INVERTIR_IZQ = False         # si un motor gira al revés, cambia esto en vez de invertir cables
INVERTIR_DER = False
PWM_MIN = 28                 # % mínimo para que el TT arranque (zona muerta)

# ------------------------------------------------------------------ SEGUIDOR (PID)
LINEA_NEGRA = True           # False si la línea es blanca sobre fondo oscuro
RC_TIMEOUT_US = 2500
UMBRAL_LINEA = 300           # 0..1000: valor mínimo para decir que un sensor ve línea
VEL_BASE = 45                # % velocidad en recta
KP = 38.0                    # % de corrección por unidad de error (error -1..1)
KD = 3.2                     # % por (unidad de error / s)
KI = 0.0
T_PERDIDA_MS = 450           # sin línea: sigue derecho con la brújula este tiempo (zonas de color)
T_BUSCAR_MS = 3000           # luego gira buscando la línea hasta este tiempo

# ------------------------------------------------------------------ RUMBO SIN BRÚJULA (odometría)
# Si no hay brújula conectada, el rumbo se estima con el comando de los motores.
# 0° = hacia donde mira el robot al encender o al enviar "cero".
VEL_GIRO_CAL = 40            # % con el que gira en el sitio (y con el que se calibra el giro)
ZONA_MUERTA = 22             # % de duty por debajo del cual el TT no se mueve
TROCHA_MM = 128.6            # distancia entre centros de las ruedas (se corrige con cal_giro)
TAU_MOTOR = 0.08             # s, inercia de los motores
AJUSTE_RECTO = 0.0           # % para ir recto: NEGATIVO frena la rueda izquierda, POSITIVO la derecha (ej. -4)

# ------------------------------------------------------------------ BRÚJULA (opcional)
DECLINACION = 0.0            # grados; 0 = norte magnético (suficiente para el robot)
BRUJULA_INVERTIR = False     # si al girar a la DERECHA el rumbo baja, ponlo en True
KP_RUMBO = 0.9               # % por grado al avanzar manteniendo rumbo
KP_GIRO = 0.8                # % por grado al girar en el sitio
VEL_GIRO_MAX = 40
TOL_GIRO = 4.0               # grados

# ------------------------------------------------------------------ OBSTÁCULOS
# Esquive en rectángulo guiado por la brújula: gira 90°, sale de lado, avanza hasta
# rebasar el obstáculo y vuelve en diagonal (45°) hasta encontrar la línea.
DIST_OBSTACULO = 18.0        # cm: empieza la maniobra
DIST_MAX = 80.0              # cm: alcance máximo que se mide
EVITAR_LADO = 1              # 1 = esquiva por la derecha, -1 = por la izquierda
EVITAR_LATERAL_MM = 220      # cuánto se aparta de la línea (radio del obstáculo + ~170 mm)
EVITAR_PASAR_MM = 520        # cuánto avanza paralelo para rebasarlo
VEL_MM_S = 230               # velocidad REAL del robot a VEL_BASE (mídela: mm recorridos en 1 s)
EVITAR_TIMEOUT_MS = 7000     # si no encuentra la línea al volver, pasa a BUSCAR
# Servo del ultrasonido (solo si PIN_SERVO no es None): apunta el HC hacia donde va la curva
SERVO_MAX_ANG = 60           # grados a cada lado
SERVO_LOOKAHEAD_MM = 380     # a qué distancia por la pista apunta
SERVO_INVERTIR = False       # si en curva a la derecha el servo mira a la izquierda, ponlo en True

# ------------------------------------------------------------------ MANUAL
VEL_MANUAL = 55
T_MANUAL_MS = 1200           # cuánto avanza/retrocede con un comando de voz o botón
JOY_TIMEOUT_MS = 350         # el control ESP-NOW debe enviar al menos cada 0,35 s

# ------------------------------------------------------------------ COLOR
COLORES = ("negro", "blanco", "rojo", "amarillo", "verde", "azul")
COLOR_LECTURAS_ESTABLES = 2  # lecturas iguales seguidas para aceptar un cambio de color
COLOR_DIST_MAX = 0.35        # si el color más parecido está más lejos que esto: "desconocido"
# Acción opcional al entrar a un color en modo autónomo: ("pausa", ms) o ("vel", %)
ACCIONES_COLOR = {
    # "rojo": ("pausa", 1500),
    # "amarillo": ("vel", 35),
}

# ------------------------------------------------------------------ GEOMETRÍA (para el rumbo deseado)
DIST_EJE_REGLETA_MM = 107.0  # del eje de las ruedas a la regleta
SEMIANCHO_REGLETA_MM = 33.3  # 3,5 x 9,525 mm

ARCHIVO_CALIB = "calib.json"
