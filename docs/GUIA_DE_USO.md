# Robot seguidor de línea UMNG — firmware y app

## 1. Thonny: qué subir al ESP32
1. Instala MicroPython 1.22 o superior en el ESP32. En Thonny: *Herramientas → Opciones → Intérprete → MicroPython (ESP32) → Instalar o actualizar MicroPython*, variante **ESP32 / WROOM**.
2. Abre `firmware_esp32/config.py` y cambia `WIFI_SSID` y `WIFI_PASS` por el nombre y la clave del **hotspot de tu celular**.
3. Sube al ESP32 estos 9 archivos de la carpeta `firmware_esp32`. En el panel *Archivos* de Thonny, selecciónalos, haz clic derecho y elige *Subir a /*:
   `config.py`, `main.py`, `motores.py`, `regleta.py`, `color.py`, `odometria.py`, `brujula.py`, `ultrasonido.py` y `prueba_hardware.py`.
   Hay que subir `brujula.py` y `ultrasonido.py` aunque no los uses, porque `main.py` los importa.
4. **Primera prueba, sin WiFi:** abre `prueba_hardware.py` en Thonny y pulsa **F5**. Primero mueve cada motor y después muestra en vivo los 8 sensores de la regleta y las frecuencias del sensor de color. Termina con **Ctrl+C**.
5. **Programa completo:** pulsa el botón de reinicio (EN) del ESP32 o **Ctrl+D** en la consola de Thonny. `main.py` arranca solo e imprime la IP, por ejemplo: `WiFi conectado. Abre http://192.168.43.50`. Anota esa IP.

El ultrasonido está desactivado (`PIN_TRIG = None` en `config.py`). Para activarlo más adelante, pon `PIN_TRIG, PIN_ECHO = 2, 34`.

## 2. VS Code: la app Streamlit (en el PC) y el celular
1. Conecta el PC al **mismo hotspot** del celular.
2. En VS Code abre la carpeta `app_streamlit`. En la terminal (*Terminal → Nueva terminal*):
   ```
   pip install -r requirements.txt
   streamlit run app.py
   ```
   Si Windows pregunta por el firewall, permite el acceso en **redes privadas**.
3. En el PC se abre la app en `http://localhost:8501`. En la barra lateral escribe la IP del robot.
4. **Desde el celular:** en el PC ejecuta `ipconfig` y busca la "Dirección IPv4" del adaptador WiFi (por ejemplo 192.168.43.120). En el navegador del celular abre `http://192.168.43.120:8501`. La carpeta `.streamlit` ya trae la configuración para permitir ese acceso.
5. **Voz desde el celular:** toca el cuadro "Comando de voz", pulsa el **micrófono del teclado** (Gboard u otro), di la orden, por ejemplo "gira a la derecha", y toca *Enviar orden*.
   El botón "Hablar (micrófono del navegador)" funciona en el PC. En el celular, el navegador lo bloquea porque la página no es https. Para habilitarlo en Chrome Android, ve a `chrome://flags`, busca "Insecure origins treated as secure", agrega `http://IP-DEL-PC:8501` y reinicia Chrome.

## 3. Primera puesta en marcha (en este orden)
1. **Orden de la regleta:** abre la app y tapa con el dedo el sensor de la IZQUIERDA (mirando el robot desde atrás). Debe subir la barra S1; si sube S8, pon `INVERTIR_REGLETA = True` en `config.py`.
2. **Probar motores:** levanta el robot y pulsa *Probar motores*. Cada rueda gira primero hacia adelante y luego hacia atrás. Si alguna lo hace al revés, cambia `INVERTIR_IZQ` o `INVERTIR_DER`.
3. **Velocidad real:** en manual, manda *adelante*, mide cuántos mm avanza en 1 s a la velocidad base y escribe ese valor en `VEL_MM_S`. Si se desvía al ir recto, ajusta `AJUSTE_RECTO` (por ejemplo 2 o -2).
4. **Regleta:** pon la regleta sobre la línea y pulsa *1. Regleta*. El robot oscila 4 s; los 8 sensores tienen que ver la línea y el fondo.
5. **Giro (reemplaza a la brújula):** pon el robot **centrado sobre un tramo recto de la línea, mirando a lo largo de ella**, y pulsa *2. Giro*. Durante unos 15 s el robot gira en el sitio y cronometra cada vez que la regleta cruza la línea (pasa cada 180°), y con eso calcula sus grados por segundo. Después hace 3 medias vueltas de prueba y ajusta solo cuándo cortar los motores para no pasarse.
6. **Colores:** pon el TCS3200 sobre cada color de la pista y pulsa *Guardar este color*, uno por uno.
7. **Autónomo:** pon el robot sobre la línea, pulsa *Rumbo = 0°* y luego *Autónomo*. Para ajustar el PID, sube `Kp` hasta que oscile y luego sube `Kd` hasta que deje de oscilar.

Las calibraciones se guardan en `calib.json` dentro del ESP32. Repite la del giro si cambias de pilas o de piso, porque la velocidad de giro depende del voltaje y del agarre.

## 4. Comandos (HTTP `/cmd?c=...`, voz o ESP-NOW)
| Comando | Acción |
|---|---|
| `auto` / `manual` / `parar` | cambiar de modo o detener |
| `adelante` / `atras` | avanza o retrocede 1,2 s manteniendo el rumbo |
| `derecha` / `izquierda` | gira 90° (rumbo estimado, o brújula si hay) |
| `rumbo:90` · `girar:-45` | rumbo absoluto · giro relativo |
| `joy:x,y` | control proporcional (x, y de -1 a 1); el control ESP-NOW lo envía cada 0,3 s o menos |
| `vel:50` · `pid:38,3.2` · `lado:-1` | ajustes en caliente |
| `cal_linea` · `cal_giro` · `cal_color:rojo` · `cero` | calibraciones · `cero` toma la dirección actual como 0° |
| `prueba` | prueba de motores |

La telemetría completa está en `http://<ip>/estado` (JSON): rumbo actual y deseado, regleta, color, distancia, obstáculos, motores, batería y eventos.

## 5. Notas importantes
- **Regleta (8 sensor lines follow, tipo QTR-8RC):** VCC a **5 V** (pin VIN/5V del ESP32 o +5V del L298N; a 3V3 los LED infrarrojos quedan muy débiles y la regleta no cambia), GND, D1–D8 a GPIO 32, 33, 25, 26, 27, 14, 13 y 4. IR va a 3V3 o queda sin conectar (así los LED quedan encendidos).
- **Sin brújula:** el rumbo se estima con el comando de los motores (odometría). Los giros y el esquive usan cambios de rumbo cortos, que quedan bien calibrados. El rumbo absoluto de la app se va desviando con el tiempo; usa *Rumbo = 0°* al empezar cada recorrido.
- **Brújula opcional:** si más adelante conectas un GY-271 (SDA 21, SCL 22, VCC 3V3), el firmware la detecta solo y la usa en lugar de la estimación.
- **Tramos de color en la línea:** el infrarrojo casi no ve el rojo ni el amarillo. En esos tramos el robot repite el giro que traía durante `T_PERDIDA_MS`; si la pista tiene tramos de color largos en curva, sube ese valor.
- **Ultrasonido:** está desactivado. Sin él, el robot no detecta obstáculos. Si lo activas, recuerda que con el sensor fijo solo ve a tiempo los obstáculos en tramos rectos o curvas abiertas; para curvas cerradas va en un servo SG90 (`PIN_SERVO = 5`).
- **Batería de 9 V cuadrada:** sirve para probar con el robot levantado. En la pista, el voltaje se cae al arrancar los motores y el ESP32 se puede reiniciar; para correr usa 2×18650 o alimenta el ESP32 aparte con un power bank, uniendo los negativos.
