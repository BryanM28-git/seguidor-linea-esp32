# voz.py — Convierte una frase en español en comandos para el robot
import re
import unicodedata

CARDINALES = {"norte": 0, "noreste": 45, "este": 90, "sureste": 135, "sur": 180,
              "suroeste": 225, "oeste": 270, "noroeste": 315}
NUMEROS = {"cero": 0, "uno": 1, "dos": 2, "tres": 3, "cuatro": 4, "cinco": 5, "diez": 10, "quince": 15,
           "veinte": 20, "treinta": 30, "cuarenta": 40, "cuarenta y cinco": 45, "cincuenta": 50,
           "sesenta": 60, "setenta": 70, "ochenta": 80, "noventa": 90, "cien": 100, "ciento ochenta": 180,
           "doscientos setenta": 270, "trescientos": 300}


def normalizar(t):
    t = unicodedata.normalize("NFD", (t or "").lower())
    t = "".join(c for c in t if unicodedata.category(c) != "Mn")
    for palabra in sorted(NUMEROS, key=len, reverse=True):
        t = re.sub(r"\b%s\b" % palabra, str(NUMEROS[palabra]), t)
    return re.sub(r"[^a-z0-9 ]", " ", t)


def interpretar(texto):
    """Devuelve el comando del robot o None. Ej.: 'gira a 90 grados' -> 'rumbo:90'."""
    t = normalizar(texto)
    num = re.search(r"\b(\d{1,3})\b", t)
    n = int(num.group(1)) if num else None
    tiene = lambda *ps: any(re.search(r"\b%s" % p, t) for p in ps)

    if t.strip() == "para" or tiene("parar", "para ya", "para el", "alto", "deten", "stop", "frena", "quieto"):
        return "parar"
    if tiene("automatic", "autonom", "sigue la linea", "seguidor"):
        return "auto"
    if tiene("manual"):
        return "manual"
    if tiene("velocidad") and n is not None:
        return "vel:%d" % min(n, 100)
    if tiene("calibra"):
        if tiene("giro"):
            return "cal_giro"
        if tiene("brujula", "compas"):
            return "cal_brujula"
        if tiene("linea", "regleta", "infrarroj"):
            return "cal_linea"
        for c in ("negro", "blanco", "rojo", "amarillo", "verde", "azul"):
            if tiene(c):
                return "cal_color:" + c
    if tiene("reinicia", "fija", "resetea") and tiene("rumbo"):     # "reinicia el rumbo" -> 0° = dirección actual
        return "cero"
    for nombre, ang in CARDINALES.items():
        if re.search(r"\b%s\b" % nombre, t) and tiene("orienta", "apunta", "mira", "rumbo", "gira", "ve al", "hacia"):
            return "rumbo:%d" % ang
    if n is not None and tiene("grado"):
        if tiene("derecha"):
            return "girar:%d" % n
        if tiene("izquierda"):
            return "girar:-%d" % n
        return "rumbo:%d" % (n % 360)
    if tiene("rumbo", "orienta") and n is not None:
        return "rumbo:%d" % (n % 360)
    if tiene("adelante", "arriba", "avanza", "sigue derecho", "recto"):
        return "adelante"
    if tiene("atras", "abajo", "retrocede", "reversa"):
        return "atras"
    if tiene("derecha"):
        return "derecha"
    if tiene("izquierda"):
        return "izquierda"
    return None
