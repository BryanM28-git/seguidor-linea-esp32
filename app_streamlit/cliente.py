# cliente.py — Comunicación HTTP con el robot
import requests

TIMEOUT = 1.2


def url(direccion, ruta):
    d = direccion.strip().rstrip("/")
    if not d.startswith("http"):
        d = "http://" + d
    return d + ruta


def estado(direccion):
    """Telemetría del robot como dict, o None si no responde."""
    try:
        r = requests.get(url(direccion, "/estado"), timeout=TIMEOUT)
        r.raise_for_status()
        return r.json()
    except (requests.RequestException, ValueError):
        return None


def comando(direccion, cmd):
    try:
        r = requests.get(url(direccion, "/cmd"), params={"c": cmd}, timeout=TIMEOUT)
        return r.json()
    except (requests.RequestException, ValueError) as e:
        return {"ok": False, "msg": "sin conexión con el robot (%s)" % e.__class__.__name__}
