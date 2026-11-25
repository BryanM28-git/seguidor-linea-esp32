STA_IF, AP_IF = 0, 1
def hostname(*a): pass
class WLAN:
    def __init__(self, i): self.i = i
    def active(self, *a): return True
    def connect(self, *a): pass
    def isconnected(self): return True
    def ifconfig(self): return ("192.168.43.50", "", "", "")
    def config(self, **k): pass
    def disconnect(self): pass
