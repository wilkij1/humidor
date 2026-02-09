import asyncio
from microdot import Microdot
import network
import socket
import time

from machine import Pin

led = Pin("LED", Pin.OUT)
led.off()

ssid = "wilk_s"
pw = "tHesKylabiSfAlling"

def connect_wlan(ssid, pw, verbose=False):
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    wlan.connect(ssid, pw)

    # wait for connect or fail
    max_wait = 10
    while max_wait > 0:
        if wlan.status() < 0 or wlan.status() >= 3:
            break
        max_wait -= 1
        if verbose:
            print(f"waiting {wlan.status()=}")
        time.sleep_ms(100)
        
    if wlan.status() != 3:
        raise RuntimeError(f"network connection failed with status == {wlan.status()}")
    else:
        ipaddr = wlan.ifconfig()[0]
        if verbose:
            print(f"connected on {ipaddr}")
            print(f"{wlan.ifconfig()=}")
    
    return wlan, ipaddr

wlan, ipaddr = connect_wlan(ssid, pw, verbose=True)

app = Microdot()

# app.before_request
# async def log_request(request):
#     print(f"BEFORE {request=}")
#     print(f"BEFORE {request.method=}, {request.url=}, {request.headers=}")

def report_request(request, verbose=False):
    attrs = ("method path args headers cookies content_type content_length"
             " json form files client_addr app sock route"
             " url url_prefix url_args http_version query_string"
             " scheme").split()
    verbose_attrs = ("args headers json"
                     " form files").split()
    
    print(f"{request.__dict__.keys()=}")
    for aa in attrs:
        if aa in verbose_attrs and not verbose:
            continue
        print(f"{aa}: {getattr(request, aa, 'UNKNOWN')}")
        
@app.route('/')
def index(request):
    report_request(request)
    return 'Hello, world!'

@app.route('/status')
def status(request):
    report_request(request, True)
    return f"Device IP: {wlan.ifconfig()[0]}"

app.run(port=5000)
