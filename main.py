# MIT License
# 
# Copyright (c) 2025 Jeff Wilkinson
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
#

from humidor import startup
try:
    startup()
except Exception:
    pass

machine.reset()

===============================
#
# Start the network login, but don't wait around for the result
#

import machine
import json
import network
import sys
import time

SIGNON = "/signon.json"
CONNECT_LOG = "/connect.log"
BOOT_TIMEOUT = 8.000

# initialize 'nic' if this is power on reset, but
#  leave it alone on a soft-reset
if machine.reset_cause() == machine.PWRON_RESET:
    nic = network.WLAN(network.WLAN.IF_STA)

def logit(msg, mode='a', end="\n"):
    
    fh = None
    try:
        fh = open(CONNECT_LOG, mode)
        print(msg, file=fh, end=end)
    except Exception:
        pass
    finally:
        if fh:
            fh.close()
            
def main(blocking=False):
    global nic

    logit(f"{str(nic)=}")

    if nic.isconnected():
        logit(f"\n{nic.status()=}, already connected")
        return

    if nic.active() and nic.status() == network.STAT_CONNECTING:
        logit(f"{nic.status()=}, connecting in progress")
        return

    try:
        # Needs to connect
        # load credential data
        signon = json.load(open(SIGNON, 'r'))
        logit(f"==========\n"
              f"credentials for sign-on are {signon}", mode='w')

        # make a connection
        t0 = time.ticks_ms()
        nic.active(True)	# activate the NIC
        nic.connect(signon['ssid'], signon['password'])
        t1 = time.ticks_ms()
        dt1t0 = time.ticks_diff(t1, t0)/1000
        logit(f"{nic.status()=}, {dt1t0=:.3f} sec")
            
        if not blocking:
            logit(f"{nic.status()=}, non-blocking return")
            return
        
        while not nic.isconnected():
            t2 = time.ticks_ms()
            dt2t1 = time.ticks_diff(t2, t1)/1000
            if dt2t1 > BOOT_TIMEOUT:
                logit(f"{nic.status()=}, {dt1t0=:.3f}, {dt2t1=:.3f}")
                return
            time.sleep_ms(50)
            
        logit(f"{nic.status()=}, {dt1t0=:.3f},"
              f" {dt2t1=:.3f}, dt2t0={dt2t1+dt1t0:.3f}")
            
    except Exception as e:
        logit(f"Exception while establishing network connection: {str(e)}")

    #     for bb in sorted(nic.scan(), key=lambda bb:(bb[1],bb[0])):
    #         print(f"{str(bb[0]):16s}, {binascii.hexlify(bb[1])}, {str(bb[2:])}")

if __name__ == "__main__":
    main()
    

