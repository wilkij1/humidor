#
# Start the network login, but don't wait around for the result
#

import json
import network
import sys
import time

SIGNON = "/signon.json"
CONNECT_LOG = "/connect.log"
BOOT_TIMEOUT = 8.000

nic = None

def main(blocking=False):
    global nic

    fh = None

    # check to see if already connected to WiFi
    nic = network.WLAN(network.WLAN.IF_STA)
    
    if nic.isconnected():
        try:
            fh = open(CONNECT_LOG, 'a')
            print(f"\n{nic.status()=}, already connected",
                  file=fh)
        except Exception as e:
            if fh:
                sys.print_exception(e, file=fh)
        finally:
            if fh:
                fh.close()
        return

    if nic.active() and nic.status() == network.STAT_CONNECTING:
        try:
            fh = open(CONNECT_LOG, 'a')
            print(f"{nic.status()=}, connecting in progress",
                  file=fh)
        except Exception as e:
            if fh:
                sys.print_exception(e, file=fh)
        finally:
            if fh:
                fh.close()
        return

    try:
        try:
            fh = open(CONNECT_LOG, 'w')
        except OSError:
            pass

        # Needs to connect
        # load credential data
        signon = json.load(open(SIGNON, 'r'))
        if fh:
            print(f"==========\n"
                  f"credentials for sign-on are {signon}", file=fh)
            
        # make a connection
        t0 = time.ticks_ms()
        nic.active(True)	# activate the NIC
        nic.connect(signon['ssid'], signon['password'])
        t1 = time.ticks_ms()
        dt1t0 = time.ticks_diff(t1, t0)/1000
        if fh:
            print(f"{nic.status()=}, {dt1t0=:.3f} sec", file=fh)
            
        if not blocking:
            if fh:
                print(f"{nic.status()=}, non-blocking return", file=fh)
            return
        
        while not nic.isconnected():
            if fh:
                print(".", end="", file=fh)
            t2 = time.ticks_ms()
            dt2t1 = time.ticks_diff(t2, t1)/1000
            if dt2t1 > BOOT_TIMEOUT:
                if fh:
                    print(f"\n{nic.status()=}, {dt1t0=:.3f}, {dt2t1=:.3f}",
                          file=fh)
                return
            time.sleep_ms(100)
        if fh:
            print(f"\n{nic.status()=}, {dt1t0=:.3f},"
                  f" {dt2t1=:.3f}, dt2t0={dt2t1+dt1t0:.3f}",
                  file=fh)
            
    except Exception as e:
        if fh:
            print(f"Exception while establishing network connection: {str(e)}", file=fh)
            sys.print_exception(e, file=fh)
    finally:
        if fh:
            fh.flush()
            fh.close()
            
    #     for bb in sorted(nic.scan(), key=lambda bb:(bb[1],bb[0])):
    #         print(f"{str(bb[0]):16s}, {binascii.hexlify(bb[1])}, {str(bb[2:])}")

if __name__ == "__main__":
    main()
    
