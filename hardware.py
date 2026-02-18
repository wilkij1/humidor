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

CONSOLE_REPORT_UPDATE_S = const(5.)
CONSOLE_REPORT_UPDATE_OFFSET = const(0.250)

import asyncio
import os
import sys
import time

class Clock:
    def __init__(self, debug=False):
        self._seconds = 0
        self._t0 = time.ticks_ms()
        self._fh_log = None
        self._log_init()
        self._fh_console = sys.stderr
        self.debug = debug
        
    async def sleep(self, sec):
        delay_ms = int(sec*1000)
#         logit(f"{delay_ms=}")
        await asyncio.sleep_ms(delay_ms)
        
    def time(self):
        t1 = time.ticks_ms()
        dt = time.ticks_diff(t1, self._t0)
        self._seconds += dt/1000.
        self._t0 = t1
        
        return self._seconds
    
    def _log_init(self):
        
        # ==========
        # setup a logging file (logit calls only go to console until this completes)
        # ==========
        try:
            fn_cnt = len([fn for fn in os.listdir(LOGGING_DIR) if fn.startswith(LOGGING_START)])
            self.logit(f"found {fn_cnt} existing log files")
            if fn_cnt >= MAX_LOG_CNT:
                # delete the oldest file
                fn = LOGGING_DIR+LOGGING_FMT.format(fn_cnt-1)
                self.logit(f"removing old log {fn}")
                os.remove(fn)
                fn_cnt -= 1
                
            for ii in range(fn_cnt):
                fn_src = LOGGING_DIR+LOGGING_FMT.format(fn_cnt - 1 - ii)
                fn_dst = LOGGING_DIR+LOGGING_FMT.format(fn_cnt - ii)
                self.logit(f"renaming {fn_src} -> {fn_dst}")
                os.rename(fn_src, fn_dst)

            fn = LOGGING_DIR+LOGGING_FMT.format(0)
            self._fh_log = open(fn, "w")
            self.logit(f"opened {fn} for logging")

        except Exception:
            pass   # keep going, even if logging setup doesn't work
    
    def logit(self, msg):
        """print a log message to the log file and optionally to the console, when available"""
        
        t = self.time()
        msg = f"{t:8.3f}: {msg}"
        if self._fh_log is not None:
            if isinstance(self._fh_log, str):
                with open(file, "a") as fh:
                    print(msg, file=fh)
            else:
                print(msg, file=self._fh_log)
                
        if self.debug:
            print(msg, file=self._fh_console)
        
    def close(self):
        if self._fh_log is not None:
            self._fh_log.close()
            
class ConsoleMonitor:
    
    def __init__(self, clock, humidor, controller, update_interval, update_offset):
        self._humidor = humidor
        self._controller = controller
        self._update_interval = update_interval
        self._update_offset = update_offset
        self._clock = clock
        
    async def monitor(self):
        
        humidor = self._humidor
        ctrl = self._controller
        
        next_report_time = self._update_interval
        current_time = self._clock.time()
        while next_report_time < current_time:
            next_report_time += self._update_interval
        await self._clock.sleep(next_report_time - current_time + self._update_offset)
        
        ctrl = self._controller
        humidor = self._humidor
        
        while True:
            (Tin, RHin, Tout, RHout, Ths) = await humidor.get_conditions()
            
            (Tsetpoint, cmd_upper_limit, last_pid_time) = ctrl.get_status()
            (heat_pwm, fan_pwm) = humidor.get_heat(), humidor.get_fan()
            
            self._clock.logit(f"Tsetpoint={Tsetpoint:6.2f} C, "
                  f"Heat={heat_pwm:5.1f}%, Fan={fan_pwm:5.1f}%, "
                  f"UL={cmd_upper_limit:5.1f}%")
            self._clock.logit(f" Tin ={Tin:6.2f} C, RHin ={RHin:5.1f}%")
            self._clock.logit(f" Tout={Tout:6.2f} C, RHout={RHout:5.1f}%")
            self._clock.logit(f" Fan={fan_rpm:4.0f} RPM, Ths ={Ths:6.2f} C")
            self._clock.logit(f" last PID update at {last_pid_time:8.3f}")
            
            # wait for the next reporting time, making sure we
            #  aren't so late as to have missed one
            current_time = self._clock.time()
            while next_report_time < current_time:
                next_report_time += self._update_interval
            await self._clock.sleep(next_report_time - current_time + self._update_offset)
            
