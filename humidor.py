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

import asyncio
from bme280_float import (
    BME280,
    BME280_OSAMPLE_1,
    BME280_OSAMPLE_2,
    BME280_OSAMPLE_4,
    BME280_OSAMPLE_8,
    BME280_OSAMPLE_16,
    BME280_I2CADDR
    )
import gc
from machine import I2C, Pin, PWM
from micropython import const
from pid import PID
import sys
import time

# ==========
# Tunable parameters
# ==========

# task sleep durations following update run
HUMIDOR_SLEEP_MS = const(1_000)
NETWORK_SLEEP_MS = const(500)

# ==========
# Handy definitions
# ==========

# control parameters file. Parameters are stored in this
#  file and loaded on resets. Changes via the network interface
#  update these values so that the change persists when power is
#  cycled
FN_CONTROL = "control.json"

# logging file. Names are shuffled on every powerup and
#  oldest one deleted if more than 5
LOGGING_DIR = "/logs/"
LOGGING_FMT = "logging_{:d}.txt"
LOGGING_START = "logging_"
MAX_LOG_CNT = const(5)

# I/O definitions
PIN_SDA0 = const(20)
PIN_SCL0 = const(21)
PIN_SDA1 = const(18)
PIN_SCL1 = const(19)
PIN_PWM_HEAT = const(16)
PIN_PWM_FAN = const(17)
PIN_LED_HEAT = const(22)
PIN_LED_FAN = const(26)

# ==========
# G L O B A L S
# ==========
fh_log = None
fh_console = sys.stderr
t0_starting = 0
delta_t1t0 = 0

def startup(humidor_=None):
    
    global humidor, t0_starting
    
    t0_starting = time.ticks_ms()

    log_init()

    if humidor_ is None:
        humidor = Humidor()
        logit("Humidor initialized")
    else:
        humidor = humidor_
        logit("Humidor from main.py")

    try:
        asyncio.run(main())
        logit("exiting from main()")
    except KeyboardInterrupt:
        logit("KeyboardInterrupt intercepted and being re-raised")
        raise
    finally:
        if fh_log:
            fh_log.close()
            
    return humidor

async def main():
    
    global delta_t1t0
    
    tasks = []
    
    # start the network interface task
    pass

    # load the control parameters file
    try:
        temp_setpoint = 24.0
        out_limits = (0.1, 99.9)
        Kp, Ki, Kd = (30., 3., 0.)
        sample_time = HUMIDOR_SLEEP_MS
        starting_output = 50.0
        
    except OSError:
        logit(f"cannot load parameters file, controller will not start")
        
    # start the PID controller task
    pid = PID(Kp, Ki, Kd,
              setpoint=temp_setpoint,
              sample_time=sample_time,
              output_limits=out_limits,
              auto_mode=True,
              scale='ms')
    pid.set_auto_mode(True, last_output=starting_output)
    
    temp, _, _ = humidor.update(0)
    
    next_report = 0
    
    while True:
        t1 = time.ticks_ms()
        delta_t1t0 = time.ticks_diff(t1, t0_starting)
        
        # compute new output from the PID according to the
        #  system's current value
        control = pid(temp)
        
        # feed the output to the system and get its current value
        temp, heat_pwm, fan_pwm = humidor.update(control)

        if time.ticks_diff(delta_t1t0, next_report) >= 0:
            logit(f"T={temp:6.2f}C, Heat={heat_pwm:5.1f}%, Fan={fan_pwm:5.1f}%")
            next_report = time.ticks_add(next_report, 5_000)
            
        # preemptively run the garbage collector to
        #  avoid long pauses at inopportune times
        gc.collect()
        
        await asyncio.sleep_ms(HUMIDOR_SLEEP_MS)
        
    
class Humidor:
    
    def __init__(self):
        self.debug = True
    
        # I2C connections for BME280's
        self._i2c0 = I2C(0, sda=PIN_SDA0, scl=PIN_SCL0, freq=100_000)
        self._i2c1 = I2C(1, sda=PIN_SDA1, scl=PIN_SCL1, freq=100_000)
        
        # heat and fan indicators
        self.led_heat = Pin(PIN_LED_HEAT, Pin.OUT)
        self.led_heat.off()
        self.led_fan = Pin(PIN_LED_FAN, Pin.OUT)
        self.led_fan.off()
        
        # heat control
        self.pwm_heat = PWM(PIN_PWM_HEAT, freq=100, duty_u16=0)
        self.set_heat(0.1)
        
        # fan control
        self.pwm_fan = PWM(PIN_PWM_FAN, freq=25_000, duty_u16=10)
        self.set_fan(0.1)
        
        try:
            self.bme_inside = BME280(mode=(1,1,1),
                                     address=BME280_I2CADDR,
                                     i2c=self._i2c0)
        except OSError:
            self.bme_inside = None
            
        try:
            self.bme_outside = BME280(mode=(1,1,1),
                                      address=BME280_I2CADDR,
                                      i2c=self._i2c1)
        except OSError:
            self.bme_outside = None
            
        self._fan_breakpoints = [
            (0, 0),
            (10, 25),
            (50, 50),
            (100, 100),
            ]

    def read_indoor(self):
        
        if self.bme_inside:
            return self.read_compensated_data(self.bme_inside)
        else:
            return None, None
        
    def read_outdoor(self):
        if self.bme_outside:
            return self.read_compensated_data(self.bme_outside)
        else:
            return None, None

    def read_compensated_data(self, bme):
        """return temperature (degC) and humidity (%)"""
        
        # driver returns pressure, even though it is disabled
        t, _, h = bme.read_compensated_data()
        return t, h

    def set_fan(self, percent):
        self._set_pwm(self.pwm_fan, percent)
        
    def set_heat(self, percent):
        self._set_pwm(self.pwm_heat, percent)
        
    def _set_pwm(self, pwm, percent):
        percent = max(0.1, min(99.9, percent))
        duty_cycle = int(655.35*percent)
        pwm.duty_u16(duty_cycle)
        
    def get_control(self):
        
        heat_pwm = self.heat_pwm.duty_u16()/655.35
        fan_pwm = self.fan_pwm.duty_u16()/655.35
        
        return heat_pwm, fan_pwm
        
    def update(self, heat_pwm):
        """update the heat and fan settings"""
        
        self.set_heat(heat_pwm)
        if heat_pwm >= 1.:
            self.led_heat.on()
        else:
            self.led_heat.off()
            
        for heat_bkpt, fan_pwm in self._fan_breakpoints:
            if heat_pwm <= heat_bkpt:
                self.set_fan(fan_pwm)
                break
        if fan_pwm >= 1.:
            self.led_fan.on()
        else:
            self.led_fan.off()
            
        temp, _ = self.read_indoor()
        
        
        return temp, heat_pwm, fan_pwm

def log_init():
    global fh_log
    
    # ==========
    # setup a logging file (logit calls only go to console until this completes)
    # ==========
    try:
        fn_cnt = len([fn for fn in os.listdir(LOGGING_DIR) if fn.startswith(LOGGING_START)])
        logit(f"found {fn_cnt} existing log files")
        if fn_cnt >= MAX_LOG_CNT:
            # delete the oldest file
            fn = LOGGING_DIR+LOGGING_FMT.format(fn_cnt-1)
            logit(f"removing old log {fn}")
            os.remove(fn)
            fn_cnt -= 1
            
        for ii in range(fn_cnt):
            fn_src = LOGGING_DIR+LOGGING_FMT.format(fn_cnt - 1 - ii)
            fn_dst = LOGGING_DIR+LOGGING_FMT.format(fn_cnt - ii)
            logit(f"renaming {fn_src} -> {fn_dst}")
            os.rename(fn_src, fn_dst)

        fn = LOGGING_DIR+LOGGING_FMT.format(0)
        fh_log = open(fn, "w")
        logit(f"opened {fn} for logging")

    except Exception:
        pass   # keep going, even if logging setup doesn't work
    
def logit(msg):
    """print a log message to the log file and optionally to the console, when available"""
    
    delta_t1t0 = time.ticks_diff(time.ticks_ms(), t0_starting)
    msg = f"{delta_t1t0//1000:6d}.{delta_t1t0%1000:03d} {msg}"
    if fh_log is not None:
        if isinstance(fh_log, str):
            with open(file, "a") as fh:
                print(msg, file=fh)
        else:
            print(msg, file=fh_log)
            
    if humidor.debug:
        print(msg, file=fh_console)
        
    
if __name__ == "__main__":
    startup()
