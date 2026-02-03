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

from array import array
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
from machine import ADC, I2C, mem32, Pin, PWM
from math import log
from micropython import const
from pid import PID
import sys
import time

# ==========
# Tunable parameters
# ==========

# task sleep durations following update run
HUMIDOR_UPDATE_S = const(1.)
NETWORK_SLEEP_MS = const(500)
REPORT_INTERVAL_S = const(2.)
AVG_LENGTH = const(6*10) # 10 minutes

# gating interval for fan tachometer updates, in ms
FAN_TACH_GATE_MS = const(2_000)

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
PIN_PWM_HEAT = const(27)
PIN_PWM_FAN = const(17)
PIN_LED_HEAT = const(22)
PIN_LED_FAN = const(26)
PIN_FAN_TACH = const(19)

# ==========
# G L O B A L S
# ==========
fh_log = None
fh_console = sys.stderr
clock = None

def startup(humidor_=None):
    
    global humidor, clock
    
    clock = Clock()
    
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
    
    tasks = []
    
    # start the network interface task
    pass

    # start the tachometer monitor task
    tach_mon = FanTachometer(humidor.fan_counter, FAN_TACH_GATE_MS)
    tach_mon_task = asyncio.create_task(tach_mon.monitor_fan())
    tasks.append(tach_mon_task)
    
    # load the control parameters file
    try:
        temp_setpoint = 24.0
        out_limits = (0.1, 99.)
        Kp, Ki, Kd = (500., 0., 0.)
#         sample_time = HUMIDOR_SLEEP_MS
        last_cmd_pwm = 50.0
        
    except OSError:
        logit(f"cannot load parameters file, controller will not start")
        
    # start the PID controller task
    pid = PID(Kp, Ki, Kd,
              setpoint=temp_setpoint,
#               sample_time=sample_time,
              output_limits=out_limits,
              auto_mode=True,
              starting_output=last_cmd_pwm)
#     pid.set_auto_mode(True, last_output=starting_output)
    
    # current temperature
    temp, _, _ = humidor.update(last_cmd_pwm)
    logit(f"starting temp={temp:5.2f}")
    
    next_report_time = 0.
    next_update_time = HUMIDOR_UPDATE_S*5
    last_update_time = 0.
    await clock.sleep(HUMIDOR_UPDATE_S)
    
    while True:
        # check the time and calculate the delta from last update
        current_time = clock.time()
        dt = current_time - last_update_time
        
        # compute new output from the PID according to the
        #  current temperature
        Tin, RHin = humidor.read_indoor()
        cmd_heat_pwm = pid(Tin, dt=dt)
        
        # feed the control output to the humidor
        #  and get the current temp and settings
        _, act_heat_pwm, fan_pwm = humidor.update(cmd_heat_pwm)

        last_update_time = current_time
        
        if current_time >= next_report_time:

            (Tin, RHin, Tout, RHout,
             Ths, heat_pwm, fan_pwm) = humidor.read_status()
            
            fan_rpm = await tach_mon.read_rpm()
            fan_rpm = 0 if fan_rpm is None else fan_rpm
            
            logit(f"Tset={temp_setpoint:6.2f} C, "
                  f"Heat={act_heat_pwm:5.1f}%, Fan={fan_pwm:5.1f}%")
            logit(f" Tin ={Tin:6.2f} C, RHin ={RHin:5.1f}%")
            logit(f" Tout={Tout:6.2f} C, RHout={RHout:5.1f}%")
            logit(f" Fan={fan_rpm:4.0f} RPM, Ths ={Ths:6.2f} C")
            
            next_report_time += REPORT_INTERVAL_S
            
        # preemptively run the garbage collector to
        #  avoid long pauses at inopportune times
        gc.collect()
        
        current_time = clock.time()
        next_update_time = last_update_time + HUMIDOR_UPDATE_S
        while next_update_time < current_time:
            next_update_time += HUMIDOR_UPDATE_S
           
        await clock.sleep(next_update_time - current_time)
        

class Clock:
    def __init__(self):
        self._seconds = 0
        self._t0 = time.ticks_ms()
        
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
    

class Humidor:
    
    def __init__(self):
        self.debug = True
    
        # I2C connections for BME280's
        self._i2c0 = I2C(0, sda=PIN_SDA0, scl=PIN_SCL0, freq=100_000)
#         self._i2c1 = I2C(1, sda=PIN_SDA1, scl=PIN_SCL1, freq=100_000)
        
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
                                      address=BME280_I2CADDR+1,
                                      i2c=self._i2c0)
        except OSError:
            self.bme_outside = None
            
        self._heatsink = ADC(Pin(28))
        self._internal = ADC(4)
        
        self._fan_breakpoints = [
            (0, 0), # fan 0 PWM is about 1/4 speed (820 RPM)
            (1, 0),
            (10, 25),
            (50, 50),
            (100, 100),
            ]

        self.fan_counter = PWMCounter(PIN_FAN_TACH, PWMCounter.EDGE_RISING)
        
    def read_status(self):
        """return the inside temp, inside humidity, outside temp, outside
            humidity, heatsink temp, heat PWM, and fan PWM"""
        itemp, ihumidity = self.read_indoor()
        otemp, ohumidity = self.read_outdoor()
        hs_temp = self.read_heatsink()
        heat_pwm = self.get_heat()
        fan_pwm = self.get_fan()
#         logit(f"{heat_pwm=}, {fan_pwm=}")
        
        return (itemp, ihumidity, otemp, ohumidity,
                hs_temp, heat_pwm, fan_pwm)
    
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

    def read_heatsink(self):
        """thermistor on heat sink
        Read the temperature of the heating element to
        assure it stays in a safe range below 75C
        
        """
        def steinhart_hart(R, a, b, c):
            """Temperature model of a thermistor"""
            log_R = log(R)
            return 1/(a + b*log_R + c*log_R**3.)
        
        # calibration constants from fitted measurements over
        #  the range of 0 to 115C
        a, b, c = [ 6.85243040e+02, -6.00512143e+01,  1.80427980e-01]
        
        # divider is formed by R1 = 1000 on the high side of the
        #  thermistor (R2)
        r1 = 1000.
        adc = self._heatsink.read_u16()
        # clamp to a range of appx 0C (59577 counts) to 100C (16075)
        adc = min(59577, max(16075, adc))
        reading = adc * 3.3/65535
#         logit(f"{adc=}, {reading=}, {r1=}, {3.3 - reading=}")
        r2 = r1*reading/(3.3 - reading)
        
        recip_K = steinhart_hart(r2, a, b, c)
        degC = 1/recip_K - 273.15
        
        return degC
    
    def read_internal(self):
        """CPU temperature"""
        reading = self._internal.read_u16() * 3.3/65535
        T = 27 - (reading - 0.706) / 0.001721
        return T
    
    def read_compensated_data(self, bme):
        """return temperature (degC) and humidity (%)"""
        
        # driver returns pressure, even though it is disabled
        t, _, h = bme.read_compensated_data()
        return t, h

    def get_fan(self):
        return self._get_pwm(self.pwm_fan)
    
    def get_heat(self):
        return self._get_pwm(self.pwm_heat)
    
    def _get_pwm(self, pwm):
        duty_cycle = pwm.duty_u16()/65535
        return duty_cycle * 100
    
    def set_fan(self, percent):
        self._set_pwm(self.pwm_fan, percent)
        
    def set_heat(self, percent):
        self._set_pwm(self.pwm_heat, percent)
        
    def _set_pwm(self, pwm, percent):
        percent = max(0.1, min(99.9, percent))
        duty_cycle = int(655.35*percent)
        pwm.duty_u16(duty_cycle)
        
    def update(self, heat_pwm):
        """update the heat and fan settings"""
        
        self.set_heat(heat_pwm)
        if heat_pwm >= 1.:
            self.led_heat.on()
        else:
            self.led_heat.off()
            
        for heat_bkpt, fan_pwm in self._fan_breakpoints:
#             logit(f"{heat_pwm=}, {heat_bkpt=}, {fan_pwm=}")
            if heat_pwm <= heat_bkpt:
                self.set_fan(fan_pwm)
                break
        if fan_pwm >= 1.:
            self.led_fan.on()
        else:
            self.led_fan.off()
            
        temp, _ = self.read_indoor()
        
        return temp, heat_pwm, fan_pwm

#
# PWM counter control from
#  https://github.com/phoreglad/pico-MP-modules/tree/main/PWMCounter
#

MCU_RP235X = 1
IO_BANK0_BASE = 0x40028000
PWM_BASE = 0x400a8000
MAX_PINS = 48
MCU = MCU_RP235X
    
class PWMCounter:
    LEVEL_HIGH = 1
    EDGE_RISING = 2
    EDGE_FALLING = 3

    def __init__(self, pin, condition=LEVEL_HIGH):
        assert pin < MAX_PINS and pin % 2, "Invalid pin number"
        slice_offset = pin // 2 % 8 * 20 if pin < 32 else (pin // 2 % 4 + 8) * 20
        self._csr = PWM_BASE | (0x00 + slice_offset)
        self._ctr = PWM_BASE | (0x08 + slice_offset)
        self._div = PWM_BASE | (0x04 + slice_offset)
        self._condition = condition
#         time.sleep_ms(5)
        self.setup(pin)

    def setup(self, pin):
        # Set pin to PWM
        mem32[IO_BANK0_BASE | (0x04 + pin * 8)] = 4
        # If using RP235x clear pad isolation and set input enable.
        if MCU == MCU_RP235X:
            mem32[0x40039004 + 0x04 * pin] = 0x140
        # Setup PWM counter for selected pin to chosen counter mode
        mem32[self._csr] = self._condition << 4
        self.reset()

    def start(self):
        mem32[self._csr + 0x2000] = 1

    def stop(self):
        mem32[self._csr + 0x3000] = 1

    def reset(self):
        mem32[self._ctr] = 0

    def read(self):
        return mem32[self._ctr]

    def read_and_reset(self):
        tmp = self.read()
        self.reset()
        return tmp

    def set_div(self, int_=1, frac=0):
        if int_ == 256: int_ = 0
        mem32[self._div] = (int_ & 0xff) << 4 | frac & 0xf


class FanTachometer:
    
    def __init__(self, counter, gate):
        self.counter = counter
        # this call to setup() shouldn't be needed, but it makes soft resets
        #  work reliably. Otherwise, the PWM read returns 0's on every other start
        self.counter.setup(PIN_FAN_TACH)
        self.counter.set_div()
        self.counter.start()
        
        self._rpm = None
        self._gate = gate  # counting interval in ms
        self._lock = asyncio.Lock()
        
    async def read_rpm(self):
        
        async with self._lock:
            return self._rpm
        
    async def monitor_fan(self):
        
        try:
            while True:
                # reset the counter, wait for the counting period and read
                self.counter.reset()
                await asyncio.sleep_ms(self._gate)
                counts = self.counter.read()
                
                # tach outputs 2 pulses/revolution. Convert to RPM
                async with self._lock:
                    self._rpm = counts/(self._gate/1_000.) * 30
#                 logit(f"{counts=}, RPM={self._rpm:.0f}")
        except asyncio.CancelledError:
            self.counter.stop()
            
            
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
    
    t = clock.time()
    msg = f"{t:8.3f}: {msg}"
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
