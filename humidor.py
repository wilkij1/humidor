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
import json
from machine import ADC, I2C, mem32, Pin, PWM, unique_id
from math import log
from micropython import const
from mqtt_as import MQTTClient, config as mqtt_config
# from mqtt_messages import MQTTMessages
from pid import PID
import re
import sys
import time
from ubinascii import hexlify

gc.collect()

# ==========
# Tunable parameters
# ==========

# task sleep durations following update run
HUMIDOR_UPDATE_S = const(1.)
NETWORK_SLEEP_MS = const(500)
REPORT_INTERVAL_S = const(10.)
AVG_LENGTH = const(6*10) # 10 minutes
MQTT_UPDATE_S = const(2.)
MQTT_UPDATE_OFFSET = const(0.200)

# gating interval for fan tachometer updates, in ms
FAN_TACH_GATE_MS = const(2_000)
# Lower heatsink temp limit where fan is running 100%
HS_MIN_FAN_RUN = const(45.)
# Heatsink temperature where heater output is limited
HS_SETPOINT = const(80.)
# Heatsink shutdown temperature
HS_SHUTDOWN = const(85.)

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

class MQTTMessages:
    
    identifiers = [hexlify(unique_id()).decode()]
    usfx = hexlify(unique_id()).decode()[-6:]
    
    messages = [
        ("homeassistant/climate/Humidor/config", {
            "name": "Humidor",
            "unique_id": "mode"+usfx,
            "availability": {
                "payload_available":"online",
                "payload_not_available":"offline",
                "topic":"humidor/available",
                },
            "modes": ["off", "auto", "fan_only"],
            "mode_state_topic": "humidor/state",
            "mode_state_template": "{{ value_json.mode }}",
            "mode_command_topic": "humidor/set/mode",
            "fan_modes": ["auto", "low", "medium", "high"],
            "fan_mode_command_topic": "humidor/set/fan_mode",
            "fan_mode_state_topic": "humidor/state",
            "fan_mode_state_template": "{{ value_json.fan_mode }}",
            "current_temperature_topic": "humidor/state",
            "current_temperature_template": "{{ value_json.inside_temperature }}",
            "current_humidity_topic": "humidor/state",
            "current_humidity_template": "{{ value_json.inside_humidity }}",
            "temperature_command_topic": "humidor/set/setpoint",
            "temperature_state_topic": "humidor/state",
            "temperature_state_template": "{{ value_json.setpoint }}",
            "temperature_unit": "C",
            "temp_step": 0.5,
            "initial": 4,
            "min_temp": -10,
            "max_temp": 50,

            "device": {
                "name": "Humidor",
                "ids": identifiers,
                "manufacturer": "Allegro Engineering",
                "model": "H01",
                "hw": "v1.0", # hw_version
                "sw": "v1.0", # sw_version
                "suggested_area": "porch",
                },
            }),
        ("homeassistant/sensor/HumidorHeatsinkT/config", {
            "device_class": "temperature",
            "name": "Heatsink Temperature",
            "state_topic": "humidor/state",
            "unit_of_measurement": "degC",
            "value_template": "{{ value_json.heatsink_temperature | round(1) }}",
            "unique_id": "heatsink_temp"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorOutsideT/config", {
            "device_class": "temperature",
            "name": "Outside Temperature",
            "state_topic": "humidor/state",
            "unit_of_measurement": "degC",
            "value_template": "{{ value_json.outside_temperature | round(1) }}",
            "unique_id": "outside_temp"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorOutsideH/config", {
            "device_class": "humidity",
            "name": "Outside Humidity",
            "state_topic": "humidor/state",
            "unit_of_measurement": "%",
            "value_template": "{{ value_json.outside_humidity | round(1) }}",
            "unique_id": "outside_humidity"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorFanPWM/config", {
            "name": "Fan PWM",
            "state_topic": "humidor/state",
            "um": "%",
            "value_template": "{{ value_json.fan_pwm | round(1) }}",
            "unique_id": "fan_pwm"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorHeatPWM/config", {
            "name": "Heat PWM",
            "state_topic": "humidor/state",
            "um": "%",
            "value_template": "{{ value_json.heat_pwm | round(1) }}",
            "unique_id": "heat_pwm"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorHeatUL/config", {
            "name": "Heat Upper Limit",
            "state_topic": "humidor/state",
            "um": "%",
            "value_template": "{{ value_json.heat_upper_limit_pwm | round(1) }}",
            "unique_id": "heat_upper_limit"+usfx,
            "device": { "ids": identifiers },
            }),
#         ("homeassistant/number/HumidorInsideTLoAlarm/config", {
#             "name": "Inside Temperature Low Alarm",
#             "state_topic": "humidor/state",
#             "state_template": "{{ value_json.inside_temperature_lo_sp }}",
#             "command_topic": "humidor/set/inside_temperature_lo_sp }}",
#             "device_class": "temperature",
#             "unique_id": "inside_temp"+usfx,
#             "dev": { "ids": identifiers },
#             "um": "degC",
#             "min": -10,
#             "max": 50,
#             }),
        ]
    
    def __init__(self):
        pass

    def discovery_messages(self):

        # translate the problematic degC values
        disc_messages = []
        for topic, message in self.messages:
            msg = json.dumps(message)
            msg = re.sub(r'"degC"', '"°C"', msg).encode('UTF-8')
            disc_messages.append((topic, msg))
        return disc_messages


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
    
    global tasks, mqm
        
    tasks = []
    
    # start the network interface task
    pass

    while True:
        (Tin, RHin, Tout, RHout,
         Ths, heat_pwm, fan_pwm) = humidor.read_status()

        logit(f"initial {Tin=}, {RHin=}, {Tout=}, {Ths=}")
        if Tin is not None:
            break
        
        await clock.sleep(1)
        
    # start the tachometer monitor task
    tach_mon = FanTachometer(humidor.fan_counter, FAN_TACH_GATE_MS)
    tasks.append(tach_mon.monitor_fan())
#     tach_mon_task = asyncio.create_task(tach_mon.monitor_fan())
#     tasks.append(tach_mon_task)
    
    # Humidor Climate Control
    controller = ClimateController(humidor, tach_mon)
    tasks.append(controller.control())
#     controller_task = asyncio.create_task(controller.control())
#     tasks.append(controller_task)

    consol_mon = ConsoleMonitor(humidor, tach_mon, controller)
    tasks.append(consol_mon.monitor())
    
    mqtt_mon = MQTTMonitor(humidor, controller)
    tasks.append(mqtt_mon.monitor())
    
    await asyncio.gather(*tasks)
    

class MQTTMonitor():
    
    def __init__(self, humidor, controller):
        self.humidor = humidor
        self.controller = controller
        self.unique_id = hexlify(unique_id())
        
        self.topics = {
            "set_setpoint": "humidor/set/setpoint",
            "available": "humidor/available",
            "ha_status": "homeassistant/status",
            "set_t_lo_sp": "homeassistant/set/inside_temperature_lo_sp",
            "set_t_hi_sp": "homeassistant/set/inside_temperature_hi_sp",
        }
        
        self.config = mqtt_config.copy()
        # config["server"] = "192.168.1.232"  # Change to suit
        self.config["server"] = "homeassistant.local"
        # WiFi credentials
        self.config["ssid"] = "wilk_s"
        self.config["wifi_pw"] = "tHesKylabiSfAlling"
        # MQTT broker credentials
        self.config["user"] = "mqtt_user"
        self.config["password"] = "mqtt_user"
        self.config["will"] = (self.topics['available'], "offline", False, 0)
        self.config["keepalive"] = 60
        self.config["queue_len"] = 1  # Use event interface with default queue
        
        self._mqm = MQTTMessages()
        
        self.client = MQTTClient(self.config)
        self.client.DEBUG = True
        self.outages = 0
        self._is_connected = False
        
        self.Tinside_alarm = False
        self.Tinside_alarm_lo_sp = -10
        self.Tinside_alarm_hi_sp = 50
        
    @property
    def is_connected(self):
        return self._is_connected
    
    async def monitor(self):
        # publish humidor status periodically
        try:
            await self.client.connect(quick=False)
        except OSError:
            print("Connection failed.")
            return
        
        for task in (self.up, self.down, self.messages):
            asyncio.create_task(task())

        self._is_connected = True
            
        n = 0
        next_data_update_time = MQTT_UPDATE_S
        current_time = clock.time()
        while next_data_update_time < current_time:
            next_data_update_time += MQTT_UPDATE_S
        await clock.sleep(next_data_update_time - current_time)
        
        while True:
            (Tset, _, _, heat_ul, _) = self.controller.read_status()
            (Tin, RHin, Tout, RHout,
             Ths, heat_pwm, fan_pwm) = self.humidor.read_status()
            
#             Tinside_lo_sp, Tinside_hi_sp, Tinside_alarm =\
#                 self.controller.get_setpoint("Tinside_lo_sp Tinside_hi_sp Tinside_alarm".split())
#             
#             Tinside_alarm = (Tinside_lo_sp < Tin < Tinside_hi_sp)

            msg = json.dumps({
                "mode": "auto",
                "fan_mode": "auto",
                "setpoint": Tset,
                "inside_temperature": Tin,
                "inside_humidity": RHin,
                "outside_temperature": Tout,
                "outside_humidity": RHout,
                "heatsink_temperature": Ths,
                "heat_pwm": heat_pwm,
                "heat_upper_limit": heat_ul,
                "fan_pwm": fan_pwm,
                "fan_rpm": 0.,
#                 "inside_temperature_alarm": "on" if self.Tinside_alarm else "off",
#                 "inside_humidity_alarm": "off",
#                 "inside_temperature_lo_sp": self.Tinside_alarm_lo_sp,
#                 "inside_temperature_hi_sp": self.Tinside_alarm_hi_sp,
                })
            
            # If WiFi is down the following will pause for the duration.
            await self.client.publish("humidor/state", msg, retain=True, qos=0)

            current_time = clock.time()
            while next_data_update_time < current_time:
                next_data_update_time += MQTT_UPDATE_S
            await clock.sleep(next_data_update_time - current_time + MQTT_UPDATE_OFFSET)
            
            n += 1
    
    async def messages(self):
        async for topic, msg, retained in self.client.queue:
            logit(f"rx: {topic.decode()}, {retained}, {msg.decode()=}")
            try:
                rx_msg = json.loads(msg.decode())
                logit(f"rx: {rx_msg=}")
            except ValueError:
                logit(f"ValueError while JSON parsing received message")
                continue
        
            dtopic = topic.decode()
            dmsg = json.loads(msg.decode())
            
#             if dtopic == self.topics["set_t_lo_sp"]:
#                 self.controller.set_setpoint("Tinside_lo_sp", float(dmsg))
#                 continue
#             if dtopic == self.topics["set_t_hi_hp"]:
#                 self.controller.set_setpoint("Tinside_hi_sp", float(dmsg))
#                 continue
            if dtopic == self.topics["set_setpoint"]:
                logit(f"setpoint to {dmsg}")
                continue
            if dtopic == self.topics["ha_status"] and dmsg == "online":
                await clock.sleep(0.25)	# just to provide HA some breathing room
                await self.client.publish(
                    self.topics['discovery'],
                    json.dumps(self.discovery_payload),
                    retain=True, qos=1)
                continue
            

    async def down(self):
        while True:
            await self.client.down.wait()  # Pause until connectivity changes
            self._is_connected = False
            print(f"{await self.client.wan_ok()=}")
            self.client.down.clear()
            # wifi_led(False)
            self.outages += 1
            logit("WiFi or MQTT broker is down.")

    async def up(self):
        while True:
            await self.client.up.wait()
            self._is_connected = True
            self.client.up.clear()
            # wifi_led(True)
                
            # publish the discovery messages
            logit("publishing HA configuration")
            for topic, message in self._mqm.discovery_messages():
                await self.client.publish(topic, message, retain=True, qos=0)
                
            await self.client.subscribe(self.topics['ha_status'], 1)
            await self.client.subscribe(self.topics["set_setpoint"], 1)
            await self.client.publish(self.topics['available'], "online", retain=True, qos=1)
   
class ConsoleMonitor:
    
    def __init__(self, humidor, tach_mon, controller):
        self.humidor = humidor
        self.tach_mon = tach_mon
        self.controller = controller
        
    async def monitor(self):
        
        next_report_time = 5.
        await clock.sleep(next_report_time)
        
        ctrl = self.controller
        
        while True:
            (Tin, RHin, Tout, RHout,
             Ths, heat_pwm, fan_pwm) = self.humidor.read_status()
            
            fan_rpm = await self.tach_mon.read_rpm()
            fan_rpm = 0 if fan_rpm is None else fan_rpm
            
            (temp_setpoint, act_heat_pwm, fan_pwm,
             cmd_upper_limit, last_pid_time) = ctrl.read_status()
            logit(f"Tset={temp_setpoint:6.2f} C, "
                  f"Heat={act_heat_pwm:5.1f}%, Fan={fan_pwm:5.1f}%, "
                  f"UL={cmd_upper_limit:5.1f}%")
            logit(f" Tin ={Tin:6.2f} C, RHin ={RHin:5.1f}%")
            logit(f" Tout={Tout:6.2f} C, RHout={RHout:5.1f}%")
            logit(f" Fan={fan_rpm:4.0f} RPM, Ths ={Ths:6.2f} C")
            logit(f" last PID update at {last_pid_time:8.3f}")
            
            # wait for the next reporting time, making sure we
            #  aren't so late as to have missed one
            current_time = clock.time()
            while next_report_time < current_time:
                next_report_time += REPORT_INTERVAL_S
            await clock.sleep(next_report_time - current_time)
            
class ClimateController:
    
    def __init__(self, humidor, tach_mon):
        self.humidor = humidor
        self.tach_mon = tach_mon
        self._act_heat_pwm = 0
        self._fan_pwm = 0
        self._cmd_upper_limit = 99.9
        self._last_update_time = None
        
        # load the control parameters file
        try:
            # regular operating parameters
            self.temp_setpoint = 21.5
            self.heat_limits = (0.1, 100.)
            self.Kp, self.Ki, self.Kd = (500., 0., 0.)
            self.last_cmd_pwm = 50.0

            # safety parameters controlling the max heatsink temperature
            self.hs_setpoint = HS_SETPOINT
            self.Kp_hs, self.Ki_hs, self.Kd_hs = (3., 0.5, 0.)

        except OSError:
            logit(f"cannot load parameters file, controller will not start")
                
        # start the PID controller task
        self.pid = PID(self.Kp, self.Ki, self.Kd,
                       setpoint=self.temp_setpoint,
                       output_limits=self.heat_limits,
                       auto_mode=True,
                       starting_output=self.last_cmd_pwm)

        self.heatsink_pid = PID(self.Kp_hs, self.Ki_hs, self.Kd_hs,
                                setpoint=self.hs_setpoint,
                                output_limits=(0.1, 99.9),
                                auto_mode=True,
                                starting_output=self.heat_limits[1])
            
    async def control(self):
        
        next_report_time = 0.
        next_update_time = HUMIDOR_UPDATE_S*5
        last_update_time = 0.
        
        current_time = clock.time()
        while next_update_time < current_time:
            next_update_time += HUMIDOR_UPDATE_S
        await clock.sleep(next_update_time - current_time)
        
        # current temperature
        temp, _, _ = self.humidor.update(self.last_cmd_pwm)
        if temp is None:
            temp = 0.
        logit(f"starting temp={temp:5.2f}")
        
        safety_trip = False
        
        while True:
            # check the time and calculate the delta from last update
            current_time = clock.time()
            dt = current_time - last_update_time
            
            # manage the heatsink temp by limiting the upper range for
            #  the heater
            Ths = self.humidor.read_heatsink()
            cmd_upper_limit = self.heatsink_pid(Ths, dt=dt)
            self._cmd_upper_limit = cmd_upper_limit
            
            # reduce the heater PID's upper limit to keep the heatsink
            #  below it's setpoint
            self.pid.output_limits = (0.1, cmd_upper_limit)
            
            # compute new output from the PID according to the
            #  current temperature
            Tin, RHin = self.humidor.read_indoor()
            cmd_heat_pwm = self.pid(Tin, dt=dt)
            
            # safety check - turn off heat completely at this
            #  value
            if (Ths > HS_SHUTDOWN):
                cmd_heat_pwm = 0
                safety_trip = True
            elif safety_trip and (Ths > HS_MIN_FAN_RUN):
                cmd_heat_pwm = 0
            else:
                safety_trip = False
                
            # feed the control output to the humidor
            #  and get the current temp and settings
            _, act_heat_pwm, fan_pwm = self.humidor.update(cmd_heat_pwm)
            self._act_heat_pwm = act_heat_pwm
            self._fan_pwm = fan_pwm
            
            last_update_time = current_time
            self._last_update_time = last_update_time
            
            # preemptively run the garbage collector to
            #  avoid long pauses at inopportune times
            gc.collect()
            
            current_time = clock.time()
            while next_update_time < current_time:
                next_update_time += HUMIDOR_UPDATE_S
            await clock.sleep(next_update_time - current_time)
     
    def read_status(self):
        return (self.temp_setpoint, self._act_heat_pwm,
                self._fan_pwm, self._cmd_upper_limit, self._last_update_time)

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
                                     address=BME280_I2CADDR+1,
                                     i2c=self._i2c0)
        except OSError:
            self.bme_inside = None
            
        try:
            self.bme_outside = BME280(mode=(1,1,1),
                                      address=BME280_I2CADDR,
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
        if heat_pwm >= 10.:
            self.led_heat.on()
        else:
            self.led_heat.off()
            
        Ths = self.read_heatsink()
        if Ths > HS_MIN_FAN_RUN:
            fan_pwm = 100.
        else:
            for heat_bkpt, fan_pwm in self._fan_breakpoints:
                if heat_pwm <= heat_bkpt:
                    break
                
        self.set_fan(fan_pwm)
        
        if fan_pwm >= 10.:
            self.led_fan.on()
        else:
            self.led_fan.off()

        temp, _ = self.read_indoor()
        
#         logit(f"{temp=}, {heat_pwm=}, {fan_pwm=}, {Ths=}")

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
        time.sleep_ms(100)
        self.setup(pin)

    def setup(self, pin):
        # Set pin to PWM
        mem32[IO_BANK0_BASE | (0x04 + pin * 8)] = 4
        # If using RP235x clear pad isolation and set input enable.
        if MCU == MCU_RP235X:
            mem32[0x40039004 + 0x04 * pin] = 0x140
        # Setup PWM counter for selected pin to chosen counter mode
        mem32[self._csr] = self._condition << 4
        time.sleep_ms(100)
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
