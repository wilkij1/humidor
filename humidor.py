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

# TODO: humidor.DEBUG flag added, console monitor check it for outputs?
# TODO: Clock attribute added to humidor
# TODO: Move fan RPM monitor into humidor, start task in humidor.__init__()
# TODO: Fix fan RPM monitor
# TODO: straighten out MQTT topics and discovery
# TODO: rewrite MQTT monitoring and publishing
# TODO: move .update() from humidor to controller
# TODO: modify controller to support mode and fan modes
# TODO: initialize communication, even if Humidor() init fails

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
from hardware import Clock, ConsoleMonitor
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
HUMIDOR_UPDATE_OFFSET = const(0.)
NETWORK_SLEEP_MS = const(500)
REPORT_INTERVAL_S = const(10.)
REPORT_INTERVAL_OFFSET = const(0.250)
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
# trip point to turn on fan indicator
LED_FAN_LOW = const(10.)
# trip point to turn on heat indicator
LED_HEAT_LOW = const(10.)
# low fan speed (1% value is about 1/2 rotational speed)
FAN_LOW_PWM = const(1.)
# medium fan speed
FAN_MED_PWM = const(50.)
# high fan speed
FAN_HIGH_PWM = const(100.)
# minimum and maximum temperature setpoints
MIN_SETPOINT = const(-10.)
MAX_SETPOINT = const(50)

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
PIN_PWM_HEAT = const(22)
PIN_PWM_FAN = const(17)
PIN_LED_HEAT = const(10)
PIN_LED_FAN = const(11)
PIN_FAN_EN = const(12)
PIN_FAN_TACH = const(19)

# Error return values
TEMP_ERROR = const(-99.99)
HUMID_ERROR = const(0.)

# ==========
# G L O B A L S
# ==========
clock = Clock(debug=True)

def logit(message):
    clock.logit(message)
    
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
            "mode_state_topic": "humidor/state/mode",
            "mode_command_topic": "humidor/set/mode",
#             "mode_state_template": "{{ value_json.mode }}",
            "fan_modes": ["auto", "low", "medium", "high"],
            "fan_mode_state_topic": "humidor/state/fan_mode",
            "fan_mode_command_topic": "humidor/set/fan_mode",
#             "fan_mode_state_template": "{{ value_json.fan_mode }}",
            "current_temperature_topic": "humidor/current",
            "current_temperature_template": "{{ value_json.in_temp }}",
            "current_humidity_topic": "humidor/current",
            "current_humidity_template": "{{ value_json.in_humid }}",
            "temperature_command_topic": "humidor/set/setpoint",
            "temperature_state_topic": "humidor/state/setpoint",
#             "temperature_state_template": "{{ value_json.setpoint }}",
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
            "state_topic": "humidor/state/current",
            "unit_of_measurement": "degC",
            "value_template": "{{ value_json.hs_temp | round(1) }}",
            "unique_id": "heatsink_temp"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorOutsideT/config", {
            "device_class": "temperature",
            "name": "Outside Temperature",
            "state_topic": "humidor/state/current",
            "unit_of_measurement": "degC",
            "value_template": "{{ value_json.out_temp | round(1) }}",
            "unique_id": "outside_temp"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorOutsideH/config", {
            "device_class": "humidity",
            "name": "Outside Humidity",
            "state_topic": "humidor/state/current",
            "unit_of_measurement": "%",
            "value_template": "{{ value_json.out_humid | round(1) }}",
            "unique_id": "outside_humidity"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorFanPWM/config", {
            "name": "Fan PWM",
            "state_topic": "humidor/state/current",
            "um": "%",
            "value_template": "{{ value_json.fan_pwm | round(1) }}",
            "unique_id": "fan_pwm"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorHeatPWM/config", {
            "name": "Heat PWM",
            "state_topic": "humidor/state/current",
            "um": "%",
            "value_template": "{{ value_json.heat_pwm | round(1) }}",
            "unique_id": "heat_pwm"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorHeatUL/config", {
            "name": "Heat Upper Limit",
            "state_topic": "humidor/state/current",
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

        # translate the problematic degC values that get
        #  corrupted when run through json.dumps()
        disc_messages = []
        for topic, message in self.messages:
            msg = json.dumps(message)
            msg = re.sub(r'"degC"', '"°C"', msg).encode('UTF-8')
            disc_messages.append((topic, msg))
        return disc_messages


def startup(humidor_=None):
    
    if humidor_ is None:
        humidor = Humidor(clock)
        clock.logit("Humidor initialized")
    else:
        humidor = humidor_
        clock.logit("Humidor from main.py")

    # record subsystem status
    clock.logit(f"{humidor._i2c0=},"
                f" {humidor.bme_inside=}, {humidor.bme_outside=}")
    clock.logit(f"{humidor._operational=}")
    
    try:
        asyncio.run(main(humidor))
        clock.logit("exiting from main()")
    except KeyboardInterrupt:
        clock.logit("KeyboardInterrupt intercepted and being re-raised")
        raise
    finally:
        clock.close()
            
    return humidor


async def main(humidor):
    
    global tasks
        
    tasks = []
    
    # start the network interface task
    pass

#     # start the tachometer monitor task
#     tach_mon = FanTachometer(humidor.fan_counter, FAN_TACH_GATE_MS)
#     tach_mon_task = asyncio.create_task(tach_mon.monitor_fan())
#     tasks.append(tach_mon_task)
    
    # Humidor Climate Control
    controller = ClimateController(clock, humidor)
    tasks.append(controller.control())
#     controller_task = asyncio.create_task(controller.control())
#     tasks.append(controller_task)

    # local logging
    consol_mon = ConsoleMonitor(clock, humidor, controller, REPORT_INTERVAL_S, REPORT_INTERVAL_OFFSET)
    tasks.append(consol_mon.monitor())
  
    # MQTT I/O remote control communication
    mqtt_mon = MQTTMonitor(clock, humidor, controller)
    tasks.append(mqtt_mon.monitor())
    
    await asyncio.gather(*tasks)
    

class MQTTMonitor():
    
    def __init__(self, clock, humidor, controller):
        self._clock = clock
        self._humidor = humidor
        self._controller = controller
        self.unique_id = hexlify(unique_id())
        
        self.subscriptions = [
            "homeassistant/status",
            "humidor/set/#",
            ]
        
        self.config = mqtt_config.copy()
        self.config["server"] = "homeassistant.local"
        # WiFi credentials
        self.config["ssid"] = "wilk_s"
        self.config["wifi_pw"] = "tHesKylabiSfAlling"
        # MQTT broker credentials
        self.config["user"] = "mqtt_user"
        self.config["password"] = "mqtt_user"
        self.config["will"] = ("humidor/available", "offline", True, 1)
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
        current_time = self._clock.time()
        while next_data_update_time < current_time:
            next_data_update_time += MQTT_UPDATE_S
        await self._clock.sleep(next_data_update_time - current_time)
        
        while True:
            Tset = self._controller.setpoint
            (Tin, RHin, Tout, RHout, Ths) = self._humidor.get_conditions()
            heat_pwm, fan_pwm = self._humidor.get_heat(), self._humidor.get_fan()
            
#             Tinside_lo_sp, Tinside_hi_sp, Tinside_alarm =\
#                 self.controller.get_setpoint("Tinside_lo_sp Tinside_hi_sp Tinside_alarm".split())
#             
#             Tinside_alarm = (Tinside_lo_sp < Tin < Tinside_hi_sp)

            msg = json.dumps({
                "mode": "auto",
                "fan_mode": "auto",
                "setpoint": Tset,
                "in_temp": Tin,
                "in_humid": RHin,
                "out_temp": Tout,
                "out_humid": RHout,
                "hs_temp": Ths,
                "heat_pwm": heat_pwm,
                # "heat_upper_limit": heat_ul,
                "fan_pwm": fan_pwm,
                "fan_rpm": 0.,
#                 "inside_temperature_alarm": "on" if self.Tinside_alarm else "off",
#                 "inside_humidity_alarm": "off",
#                 "inside_temperature_lo_sp": self.Tinside_alarm_lo_sp,
#                 "inside_temperature_hi_sp": self.Tinside_alarm_hi_sp,
                })
            
            # If WiFi is down the following will pause for the duration.
            await self.client.publish("humidor/current", msg, retain=True, qos=0)

            current_time = self._clock.time()
            while next_data_update_time < current_time:
                next_data_update_time += MQTT_UPDATE_S
            await self._clock.sleep(next_data_update_time - current_time + MQTT_UPDATE_OFFSET)
            
            n += 1
    
    async def messages(self):
        async for topic, msg, retained in self.client.queue:
            dtopic = topic.decode()
            try:
                dmsg = json.loads(msg.decode())
            except ValueError:
                dmsg = msg.decode()
                
            self._clock.logit(f"rx: {dtopic}, {retained}, {dmsg=}, {type(dmsg)=}")
            # try:
                # rx_msg = json.loads(msg.decode())
                # self._clock.logit(f"rx: {rx_msg=}")
            # except ValueError:
                # self._clock.logit(f"ValueError while JSON parsing received message")
                # continue

            if dtopic == "humidor/set/mode":
                self._controller.mode = dmsg
                await self.client.publish("humidor/state/mode", dmsg, True, 1)
            elif dtopic == "humidor/set/fan_mode":
                self._controller.fan_mode = dmsg
                await self.client.publish("humidor/state/mode", dmsg, True, 1)
            elif dtopic == "humidor/set/setpoint":
                self._controller.setpoint = float(dmsg)
                await self.client.publish("humidor/state/setpoint", str(dmsg), True, 1)
            # if dtopic == self.topics["ha_status"] and dmsg == "online":
                # await clock.sleep(0.25)	# just to provide HA some breathing room
                # await self.client.publish(
                    # self.topics['discovery'],
                    # json.dumps(self.discovery_payload),
                    # retain=True, qos=1)
                # continue
            

    async def down(self):
        while True:
            await self.client.down.wait()  # Pause until connectivity changes
            self._is_connected = False
            self.client.down.clear()
            # wifi_led(False)
            self.outages += 1
            self._clock.logit("WiFi or MQTT broker is down.")

    async def up(self):
        while True:
            await self.client.up.wait()
            self._is_connected = True
            self.client.up.clear()
            # wifi_led(True)
                
            # publish the discovery messages
            self._clock.logit("publishing HA configuration")
            for topic, message in self._mqm.discovery_messages():
                await self.client.publish(topic, message, retain=True, qos=0)
                
            await self.publish_state()
            
            # subscribe to interesting topics
            for topic in self.subscriptions:
                await self.client.subscribe(topic, 1)

            # announce availability
            await self.client.publish("humidor/available", "online", retain=True, qos=1)

    async def publish_state(self):
        
        state_messages = [
            ("humidor/state/mode", self._controller.mode),
            ("humidor/state/fan_mode", self._controller.fan_mode),
            ("humidor/state/setpoint", self._controller.setpoint),
            ]
            
        for topic, message in state_messages:
            await self.client.publish(topic, json.dumps(message), retain=True, qos=1)
   
class ClimateController:
    
    def __init__(self, clock, humidor):
        self._mode = "auto"
        self._fan_mode = "auto"
        self._clock = clock
        self._humidor = humidor
        self._cmd_upper_limit = 99.9
        self._last_update_time = None
        
        # load the control parameters file
        try:
            # regular operating parameters
            self._setpoint = 21.5
            self.heat_limits = (0.1, 100.)
            self.Kp, self.Ki, self.Kd = (500., 0., 0.)
            self.last_cmd_pwm = 50.0

            # safety parameters controlling the max heatsink temperature
            self._hs_setpoint = HS_SETPOINT
            self.Kp_hs, self.Ki_hs, self.Kd_hs = (3., 0.5, 0.)

        except OSError:
            logit(f"cannot load parameters file, controller will not start")
            self._operational = False
            
        # start the PID controller task
        self.pid = PID(self.Kp, self.Ki, self.Kd,
                       setpoint=self._setpoint,
                       output_limits=self.heat_limits,
                       auto_mode=True,
                       starting_output=self.last_cmd_pwm)

        self.heatsink_pid = PID(self.Kp_hs, self.Ki_hs, self.Kd_hs,
                                setpoint=self._hs_setpoint,
                                output_limits=(0.1, 99.9),
                                auto_mode=True,
                                starting_output=self.heat_limits[1])
        
        self.fan_pwms = dict(low=FAN_LOW_PWM, medium=FAN_MED_PWM, high=FAN_HIGH_PWM)
        
        self._operational = True

    async def control(self):
        
        next_update_time = HUMIDOR_UPDATE_S
        last_update_time = 0.
        
        current_time = self._clock.time()
        while next_update_time < current_time:
            next_update_time += HUMIDOR_UPDATE_S
        await self._clock.sleep(next_update_time - current_time)
        
        # current temperature
        temp = self._humidor.get_inside_temp()
        self._clock.logit(f"starting temp={temp:5.2f}")
        
        safety_trip = False
        
        while True:
            # ==========
            # check the time and calculate the delta from last update,
            #  for the benefit of the PID controllers
            # ==========
            current_time = self._clock.time()
            dt = current_time - last_update_time
            
            if self._mode == "auto":
                # ==========
                # check the heatsink temp and reduce the upper allowable
                #  heat PWM (in the PID controller limits) if it is getting
                #  too high
                # If the heatsink reaches the upper safety limit (above the
                #  PID's controlled limit) then set safety_trip to True and
                #  shut off power to the heater completely until it gets to
                #  the low safety limit
                # ==========
                # read heatsink temp and calculate a new upper limit
                Ths = self._humidor.get_heatsink()
                cmd_upper_limit = self.heatsink_pid(Ths, dt=dt)
                self._cmd_upper_limit = cmd_upper_limit
                # set the heater PID's upper limit to keep the heatsink temp
                #  below it's max desired temperature
                self.pid.output_limits = (0.1, cmd_upper_limit)
                
                # compute new output from the PID according to the
                #  current temperature
                Tin, RHin = self._humidor.get_inside()
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
                self._humidor.set_heat(cmd_heat_pwm)
                
                # ==========
                # set the fan speed according to the current fan mode
                # ==========
                if self._fan_mode == "auto":
                    # fan speed depends on the heat_pwm value
                    if Ths > HS_MIN_FAN_RUN:
                        fan_pwm = 100.
                    else:
                        for heat_bkpt, fan_pwm in self._humidor._fan_breakpoints:
                            if cmd_heat_pwm <= heat_bkpt:
                                break
                            
                    self._humidor.set_fan(fan_pwm)
                    
                else:
                    # hot heatsink will override a manual fan mode
                    if Ths > HS_MIN_FAN_RUN:
                        self._humidor.set_fan(FAN_HIGH_PWM)
                    else:
                        self._humidor.set_fan(self.fan_pwms.get(self.fan_mode, FAN_HIGH_PWM))
                    
            elif self._mode == "fan_only":
                # run the fan at a commanded speed with the heat turned off                    
                self._humidor.set_fan(self.fan_pwms.get(self.fan_mode, FAN_HIGH_PWM))
                self._humidor.set_heat(0)
                
            else:
                # turn everything off
                self._humidor.set_heat(0)
                self._humidor.set_fan(0)
                
            # preemptively run the garbage collector to
            #  avoid long pauses at inopportune times
            gc.collect()
            
            # set the last update time to use for the next cycle
            # use current_time, rather than clock.time() since the GC pass might
            #  take substantial time and the delta t should reflect time between
            #  PID updates
            last_update_time = current_time
            self._last_update_time = last_update_time # record for health check purposes
            
            upd_Tset = self._setpoint
            upd_Tin = self._humidor.get_inside_temp()
            upd_Tout = self._humidor.get_outside_temp()
            upd_heat = self._humidor.get_heat()
            upd_fan = self._humidor.get_fan()
            upd_Ths = self._humidor.get_heatsink()
            upd_rpm = await self._humidor.get_rpm()
            
            self._clock.logit(
                f"Tset={upd_Tset:5.1f}C, Tin={upd_Tin:5.1f}C,"
                f" Heat={upd_heat:5.1f}, Fan={upd_fan:5.1f}%, {upd_rpm:4d} RPM,"
                f" Ths={upd_Ths:5.1f}C, Tout={upd_Tout:5.1f}C"
            )
            # calculate the sleep delay and wait for it to elapse
            current_time = self._clock.time() # now we want time from the real "present"
            while next_update_time < current_time:
                # make sure delay is >0, in case we are running late somehow
                next_update_time += HUMIDOR_UPDATE_S
            await self._clock.sleep(next_update_time - current_time + HUMIDOR_UPDATE_OFFSET)
    
    @property
    def mode(self): return self._mode
    @mode.setter
    def mode(self, mode):
        if mode in "auto fan_only off".split():
            self._mode = mode
        else:
            raise ValueError(f"mode must be 'auto', 'fan_only' or 'off', received '{mode}'")
            
    @property
    def fan_mode(self): return self._fan_mode
    @fan_mode.setter
    def fan_mode(self, fan_mode):
        if fan_mode in "auto low medium high".split():
            self._fan_mode = fan_mode
        else:
            raise ValueError(f"fan mode must be 'auto', 'low', 'medium' or 'high', received '{fan_mode}'")
    
    @property
    def setpoint(self): return self._setpoint
    @setpoint.setter
    def setpoint(self, setpoint):
        if MIN_SETPOINT <= setpoint <= MAX_SETPOINT:
            self._setpoint = setpoint
            self.pid.setpoint = setpoint
        else:
            raise ValueError(f"setpoint must be in range [{MIN_SETPOINT}, {MAX_SETPOINT}], received {setpoint}")
            
    @property
    def hs_setpoint(self):
        return self._hs_setpoint
    @hs_setpoint.setter
    def hs_setpoint(self, hs_setpoint):
        if MIN_HS_SETPOINT <= hs_setpoint <= MAX_HS_SETPOINT:
            self._hs_setpoint = hs_setpoint
            self.heatsink_pid.setpoint = hs_setpoint
        else:
            raise ValueError(f"heatsink setpoint must be in range [{MIN_HS_SETPOINT}, {MAX_HS_SETPOINT}], received {hs_setpoint}")
            
    def get_status(self):
        return (self._setpoint, self._cmd_upper_limit, self._last_update_time)


class Humidor:
    
    def __init__(self, clock):
        
        self._clock = clock
        self.debug = True
    
        # I2C connection for BME280's
        self._i2c0 = I2C(0, sda=PIN_SDA0, scl=PIN_SCL0, freq=100_000)
        
        # heat and fan indicators
        self.led_heat = Pin(PIN_LED_HEAT, Pin.OUT)
        self.led_heat.off()
        self.led_fan = Pin(PIN_LED_FAN, Pin.OUT)
        self.led_fan.off()
        
        # heat control
        self.pwm_heat = PWM(PIN_PWM_HEAT, freq=100, duty_u16=0)
        self.set_heat(0.1)
        
        # fan control
        self.fan_en = Pin(PIN_FAN_EN, Pin.OUT)
        self.pwm_fan = PWM(PIN_PWM_FAN, freq=25_000, duty_u16=10)
        self.set_fan(0)
        
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

        Pin(PIN_FAN_TACH, Pin.IN, Pin.PULL_UP)       
        self.fan_counter = PWMCounter(PIN_FAN_TACH, PWMCounter.EDGE_RISING)
        self.tachometer = FanTachometer(self.fan_counter, FAN_TACH_GATE_MS)
        self.tach_task = asyncio.create_task(self.tachometer.monitor_fan())

        self._operational = (self.bme_inside is not None and self.bme_outside is not None)
            
    def get_conditions(self):
        """return the inside temp, inside humidity, outside temp, outside
            humidity, heatsink temp"""
        itemp, ihumidity = self.get_inside()
        otemp, ohumidity = self.get_outside()
        hs_temp = self.get_heatsink()
        
        return (itemp, ihumidity, otemp, ohumidity, hs_temp)
    
    def get_status(self):
        """return heat PWM, fan PWM and fan RPM"""
        heat_pwm = self.get_heat()
        fan_pwm = self.get_fan()
        fan_rpm = 0.
        
        return heat_pwm, fan_pwm, fan_rpm
        
    def get_inside_temp(self):
        return self.get_inside()[0]
        
    def get_outside_temp(self):
        return self.get_outside()[0]
    
    def get_inside(self):
        try:
            return self.read_compensated_data(self.bme_inside)
        except AttributeError:
            return (TEMP_ERROR, HUMID_ERROR)
        
    def get_outside(self):
        try:
            return self.read_compensated_data(self.bme_outside)
        except AttributeError:
            return (TEMP_ERROR, HUMID_ERROR)
            
    def get_heatsink(self):
        """thermistor on heat sink
        Read the temperature of the heating element to
        assure it stays in a safe range below 75C
        
        @return: current heatsink temperature in degC
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
        r2 = r1*reading/(3.3 - reading)
        
        recip_K = steinhart_hart(r2, a, b, c)
        degC = 1/recip_K - 273.15

#         logit(f"{adc=}, {reading=}, {degC=}")
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
        percent = max(0, min(99.9, percent))
        if percent > 0:
            self.fan_en.on()
        else:
            self.fan_en.off()
        if percent >= LED_FAN_LOW:
            self.led_fan.on()
        else:
            self.led_fan.off()
        self._set_pwm(self.pwm_fan, percent)
        
    def set_heat(self, percent):
        if percent > LED_HEAT_LOW:
            self.led_heat.on()
        else:
            self.led_heat.off()

        self._set_pwm(self.pwm_heat, percent)
        
    def _set_pwm(self, pwm, percent):
        percent = max(0, min(99.9, percent))
        duty_cycle = int(655.35*percent)
        pwm.duty_u16(duty_cycle)
        
    def get_rpm(self):
        return await self.tachometer.read_rpm()
        
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
        
        self._rpm = 1
        self._gate = gate  # counting interval in ms
        self._lock = asyncio.Lock()
        
    async def read_rpm(self):
        
        async with self._lock:
            return 0 if self._rpm is None else self._rpm
        
    async def monitor_fan(self):
        
        try:
            while True:
                # reset the counter, wait for the counting period and read
                self.counter.reset()
                await asyncio.sleep_ms(self._gate)
                counts = self.counter.read()
                
                # tach outputs 2 pulses/revolution. Convert to RPM
                async with self._lock:
                    self._rpm = int(counts/(self._gate/1_000.)*30 + 0.5)

        except asyncio.CancelledError:
            self.counter.stop()
            
            
    
if __name__ == "__main__":
    startup()
