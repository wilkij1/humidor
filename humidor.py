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
# from hardware import Clock, ConsoleMonitor, StatusMonitor
import json
from machine import ADC, I2C, mem32, Pin, PWM, unique_id
from math import log
from micropython import const
from mqtt_as import MQTTClient, config as mqtt_config
# from mqtt_messages import MQTTMessages
import os
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
REPORT_INTERVAL_S = const(30.)
REPORT_INTERVAL_OFFSET = const(0.250)
AVG_LENGTH = const(6*10) # 10 minutes
MQTT_UPDATE_S = const(2.)
MQTT_UPDATE_OFFSET = const(0.200)
CONFIG_SAVE_S = const(60.)

# gating interval for fan tachometer updates, in ms
FAN_TACH_GATE_MS = const(2_000)
# Lower heatsink temp limit where fan is running 100%
HS_MIN_FAN_RUN = const(55.)
# Heatsink temperature where heater output is limited
HS_SETPOINT = const(60.)
# Heatsink shutdown temperature
HS_SHUTDOWN = const(65.)
# trip point to turn on fan indicator
LED_FAN_LOW = const(10.)
# trip point to turn on heat indicator
LED_HEAT_LOW = const(10.)
# low fan speed (1% value is about 1/2 rotational speed)
FAN_LOW_PWM = const(12.)
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
# previous control param file renamed to this prior to writing 
#  update to FN_CONTROL
FN_CONTROL_BAK = "control.BAK.json"

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

stat_mon = None

def logit(message):
    if stat_mon:
        stat_mon.clock.logit(message)
    else:
        print(message)
    
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
            "mode_state_topic": "humidor/status",
            "mode_state_template": "{{ value_json.mode }}",
            "mode_command_topic": "humidor/set/mode",

            "fan_modes": ["auto", "low", "medium", "high"],
            "fan_mode_state_topic": "humidor/status",
            "fan_mode_state_template": "{{ value_json.fan_mode }}",
            "fan_mode_command_topic": "humidor/set/fan_mode",
            "fan_mode_state_template": "{{ value_json.fan_mode }}",
            
            "current_temperature_topic": "humidor/status",
            "current_temperature_template": "{{ value_json.in_temp }}",
            "current_humidity_topic": "humidor/status",
            "current_humidity_template": "{{ value_json.in_humid }}",
            
            "temperature_state_topic": "humidor/status",
            "temperature_state_template": "{{ value_json.setpoint }}",
            "temperature_command_topic": "humidor/set/setpoint",

            "temperature_unit": "C",
            "temp_step": 1.,
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
        ("homeassistant/sensor/HumidorInsideT/config", {
            "device_class": "temperature",
            "name": "T Inside",
            "state_topic": "humidor/status",
            "unit_of_measurement": "degC",
            "value_template": "{{ value_json.in_temp }}",
            "unique_id": "inside_temp"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorInsideH/config", {
            "device_class": "humidity",
            "name": "RH Inside",
            "state_topic": "humidor/status",
            "unit_of_measurement": "%",
            "value_template": "{{ value_json.in_humid }}",
            "unique_id": "inside_humidity"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorOutsideT/config", {
            "device_class": "temperature",
            "name": "T Outside",
            "state_topic": "humidor/status",
            "unit_of_measurement": "degC",
            "value_template": "{{ value_json.out_temp }}",
            "unique_id": "outside_temp"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorOutsideH/config", {
            "device_class": "humidity",
            "name": "RH Outside",
            "state_topic": "humidor/status",
            "unit_of_measurement": "%",
            "value_template": "{{ value_json.out_humid }}",
            "unique_id": "outside_humidity"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorHeatPWM/config", {
            "name": "Heat PWM",
            "state_topic": "humidor/status",
            "um": "%",
            "value_template": "{{ value_json.heat_pwm }}",
            "unique_id": "heat_pwm"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorHeatsinkT/config", {
            "device_class": "temperature",
            "name": "T Heatsink",
            "state_topic": "humidor/status",
            "unit_of_measurement": "degC",
            "value_template": "{{ value_json.heatsink_temp }}",
            "unique_id": "heatsink_temp"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorFanPWM/config", {
            "name": "Fan PWM",
            "state_topic": "humidor/status",
            "um": "%",
            "value_template": "{{ value_json.fan_pwm }}",
            "unique_id": "fan_pwm"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorFanRPM/config", {
            "name": "Fan RPM",
            "state_topic": "humidor/status",
            "value_template": "{{ value_json.fan_rpm }}",
            "unique_id": "fan_rpm"+usfx,
            "device": { "ids": identifiers },
            }),
        ("homeassistant/sensor/HumidorHeatUL/config", {
            "name": "Heat UL",
            "state_topic": "humidor/status",
            "um": "%",
            "value_template": "{{ value_json.heat_upper_limit_pwm }}",
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


def startup():

    global stat_mon
    
    stat_mon = StatusMonitor()

    clock = Clock(debug=True)
    stat_mon.register("clock", clock)
    
    try:
        asyncio.run(main(stat_mon))
        clock.logit("exiting from main()")
    except KeyboardInterrupt:
        clock.logit("KeyboardInterrupt intercepted and being re-raised")
        raise
    finally:
        clock.close()
            
    return humidor


async def main(stat_mon):
    
    global tasks
        
    tasks = []

    humidor = Humidor(stat_mon)
    
    # Humidor Climate Control
    controller = ClimateController(stat_mon)

    # local logging
    consol_mon = ConsoleMonitor(stat_mon, REPORT_INTERVAL_S, REPORT_INTERVAL_OFFSET)
  
    # MQTT I/O remote control communication
    mqtt_mon = MQTTMonitor(stat_mon)

    # start the async tasks
    tasks.append( asyncio.create_task(controller.control()) )
    tasks.append( asyncio.create_task(controller.maintain_configuration()) )
    tasks.append( asyncio.create_task(consol_mon.monitor()) )
    tasks.append( asyncio.create_task(mqtt_mon.monitor()) )
    
    await asyncio.gather(*tasks)
    

class MQTTMonitor():
    
    def __init__(self, stat_mon):
        self._sm = stat_mon
        self._sm.register("mqtt_monitor", self)
        
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
        self.client.DEBUG = False
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
        clock = self._sm.clock
        next_data_update_time = MQTT_UPDATE_S
        current_time = clock.time()
        while next_data_update_time < current_time:
            next_data_update_time += MQTT_UPDATE_S
        await clock.sleep(next_data_update_time - current_time)
        
        while True:
            await self.publish_state()
            
            current_time = clock.time()
            while next_data_update_time < current_time:
                next_data_update_time += MQTT_UPDATE_S
            await clock.sleep(next_data_update_time - current_time + MQTT_UPDATE_OFFSET)
            
            n += 1
    
    async def messages(self):
        clock = self._sm.clock
        controller = self._sm.controller
        
        async for topic, msg, retained in self.client.queue:
            clock.logit(f"{topic=}, {msg=}, {retained=}")
            dtopic = topic.decode()
            try:
                dmsg = json.loads(msg.decode())
                clock.logit(f" json: {dtopic=}, {dmsg=}")
            except ValueError:
                dmsg = msg.decode()
                clock.logit(f"!json: {dtopic=}, {dmsg=}")
                
            # try:
                # rx_msg = json.loads(msg.decode())
                # self._clock.logit(f"rx: {rx_msg=}")
            # except ValueError:
                # self._clock.logit(f"ValueError while JSON parsing received message")
                # continue

            if dtopic == "humidor/set/mode":
                controller.mode = dmsg
                await self.publish_state()
            elif dtopic == "humidor/set/fan_mode":
                controller.fan_mode = dmsg
                await self.publish_state()
            elif dtopic == "humidor/set/setpoint":
                controller.setpoint = float(dmsg)
                await self.publish_state()
            

    async def down(self):
        while True:
            await self.client.down.wait()  # Pause until connectivity changes
            self._is_connected = False
            self.client.down.clear()
            # wifi_led(False)
            self.outages += 1
            logit("WiFi or MQTT broker is down.")

    async def up(self):
        while True:
            await self.client.up.wait()
            self._is_connected = True
            self.client.up.clear()
                
            # publish the discovery messages
            logit("publishing HA configuration")
            for topic, message in self._mqm.discovery_messages():
                await self.client.publish(topic, message, retain=True, qos=0)
                
            await self.publish_state()
            
            # subscribe to interesting topics
            for topic in self.subscriptions:
                await self.client.subscribe(topic, 1)

            # announce availability
            await self.client.publish("humidor/available", "online", retain=True, qos=1)

    async def publish_state(self):
        
#             Tset = self._controller.setpoint
#             (Tin, RHin, Tout, RHout, Ths) = self._humidor.get_conditions()
#             heat_pwm, fan_pwm = self._humidor.get_heat(), self._humidor.get_fan()
            
#             Tinside_lo_sp, Tinside_hi_sp, Tinside_alarm =\
#                 self.controller.get_setpoint("Tinside_lo_sp Tinside_hi_sp Tinside_alarm".split())
#             
#             Tinside_alarm = (Tinside_lo_sp < Tin < Tinside_hi_sp)

            status = await self._sm.status(True)
            msg = json.dumps(status)
            
            # If WiFi is down the following will pause for the duration.
            await self.client.publish("humidor/status", msg, retain=False, qos=0)
            for kk in sorted(status.keys()):
                await self.client.publish(f"humidor/status/{kk}", json.dumps(status[kk]), retain=False, qos=0)

class ClimateController:

    defaults = dict(
        mode="off", fan_mode="auto", setpoint=25.,
        heat_upper_limit_pwm=100.,
        Kp=const(500.), Ki=const(0.), Kd=const(0.),
        hs_setpoint=const(HS_SETPOINT),
        Kp_hs=const(3.), Ki_hs=const(0.5), Kd_hs=const(0.),
    )
    
    def __init__(self, stat_mon):
        self._sm = stat_mon
        self._sm.register("controller", self)
        
        self._config_event = asyncio.Event()
        
        self._last_update_time = None
        
        # load the control parameters file, if possible...
        startup = self.load_configuration()
                    
        # unpack the operating parameters
        self._mode = startup["mode"]
        self._fan_mode = startup["fan_mode"]
        self._setpoint = startup["setpoint"]
        self._heat_upper_limit_pwm = startup["heat_upper_limit_pwm"]
        self._Kp, self._Ki, self._Kd = startup["Kp"], startup["Ki"], startup["Kd"]

        # safety parameters controlling the max heatsink temperature
        self._hs_setpoint = startup["hs_setpoint"]
        self._Kp_hs, self._Ki_hs, self._Kd_hs = startup["Kp_hs"], startup["Ki_hs"], startup["Kd_hs"]

        self._operational = True
        
        # start the PID controller task
        self.pid = PID(self._Kp, self._Ki, self._Kd,
                       setpoint=self._setpoint,
                       output_limits=(0., self._heat_upper_limit_pwm),
                       auto_mode=True,
                       starting_output=0.)

        self.heatsink_pid = PID(self._Kp_hs, self._Ki_hs, self._Kd_hs,
                                setpoint=self._hs_setpoint,
                                output_limits=(0., 100.),
                                auto_mode=True,
                                starting_output=self._heat_upper_limit_pwm)
        
        self.fan_pwms = dict(low=FAN_LOW_PWM, medium=FAN_MED_PWM, high=FAN_HIGH_PWM)
        
        self._operational = True

    def load_configuration(self):
        
        startup = None
        
        for fn in (FN_CONTROL, FN_CONTROL_BAK):
            try:
                with open(fn, "r") as fh:
                    startup = json.load(fh)
                logit(f"loaded configuration from '{fn}'")
                break
            except (OSError, ValueError):
                pass
                
        if startup and fn == FN_CONTROL_BAK:
            # good load from backup, shuffle the files
            os.remove(FN_CONTROL)
            os.rename(FN_CONTROL_BAK, FN_CONTROL)
            os.sync()
            
        if startup:
            return startup
        
        logit(f"fallback to default configuration parameters")
        return self.defaults
    
    def save_configuration(self):
        self._config_event.set()
        
    async def write_configuration(self):
        
        startup = await self._sm.status(True)
        
        # remove any existing backup file...
        try:
            os.remove(FN_CONTROL_BAK)
        except OSError:
            pass

        # rename the existing file, if any, as the backup (and sync)
        try:
            os.rename(FN_CONTROL, FN_CONTROL_BAK)
            os.sync()
        except OSError:
            logit(f"error creating backup configuration file, continuing...")
            pass
            
        try:
            with open(FN_CONTROL, "w") as fho:
                json.dump(startup, fho)
            os.sync()
        except OSError:
            self._cm.clock.logit(f"error saving configuration file to '{FN_CONTROL}'")
            pass
            
    async def maintain_configuration(self):
        
        next_save_time = CONFIG_SAVE_S
        clock = self._sm.clock
        
        while True:
            current_time = clock.time()
            while next_save_time < current_time:
                next_save_time += CONFIG_SAVE_S

            try:
                await asyncio.wait_for(self._config_event.wait(), timeout=next_save_time - current_time)
                self._config_event.clear()
            except asyncio.TimeoutError:
                pass
                
            await self.write_configuration()
            
    async def control(self):
        
        next_update_time = HUMIDOR_UPDATE_S
        last_update_time = 0.
        
        clock = self._sm.clock
        current_time = clock.time()
        while next_update_time < current_time:
            next_update_time += HUMIDOR_UPDATE_S
        await clock.sleep(next_update_time - current_time)
        
        # current temperature
        humidor = self._sm.humidor
        temp = humidor.get_inside_temp()
        clock.logit(f"starting temp={temp:5.2f}")
        
        safety_trip = False
        
        while True:
            # ==========
            # check the time and calculate the delta from last update,
            #  for the benefit of the PID controllers
            # ==========
            current_time = clock.time()
            dt = current_time - last_update_time
            
            # read the current heatsink temperature
            Ths = humidor.get_heatsink()
            
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
                # calculate new upper limit heat control limit, based on HS temperature
                self._heat_upper_limit_pwm = self.heatsink_pid(Ths, dt=dt)
                # set the heater PID's upper limit to keep the heatsink temp
                #  below it's max desired temperature
                self.pid.output_limits = (0., self._heat_upper_limit_pwm)
                
                # compute new output from the PID according to the
                #  current temperature
                Tin, RHin = humidor.get_inside()
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
                    
            elif self._mode == "fan_only":
                # run the fan at a commanded speed with the heat turned off
                cmd_heat_pwm = 0
                
            else:
                # turn everything off
                cmd_heat_pwm = 0
            
            # ==========
            # calculate the fan setting, based on mode, fan_mode, 
            #  heat commanded and HS temperature
            # ==========
            cmd_fan_pwm = self._compute_fan_pwm(cmd_heat_pwm, Ths)
 
            # ==========
            # update the commanded heat and fan settings
            # ==========
            humidor.set_heat(cmd_heat_pwm)
            humidor.set_fan(cmd_fan_pwm)
            
            # preemptively run the garbage collector to
            #  avoid long pauses at inopportune times
            gc.collect()
            
            # set the last update time to use for the next cycle
            # use current_time, rather than clock.time() since the GC pass might
            #  take substantial time and the delta t should reflect time between
            #  PID updates
            last_update_time = current_time
            self._last_update_time = last_update_time # record for health check purposes
            
            # calculate the sleep delay and wait for it to elapse
            current_time = clock.time() # now we want time from the real "present"
            while next_update_time < current_time:
                # make sure delay is >0, in case we are running late
                next_update_time += HUMIDOR_UPDATE_S
            await clock.sleep(next_update_time - current_time + HUMIDOR_UPDATE_OFFSET)
    
    def _compute_fan_pwm(self, cmd_heat_pwm, Ths):
        
        # hot heatsink always forces high speed fan
        min_fan_pwm = 0.
        
        if Ths > HS_MIN_FAN_RUN:
            cmd_fan_pwm = FAN_HIGH_PWM
        
        elif self.mode == "auto":
            # calculate the minimum fan speed, based on the heat command
            for heat_bkpt, min_fan_pwm in self._sm.humidor._fan_breakpoints:
                if cmd_heat_pwm <= heat_bkpt:
                    break
            
            if self.fan_mode == "auto":
                cmd_fan_pwm = min_fan_pwm
            else:
                fan_mode_pwm = self.fan_pwms.get(self.fan_mode, FAN_HIGH_PWM)
                cmd_fan_pwm = max(min_fan_pwm, fan_mode_pwm)
                
        elif self.mode == "fan_only":
            if self.fan_mode == "auto":
                cmd_fan_pwm = FAN_LOW_PWM
            else:
                cmd_fan_pwm = self.fan_pwms.get(self.fan_mode, FAN_HIGH_PWM)
        else: # mode is "off"
            cmd_fan_pwm = 0.

#         self._clock.logit(f"CFP: {(self.mode, self.fan_mode, min_fan_pwm, cmd_fan_pwm)=}")
        return cmd_fan_pwm
        
    @property
    def mode(self): return self._mode
    @mode.setter
    def mode(self, mode):
        if mode in "auto fan_only off".split():
            old_mode = self._mode
            self._mode = mode
            if mode != old_mode:
                self.save_configuration()
        else:
            raise ValueError(f"mode must be 'auto', 'fan_only' or 'off', received '{mode}'")
            
    @property
    def fan_mode(self): return self._fan_mode
    @fan_mode.setter
    def fan_mode(self, fan_mode):
        if fan_mode in "auto low medium high".split():
            old_fan_mode = self._fan_mode
            self._fan_mode = fan_mode
            if fan_mode != old_fan_mode:
                self.save_configuration()
        else:
            raise ValueError(f"fan mode must be 'auto', 'low', 'medium' or 'high', received '{fan_mode}'")
    
    @property
    def setpoint(self): return self._setpoint
    @setpoint.setter
    def setpoint(self, setpoint):
        if MIN_SETPOINT <= setpoint <= MAX_SETPOINT:
            old_setpoint = self._setpoint
            self._setpoint = setpoint
            self.pid.setpoint = setpoint
            if setpoint != old_setpoint:
                self.save_configuration()
        else:
            raise ValueError(f"setpoint must be in range [{MIN_SETPOINT}, {MAX_SETPOINT}], received {setpoint}")
            
    @property
    def hs_setpoint(self):
        return self._hs_setpoint
    @hs_setpoint.setter
    def hs_setpoint(self, hs_setpoint):
        if MIN_HS_SETPOINT <= hs_setpoint <= MAX_HS_SETPOINT:
            old_hs_setpoint = self._hs_setpoint
            self._hs_setpoint = hs_setpoint
            self.heatsink_pid.setpoint = hs_setpoint
            if hs_setpoint != old_hs_setpoint:
                self.save_configuration()
        else:
            raise ValueError(f"heatsink setpoint must be in range [{MIN_HS_SETPOINT}, {MAX_HS_SETPOINT}], received {hs_setpoint}")
            
    @property
    def heat_upper_limit_pwm(self):
        return self._heat_upper_limit_pwm
        
    def get_status(self):
        return (self._setpoint, self._heat_upper_limit_pwm, self._last_update_time)


class Humidor:
    
    def __init__(self, stat_mon):
        
        self._sm = stat_mon
        self._sm.register("humidor", self)
        
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
            (30, FAN_LOW_PWM),
            (60, FAN_MED_PWM),
            (100, FAN_HIGH_PWM),
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
        percent = max(0, min(100., percent))
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
        percent = max(0, min(100., percent))
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

class StatusMonitor:

    agents = "clock humidor controller mqtt_monitor console_monitor".split()
    
    def __init__(self):
        pass

    def register(self, name, value):
        if name in self.agents:
            setattr(self, name, value)
        else:
            raise ValueError(f"'{name}' is not a recognized agent")
            
    async def status(self, full=False):
        dd = {}
        if hasattr(self, "controller"):
            mode = self.controller.mode
            fan_mode = self.controller.fan_mode
            setpoint = self.controller.setpoint
            heat_upper_limit_pwm = self.controller.heat_upper_limit_pwm
        else:
            mode, fan_mode, setpoint, heat_upper_limit_pwm = None, None, "", ""
            
        if hasattr(self, "humidor"):
            humidor = self.humidor
            Tin, RHin = humidor.get_inside()
            Tout, RHout = humidor.get_outside()
            heat_pwm, fan_pwm = humidor.get_heat(), humidor.get_fan()
            Ths = humidor.get_heatsink()
            fan_rpm = await humidor.get_rpm()
        else:
            Tin, Tout, RHin, RHout, Ths = [""]*5
            heat_pwm, fan_pwm, fan_rpm = [""]*3
            
        dd = dict(
            mode=mode, fan_mode=fan_mode, setpoint=setpoint,
            in_temp=Tin, in_humid=RHin, out_temp=Tout, out_humid=RHout,
            heat_pwm=heat_pwm, fan_pwm=fan_pwm, fan_rpm=fan_rpm,
            heatsink_temp=Ths, heat_upper_limit_pwm=heat_upper_limit_pwm,
            )
        
        if full and hasattr(self, "controller"):
            # return internal state data, also
            ctrl = self.controller
            
            for kk in ClimateController.defaults:
                vv = getattr(ctrl, "_"+kk)
                dd[kk] = vv
                
        return dd
        
        
class ConsoleMonitor:
    
    def __init__(self, stat_mon, update_interval, update_offset):
        self._sm = stat_mon
        self._sm.register("console_monitor", self)
        
        self._update_interval = update_interval
        self._update_offset = update_offset
        
    async def monitor(self):
        
        clock = self._sm.clock
        humidor = self._sm.humidor
        ctrl = self._sm.controller
        
        next_report_time = 3. #self._update_interval
        current_time = clock.time()
        while next_report_time < current_time:
            next_report_time += 3 #self._update_interval
            
        dt = next_report_time - current_time + self._update_offset
        await clock.sleep(dt)
        
        while True:
            ss = await self._sm.status()
            
            keys = (
                "setpoint heat_pwm fan_pwm in_temp in_humid out_temp out_humid"
                " fan_rpm heatsink_temp").split()
            fmts = [
                "{:6.2f}C", "{:5.1f}%", "{:5.1f}%", "{:6.2f}C", "{:5.1f}%", "{:6.2f}C", "{:5.1f}%",
                "{:4.0f} RPM", "{:6.2f}C"
                ]
            for kk, fmt in zip(keys, fmts):
                val = ss[kk]
                if val is not None and val is not "":
#                     print(f"{kk=}, {fmt=}, {val=}")
                    ss[kk] = fmt.format(val)
                else:
                    ss[kk] = "?"
                
            clock.logit(f"Tset= {ss['setpoint']}, Heat={ss['heat_pwm']}, Fan={ss['fan_pwm']}, {ss['fan_rpm']}")
            clock.logit(f" Tin= {ss['in_temp']}, RHin= {ss['in_humid']}")
            clock.logit(f" Tout={ss['out_temp']}, RHout={ss['out_humid']}")
            clock.logit(f" Ths= {ss['heatsink_temp']}")
            
            # wait for the next reporting time, making sure we
            #  aren't so late as to have missed one
            current_time = clock.time()
            while next_report_time < current_time:
                next_report_time += self._update_interval
            await clock.sleep(next_report_time - current_time + self._update_offset)
            
    
if __name__ == "__main__":
    startup()
