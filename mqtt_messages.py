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

import json
from machine import unique_id
import re
from ubinascii import hexlify

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
        ("homeassistant/number/HumidorInsideTLoAlarm/config", {
            "name": "Inside Temperature Low Alarm",
            "state_topic": "humidor/state",
            "value_template": "{{ value_json.inside_temperature_lo_alarm }}",
            "command_topic": "humidor/set/inside_temperature_lo_alarm }}",
            "device_class": "temperature",
            "unique_id": "inside_temp"+usfx,
            "dev": { "ids": identifiers },
            "um": "degC",
            "min": -10,
            "max": 50,
            }),
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
    
