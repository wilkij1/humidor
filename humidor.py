
from bme280_float import (
    BME280,
    BME280_OSAMPLE_1,
    BME280_OSAMPLE_2,
    BME280_OSAMPLE_4,
    BME280_OSAMPLE_8,
    BME280_OSAMPLE_16,
    BME280_I2CADDR
    )
from machine import I2C, Pin
import time

i2c0 = None
i2c1 = None
bme0 = None
bme1 = None

def inhg2pa(p): return p * 3386.39
def pa2inhg(p): return p / 3386.39

# current barometer reading, corrected to sea level (QFF)
# barometer_hpa = inhg2hpa(29.71)
barometer_pa = inhg2pa(29.71)
barometer_pa = 97866.1
# barometer_pa = 101325

def main(barometer_pa):
    global bme0, bme1, i2c0, i2c1
    
    led_fan = Pin("GPIO17", Pin.OUT)
    led_fan.on()
    led_fire = Pin("GPIO22", Pin.OUT)
    led_fire.off()

    i2c0 = I2C(0, sda=20, scl=21, freq=400_000)
    i2c1 = I2C(1, sda=18, scl=19, freq=400_000)
    
    bmes = {}
    for ii, i2c in enumerate([i2c0, i2c1]):
        try:
            bme = BME280(mode=(1,1,1), address=BME280_I2CADDR, i2c=i2c)
            bme.sealevel = barometer_pa
            bmes[ii] = bme
        except OSError:
            pass
    print(f"initialized {len(bmes)} BME280s")
    print(f"QNH = {barometer_pa:6.0f}Pa, {pa2inhg(barometer_pa):5.2f}\"Hg")
    
    t0 = time.ticks_ms()
    while True:
        t1 = time.ticks_ms()
        for ii, bme in bmes.items():
            print(f"{time.ticks_diff(t1, t0)/1000:8.3f}: BME{ii}:")
            print(f"  {bme.values=}")
            temp, press, rh = bme.read_compensated_data()
            print(f"  '{temp:5.2f}C', '{press:7.2f}Pa', '{rh:5.2f}%'")
            altitude = bme.altitude
            sea_level = bme.sealevel
            dew_point  = bme.dew_point
            print(f"  altitude={altitude:6.2f}m, "
                  f"S.L.={sea_level:6.0f}Pa, D.P.={dew_point:5.2f}C")
        led_fan.toggle()
        led_fire.toggle()
        time.sleep_ms(950)
#         break
    
if __name__ == "__main__":
    main(barometer_pa)