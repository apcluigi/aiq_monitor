

import streams
import math
import timers
import flash
import json
from wireless import wifi
from espressif.esp32net import esp32wifi as wifi_driver
from meas.htu21d import htu21d
from solomon.ssd1306 import ssd1306
import ubidots as iot
import sgp30

streams.serial()


def restore_baseline():
    """
    Restore the baseline saved in flash memory. 
    If the saved memory is not valid, the calibration phase begins
    """
    global baseline_valid
     # open a FlashFileStream at address 0x00310000
    ff  = flash.FlashFileStream ( 0x00310000 , 16 )
    
    flag = ff.read_int()
    eco2 = ff.read_int()
    tvoc = ff.read_int()
    
    print(flag)
    print(eco2)
    print(tvoc)
    
    ff.close()
    #if flag = 0 the baselin is saved correctly
    if flag == 0:
        baseline_valid = True
        sgp.set_iaq_baseline(eco2, tvoc)
    else:
        baseline_valid = False
        print("Warning - Baseline calibration required!")
        

def refreshBaseline():
    """
    Save the baseline in flash memory.
    """
    global baseline_valid
    global count_reset
    
    if baseline_valid:
        # open a FlashFileStream at address 0x00310000
        ff  = flash.FlashFileStream ( 0x00310000 , 16 )
    
        eco2_base = sample['eco2_base']
        tvoc_base = sample['tvoc_base']
        # write 8 byte -- 4 byte eco2_base + 4 byte tvoc_base
        ff.write(0)
        ff.write(eco2_base)
        ff.write(tvoc_base)
        ff.flush()
        ff.close()
        print('baseline saved')
    
    else:
        count_reset = count_reset + 1
        if count_reset == 11:
            baseline_valid = True

def readTemperatureHumidity():
    """
    Reads temperature and humidity from the htu21 sensor and saves them in the dictionary
    """
    t,h = htu.get_temp_humid()
    sample["temperature"] = t
    sample["humidity"] = h
    print('temperature = %.2f   humidity = %.2f' %(sample['temperature'],sample['humidity']))
    
def getAbsoluteHumidity():
    """
    Helper to calculate absolute humidity [g/m^3] with approximation formula based on temperature [°C] and humidity [%RH].
    
    Returns: 
        absolute humidity [g/m^3]
    """
    t = sample['temperature']
    rh = sample['humidity']
    #approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    absoluteHumidity = 216.7 * ((rh / 100.0) * 6.112 * math.exp((17.62 * t) / (243.12 + t))) / (273.15 + t) # [g/m^3]
    return absoluteHumidity
    
def setHumidityCompensation():
    """
    Set the absolute humidity on the SGP30 for more acurate readings of air quality signals (TVOC & eCO2)

    """
    absoluteHumidity = getAbsoluteHumidity()
    sgp.set_iaq_humidity(absoluteHumidity)
    print("Absolute Humidity = %.2f g/m^3" %(sample['absoluteHumidity']))
    
def readAirQuality():
    """
    Read the IAQ signals from SGP30 and store VOC and CO2 and saves them in the dictionary.
    
    """
    result = sgp.iaq_measure()
    sample['co2'] = result[0]
    sample['tvoc'] = result[1]
    print("CO2  = %.2f ppm  TVOC = %.2f ppb" %(result[0],result[1]))
    
def readBaseline():
    """
    Read the baseline from SGP30 and and saves them in the dictionary

    """
    result = sgp.get_iaq_baseline()
    sample['eco2_base'] = result[0]
    sample['tvoc_base'] = result[1] 
    print("eco2_base = %.2f    tvoc_base = %.2f " % (result[0],result[1]))
    
def pub_data():
    """
    publish the data in the ubidots platform
    """
    payload={"temperature": {"value":sample['temperature']},"humidity": {"value":sample['humidity']},"tvoc": {"value":sample['tvoc']},"co2": {"value":sample['co2']}}
    my_device.publish(payload)
    
def print_oled():
    ssd.draw_text(" co2  = %d  ppm    "%(sample['co2']),10,12,96,8, align=0)
    ssd.draw_text(" tvoc = %d  ppb   "%(sample['tvoc']),10,12*2,96,8, align=0)
    ssd.draw_text(" temp = %d  C"%(sample['temperature']),10,12*3,96,8, align=0)
    ssd.draw_text(" rh   = %d  %"%(sample['humidity']),10,12*4,96,8, align=0)

def manual_input_callback(data):
    """
    callback that is activated on the update of the manual_input variable
    """
    global baseline_valid
    
    print('request')
    
    if data['context']['manual_input'] == 'co2':
        payload={"co2": {"value":sample['co2']}}
        my_device.publish(payload)
    
    elif data['context']['manual_input'] == 'all':
        payload={"temperature": {"value":sample['temperature']},"humidity": {"value":sample['humidity']},"tvoc": {"value":sample['tvoc']},"co2": {"value":sample['co2']}}
        my_device.publish(payload)
    
    elif data['context']['manual_input'] == 'tvoc':
        payload={"tvoc": {"value":sample['tvoc']}}
        my_device.publish(payload)
        
    elif data['context']['manual_input'] == 'temperature':
        payload={"temperature": {"value":sample['temperature']}}
        my_device.publish(payload)
        
    elif data['context']['manual_input'] == 'humidity':
        payload={"humidity": {"value":sample['humidity']}}
        my_device.publish(payload)
    
    elif data['context']['manual_input'] == 'reset':
        print('device reset')
        baseline_valid = False
        sgp.iaq_init()
        ff  = flash.FlashFileStream ( 0x00310000 , 16 )
        ff.write(1)
        ff.flush()
        ff.close()
        
def check_air():
    if sample['co2'] < 800 and sample['tvoc'] < 1000:
        return 'GOOD'
    elif sample['co2'] < 1500 and sample['tvoc'] < 2000:
        return 'MODERATE'
    else:
        return 'BAD'

def change_color():
    """
    manages the LEDs
    """
    global air_quality_code
    global air_quality_previous
    try:
        if air_quality_previous is not None:
            #turns off the LED which is on
            digitalWrite(air_status_led[air_quality_previous], states['OFF'])
        digitalWrite(air_status_led[air_quality_code], states['ON'])
    except Exception as e:
        print("change_color",e)
        
def toogle_oled():
    """
    manages the switching of the display
    """
    global status_oled
    if status_oled == 'ON':
        print('OLED OFF')
        status_oled = 'OFF'
        ssd.off()
    elif status_oled == 'OFF':
        print('OLED ON')
        status_oled = 'ON'
        ssd.on()

def init_wifi():
    wifi_driver.auto_init()
    for _ in range(5):
        try:
            print("connect wifi")
            wifi.link("your_wifi_id", wifi.WIFI_WPA2, "your_wifi_pass")
            print("connect wifi done")
            break
        except Exception as e:
            print("wifi connect err", e)

def init_ubidots():
    """
    connect to mqtt broker, set variable update callback and start mqtt reception loop
    """
    try:
        print('connecting to mqtt broker...')
        my_device.mqtt.connect()
    
        my_device.on_variable_update(ubidots_conf['device_label'], 'manual_input', manual_input_callback, json=True)
        my_device.mqtt.loop()
    except Exception as e:
        print("Ubidots init err", e)

def init_htu():
    try:
        print("start htu21d")
        htu.start()
        print('init htu21d..')
        htu.init()
    except Exception as e:
        print("htu init err", e)

def init_sgp():
    try:
        print("start sgp30")
        sgp.start()
        print('init sgp30..')
        sgp.iaq_init()
        #sgp.set_iaq_baseline(0x9c01, 0x9b75)
        restore_baseline()
    except Exception as e:
        print("sgp init err", e)
        
def init_ssd():
    try:
        print('init ssd1306')
        ssd.init(128,64)
        ssd.on()
        ssd.draw_text("      Indoor Air Quality      ",fill = False)
    except Exception as e:
        print("ssd init err", e)
        
    
def init():
    init_wifi()
    init_ubidots()
    init_htu()
    init_sgp()
    init_ssd()

def main():
    global air_quality_code
    global air_quality_previous
    
    try:
        while True:
        
            readTemperatureHumidity()

            setHumidityCompensation()
            readAirQuality()
            readBaseline();
        
            air_quality_code = check_air()
            if air_quality_code != air_quality_previous:
                change_color()
                air_quality_previous = air_quality_code
            
            #every 2 minutes the data is published
            if tmp_publish.get() >= 120000:
                tmp_publish.reset()
                pub_data()
        
            #every 60 minutes the data is saved
            if tmp_save.get() >= 3600000:
                tmp_save.reset()
                refreshBaseline()
        
            #if the Oled is ON print the data
            if status_oled == 'ON':
                print_oled()
            
            
            sleep(2000)

    except Exception as e:
        print("main", e)




sample = {
    'temperature'      : None,
    'humidity'         : None,
    'tvoc'             : None,
    'co2'              : None,
    'eco2_base'        : None,
    'tvoc_base'        : None
}

air_status_led = {
    'GOOD'     : D18,       #GREEN
    'MODERATE' : D19,       #YELLOW
    'BAD'      : D21        #RED
}

states = {
    'ON'  : HIGH,
    'OFF' : LOW
}

ubidots_conf={
    "user_type"    : "business",
    "device_label" : "aiq_device",
    "api_token"    : "your_api_token"
}

baseline_valid = True
air_quality_previous = None
air_quality_code = None
status_oled = 'ON'
count_reset = 0

btn_pin = D2
# set the btn_pin as input with PullUp
pinMode(btn_pin,INPUT_PULLUP)
# Attach an interrupt on the button pin
onPinFall(btn_pin,toogle_oled)

# set the led pin as output 
for pin in air_status_led.values():
    pinMode(pin,OUTPUT)

# create sensor instance
htu = htu21d.HTU21D(I2C0,clk=100000)
sgp = sgp30.Adafruit_SGP30(I2C0)
ssd = ssd1306.SSD1306(I2C1)

# create ubidots iot device instance
my_device = iot.Device(ubidots_conf['device_label'], ubidots_conf['user_type'], ubidots_conf['api_token'])

# create a publish timer 
tmp_publish = timers.timer()
# create save timer
tmp_save = timers.timer()
    
tmp_publish.start()
tmp_save.start()

init()

main()






