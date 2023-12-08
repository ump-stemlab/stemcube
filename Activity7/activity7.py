from machine import Pin, I2C        #importing relevant modules & classes
import network
import time
from math import sin
from umqtt.simple import MQTTClient
import bme280  #importing BME280 library

i2c=I2C(1,sda=Pin(2), scl=Pin(3), freq=400000)    #initializing the I2C method
bme = bme280.BME280(i2c=i2c)

# Fill in your WiFi network name (ssid) and password here:
wifi_ssid = ""
wifi_password = ""

# Connect to WiFi
wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(wifi_ssid, wifi_password)
while wlan.isconnected() == False:
    print('Waiting for connection...')
    time.sleep(1)
print("Connected to WiFi")

# Fill in your Adafruit IO Authentication and Feed MQTT Topic details
mqtt_host = "io.adafruit.com"
mqtt_username = ""  # Your Adafruit IO username
mqtt_password = ""  # Adafruit IO Key
temp_feed = ""  # The MQTT topic for your Adafruit IO Feed

# Enter a random ID for this MQTT Client
# It needs to be globally unique across all of Adafruit IO.
mqtt_client_id = ""

# Initialize our MQTTClient and connect to the MQTT server
mqtt_client = MQTTClient(
        client_id=mqtt_client_id,
        server=mqtt_host,
        user=mqtt_username,
        password=mqtt_password)

mqtt_client.connect()

# Publish a data point to the Adafruit IO MQTT server every 3 seconds
# Note: Adafruit IO has rate limits in place, every 3 seconds is frequent
#  enough to see data in realtime without exceeding the rate limit.

try:
    while True:
        # Generate some dummy data that changes every loop
        values = bme.values
        temp_check=values[0] #Check temperature reading
        #Pressure_check=values[1] #Check pressure reading
        #Humidity_check=values[2] #Check Humidity reading
        
        
        # Publish the data to the topic!
        print("Publish Temp:", temp_check)
        temp_message=str(temp) #Change temperature to string
        temp=temp_message.replace('C','') #Remove "C" Symbol
        mqtt_client.publish(temp_feed, temp) #Publish
        
        # Delay a bit to avoid hitting the rate limit
        time.sleep(3)
except Exception as e:
    print(f'Failed to publish message: {e}')
finally:
    mqtt_client.disconnect()