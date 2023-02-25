import Adafruit_DHT
import json
import paho.mqtt.client as mqtt
import time
import RPi.GPIO as GPIO
import Adafruit_ADS1x15


# Define the ADS1115
adc = Adafruit_ADS1x15.ADS1115()

# Gain factor for the ADC
GAIN = 1

# Define the DHT11 sensor
DHT = Adafruit_DHT.DHT11

# Define the GPIO pin number
gpio_pin_DHT = 4
gpio_pin_light = 17
gpio_pin_Relay1 = 5
gpio_pin_Relay2 = 6
gpio_pin_Relay3 = 13
gpio_pin_Relay4 = 19

# Define the interval for sending the data (in seconds)
interval = 2

# Create the MQTT client
client = mqtt.Client()

mqttServer = "broker.hivemq.com"
mqttPort = 1883
mqttUsername = ""
mqttpassword = ""
mqttKeepAlive = 60

#MQTT topic
connecStatusTopic = "device/status"
dataTopic         = "device/sensor/data"
controlValveTopic1 = "device/control/relay1"
controlValveTopic2 = "device/control/relay2"
controlValveTopic3 = "device/control/relay3"
controlValveTopic4 = "device/control/relay4"
controlLEDTopic   = "device/control/LED"

calibration_curve = {
    0: 0,
    100: 200,
    500: 1000,
    1000: 2000,
    2000: 4000,
    3000: 6000,
    4000: 8000
}

def connect_mqtt(mqttServer, mqttPort, mqttUserName, mqttPassword, mqttServerKeepAlive):
    client.username_pw_set(mqttUserName, mqttPassword)
    client.connect(mqttServer, mqttPort, mqttServerKeepAlive)
    
def on_connect(client, userdata, flage, rc):
    print("Connected whit result code: " + str(rc))
    client.publish(connecStatusTopic, "connected")
    client.subscribe(controlValveTopic1);
    client.subscribe(controlValveTopic2);
    client.subscribe(controlValveTopic3);
    client.subscribe(controlValveTopic4);
    client.subscribe(controlLEDTopic);
        
def on_disconnect(client, userdata, rc):
    if rc != 0:
        print("reconnect MQTT...")
        client.reconnect()            

def on_message(client, userdata, msg):
    print("[" + msg.topic + "]: " + str(msg.payload.decode("utf-8")))
#     data = json.loads(msg.payload.decode("utf-8"))
    
    if msg.topic == "device/control/relay1":
        if str(msg.payload.decode("utf-8")) == "ON":
            GPIO.output(gpio_pin_Relay1, GPIO.LOW)
            print("Relay1 : ON")
        elif str(msg.payload.decode("utf-8")) == "OFF":
            GPIO.output(gpio_pin_Relay1, GPIO.HIGH)
            print("Relay1 : OFF")
            
    elif msg.topic == "device/control/relay2":
        if str(msg.payload.decode("utf-8")) == "ON":
            GPIO.output(gpio_pin_Relay2, GPIO.LOW)
            print("Relay2 : ON")
        elif str(msg.payload.decode("utf-8")) == "OFF":
            GPIO.output(gpio_pin_Relay2, GPIO.HIGH)
            print("Relay2 : OFF")
    
    elif msg.topic == "device/control/relay3":
        if str(msg.payload.decode("utf-8")) == "ON":
            GPIO.output(gpio_pin_Relay3, GPIO.LOW)
            print("Relay3 : ON")
        elif str(msg.payload.decode("utf-8")) == "OFF":
            GPIO.output(gpio_pin_Relay3, GPIO.HIGH)
            print("Relay3 : OFF")
            
    elif msg.topic == "device/control/relay4":
        if str(msg.payload.decode("utf-8")) == "ON":
            GPIO.output(gpio_pin_Relay4, GPIO.LOW)
            print("Relay4 : ON")
        elif str(msg.payload.decode("utf-8")) == "OFF":
            GPIO.output(gpio_pin_Relay4, GPIO.HIGH)
            print("Relay4 : OFF")
    
#     controlRelay(data)

def read_dh11_data(sensor, gpio_pin):
    # Read the humidity and temperature from The DHT11 sensor
    humidity, temperature = Adafruit_DHT.read_retry(sensor, gpio_pin)
    
    if humidity is not None and temperature is not None:
        return round(temperature, 2), humidity
    else:
        return None, None
    
def setup_pin():
    GPIO.setmode(GPIO.BCM)
#     GPIO.setup(gpio_pin_DHT, GPIO.IN)
    GPIO.setup(gpio_pin_Relay1, GPIO.OUT)
    GPIO.setup(gpio_pin_Relay2, GPIO.OUT)
    GPIO.setup(gpio_pin_Relay3, GPIO.OUT)
    GPIO.setup(gpio_pin_Relay4, GPIO.OUT)
    
    GPIO.output(gpio_pin_Relay1, GPIO.HIGH)
    GPIO.output(gpio_pin_Relay2, GPIO.HIGH)
    GPIO.output(gpio_pin_Relay3, GPIO.HIGH)
    GPIO.output(gpio_pin_Relay4, GPIO.HIGH)
    
def map_value(value, in_min, in_max, out_min, out_max):
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
    
def read_soil_moisture():
    
    adc.start_adc(0, gain=GAIN)
    value = adc.get_last_result()
    adc.stop_adc()
    
    value = map_value(value, 0, 32752, 100, 0)
    if(value < 0):
        return 0
    else:
        return value
        
#     return adc.read_adc_difference(0, gain=GAIN)

def read_light():
#     value = map_value(adc.read_adc_difference(1, gain=GAIN), 0, 32752, 100, 0)
    adc.start_adc(1, gain=GAIN)
    value = adc.get_last_result()
    adc.stop_adc()
    
    value = map_value(value, 0, 32752, 500, 0)
        
    if(value < 0):
        return 0
    else:
        return value

def controlRelay(data):
    
#     if data:
#         print (data)
    if data['relay1'] == 'ON':
        GPIO.output(gpio_pin_Relay1, GPIO.LOW)
        print("Relay1 : ON")
    elif data['relay1'] == 'OFF':
        GPIO.output(gpio_pin_Relay1, GPIO.HIGH)
        print("Relay1 : OFF")
    
    elif data['relay2'] == 'ON':
        GPIO.output(gpio_pin_Relay2, GPIO.LOW)
        print("Relay2 : ON")
    elif data['relay2'] == 'OFF':
        GPIO.output(gpio_pin_Relay2, GPIO.HIGH)
        print("Relay2 : OFF")
            
    elif data['relay3'] == 'ON':
        GPIO.output(gpio_pin_Relay3, GPIO.LOW)
        print("Relay3 : ON")
    elif data['relay3'] == 'OFF':
        GPIO.output(gpio_pin_Relay3, GPIO.HIGH)
        print("Relay3 : OFF")
            
    elif data['relay4'] == 'ON':
        GPIO.output(gpio_pin_Relay4, GPIO.LOW)
        print("Relay4 : ON")
    elif data['relay4'] == 'OFF':
        GPIO.output(gpio_pin_Relay4, GPIO.HIGH)
        print("Relay4 : OFF")
    
#     GPIO.cleanup()

def publish_data_to_mqtt():
    data = {}
    temperature, humidity = read_dh11_data(DHT, gpio_pin_DHT)
    data["temperature"] = temperature
    data["humidity"] = humidity
    data["soil moisture"] = read_soil_moisture()
    data["light"] = read_light()
    if GPIO.input(gpio_pin_Relay1):
        data["relay1"] = "OFF"
    else:
        data["relay1"] = "ON"
    
    if GPIO.input(gpio_pin_Relay2):
        data["relay2"] = "OFF"
    else:
        data["relay2"] = "ON"
        
    if GPIO.input(gpio_pin_Relay3):
        data["relay3"] = "OFF"
    else:
        data["relay3"] = "ON"
        
    if GPIO.input(gpio_pin_Relay4):
        data["relay4"] = "OFF"
    else:
        data["relay4"] = "ON"
    
    # Convert the dictionary to a JSON formatted string
    json_data = json.dumps(data)
    print(json_data)
    
    # Publish the JSON formatted data to the MQTT topic
    client.publish(dataTopic, json_data)

    # Wait for the specified interval
#     time.sleep(interval)

    
connect_mqtt(mqttServer, mqttPort, mqttUsername, mqttpassword, mqttKeepAlive)
client.on_connect = on_connect
client.on_message = on_message
client.on_disconnect = on_disconnect
setup_pin()

#f __name__ == '__main__':
while True:
    client.loop_start()
    publish_data_to_mqtt()
    