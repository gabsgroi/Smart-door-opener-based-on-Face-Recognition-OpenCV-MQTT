import paho.mqtt.client as mqtt
import time
import sys
import RPi.GPIO as GPIO
from time import sleep
import _thread


#import Adafruit_DHT
lock_msg = ' '
mqtt_topic = "lock"
#mqtt_broker_ip = "broker.emqx.io"
sto_aprendo = False
mqtt_broker_ip = "192.168.1.89"
apri = True
def apri():
    GPIO.setmode(GPIO.BOARD)

    # Set pin 11 as an output, and define as servo1 as PWM pin
    GPIO.setup(11,GPIO.OUT)
    servo1 = GPIO.PWM(11,50) # pin 11 for servo1, pulse 50Hz

    # Start PWM running, with value of 0 (pulse off)
    servo1.start(0)

    # Loop to allow user to set servo angle. Try/finally allows exit
    # with execution of servo.stop and GPIO cleanup :)
    print ("sto aprendo")
    servo1.ChangeDutyCycle(2+(120/18))
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    servo1.stop()
    GPIO.cleanup()

def chiudi():
    GPIO.setmode(GPIO.BOARD)

    # Set pin 11 as an output, and define as servo1 as PWM pin
    GPIO.setup(11,GPIO.OUT)
    servo1 = GPIO.PWM(11,50) # pin 11 for servo1, pulse 50Hz

    # Start PWM running, with value of 0 (pulse off)
    servo1.start(0)
    print ("sto chiudendo")
    servo1.ChangeDutyCycle(2+(50/18))
    time.sleep(0.5)
    servo1.ChangeDutyCycle(0)
    servo1.stop()
    GPIO.cleanup()
  
def gestisci_cancello(apri):
    global sto_aprendo
    if apri and not sto_aprendo:
    
        sto_aprendo = True
        apri()
        time.sleep(5)
        chiudi()
        sto_aprendo = False



# The callback for when the client receives a CONNACK response from the server.
def on_connect(client, userdata, flags, rc):
    print("Connected with result code "+str(rc))

    # Subscribing in on_connect() means that if we lose the connection and
    # reconnect then subscriptions will be renewed.
    client.subscribe(mqtt_topic)

# The callback for when a PUBLISH message is received from the server.
def on_message(client, userdata, msg):
    msg.payload = msg.payload.decode("utf-8")
    localtime = time.asctime( time.localtime(time.time()) )
    print(msg.topic+" "+msg.payload+" "+ localtime)
    if(msg.payload == "approved"):

        _thread.start_new_thread( gestisci_cancello, (apri,))
        
        
    else: 
        print ("sconosciuto")
    


client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.connect(mqtt_broker_ip, 1883)
client.loop_forever()
