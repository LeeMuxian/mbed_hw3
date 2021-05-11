import paho.mqtt.client as paho
import serial
import time

serdev = '/dev/ttyACM0'
s = serial.Serial(serdev, 9600)


# https://os.mbed.com/teams/mqtt/wiki/Using-MQTT#python-client

# MQTT broker hosted on local machine
mqttc = paho.Client()

# Settings for connection
# TODO: revise host to your IP
host = "192.168.43.211"
topic = "Mbed"
topic1 = "Mbed/Threshold"
topic2 = "Mbed/angle_detected"

# Callbacks
def on_connect(self, mosq, obj, rc):
    print("Connected rc: " + str(rc))

num_tilt = 0
def on_message(mosq, obj, msg):
    global num_tilt
    if len(str(msg.payload)) < 40:
        num_tilt = 0
        print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
        s.write(bytes("\r", 'UTF-8'))
        time.sleep(1)
        s.write(bytes("/mode_stop/run\r\n", 'UTF-8'))
        time.sleep(2)
    elif len(str(msg.payload)) >= 40 :
        if num_tilt == 9 :
            print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
            num_tilt = 0
            s.write(bytes("\r", 'UTF-8'))
            time.sleep(1)
            s.write(bytes("/mode_stop/run\r\n", 'UTF-8'))
            time.sleep(2)
        else :
            print("[Received] Topic: " + msg.topic + ", Message: " + str(msg.payload) + "\n")
            num_tilt = num_tilt + 1

def on_subscribe(mosq, obj, mid, granted_qos):
    print("Subscribed OK")

def on_unsubscribe(mosq, obj, mid, granted_qos):
    print("Unsubscribed OK")

# Set callbacks
mqttc.on_message = on_message
mqttc.on_connect = on_connect
mqttc.on_subscribe = on_subscribe
mqttc.on_unsubscribe = on_unsubscribe

# Connect and subscribe
print("Connecting to " + host + "/" + topic1 + " and " + topic2)
mqttc.connect(host, port=1883, keepalive=60)
mqttc.subscribe(topic1, 0)
mqttc.subscribe(topic2, 0)

s.write(bytes("/gesture_UI/run\r\n", 'UTF-8'))
s.write(bytes("/angle_det/run\r\n", 'UTF-8'))

# Publish messages from Python
num = 0
while num != 5:
    ret = mqttc.publish(topic1, "Message from Python for topic1 !\n", qos=0)
    print(num)
    if (ret[0] != 0):
            print("Publish failed")
    mqttc.loop()
    time.sleep(1.5)
    num += 1

# Loop forever, receiving messages
mqttc.loop_forever()