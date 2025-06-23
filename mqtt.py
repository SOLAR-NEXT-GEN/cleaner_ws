import paho.mqtt.client as mqtt
import json
import time

# MQTT broker settings
MQTT_BROKER = "localhost"  # Broker address (adjust if needed)
MQTT_PORT = 1883           # Default MQTT port
MQTT_TOPIC = "pingpong/primitive"  # Topic to publish to

# The message in the format of std_msgs/msg/String (simulated)
# The message is a dictionary that mimics the std_msgs/msg/String structure.
message = "wowzaa"

# Callback when connected to the MQTT broker
def on_connect(client, userdata, flags, rc):
    if rc == 0:
        print("Connected to MQTT broker")
    else:
        print(f"Failed to connect, return code {rc}")

# Callback when a message is published
def on_publish(client, userdata, mid):
    print(f"Message published with id: {mid}")

# Initialize the MQTT client
client = mqtt.Client()

# Assign callback functions
client.on_connect = on_connect
client.on_publish = on_publish

# Connect to the MQTT broker
client.connect(MQTT_BROKER, MQTT_PORT, 60)

# Start the network loop in a separate thread
client.loop_start()

# Wait until connected to broker
time.sleep(2)

# Publish the message (in JSON format)
try:
    message_json = json.dumps(message)  # Convert the Python dictionary to a JSON string
    print(f"Publishing message: {message_json}")
    client.publish(MQTT_TOPIC, payload=message, qos=0, retain=False)
    print(f"Message published to {MQTT_TOPIC}")
except Exception as e:
    print(f"Error publishing message: {e}")

# Give some time for the message to be sent
time.sleep(1)

# Disconnect the client
client.loop_stop()
client.disconnect()


# https://github.com/ika-rwth-aachen/mqtt_client/tree/main

# sudo apt update
# sudo apt install ros-$ROS_DISTRO-mqtt-client

# ros2 launch mqtt_client standalone.launch.ros2.xml params_file:=$(ros2 pkg prefix mqtt_client)/share/mqtt_client/config/params.ros2.primitive.yaml

# mosquitto_sub -h localhost -t pingpong/primitive
# mosquitto_pub -h localhost -t "pingpong/primitive" --repeat 20 --repeat-delay 1 -m 1

# ros2 topic echo /pong/primitive
