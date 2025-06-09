"""
extra notes:
if the target temperature is set to 0, the heater will always be off
errors must be sent through the status topic with two keys: status and message. status must be 'error' and message must be a string describing the error
the initial request topic is used to send the initial temperature and target temperature to the web client
the target temperature must be stored in flash memory on the ESP to persist between reboots
the temperature must be read from actual temperature sensors on the ESP
the heater must be turned on and off using the actual GPIO pins on the ESP
the heater must be turned on when the current temperature is below the target temperature
the heater must be turned off when the current temperature is equal to or above the target temperature
the heater_min_switch_time variable must be changed from 0 to 60.
all variables should be camelCase instead of snake_case where applicable
the encrypt and decrypt functions must be implemented using the same AES encryption as this script to communicate with the web client
"""

raise Exception("Dummy script is outdated")

import paho.mqtt.client as mqtt
import json
import time
from Crypto.Cipher import AES
from Crypto.Util.Padding import pad
import base64
import os

CONFIG = {
    'device_id': 'jcz_001',
    'secret_key': 'A',  # Must match the web client
    'topic_prefix': 'secure_jacuzzi/jcz_001/',
    'broker': 'test.mosquitto.org',
    'port': 1883
}

TOPICS = {
    'temperature': CONFIG['topic_prefix'] + 'temperature',
    'status': CONFIG['topic_prefix'] + 'status',
    'target_temp': CONFIG['topic_prefix'] + 'target_temperature',
    'initial_request': CONFIG['topic_prefix'] + 'initial_request'
}

MIN_TEMP = 0
MAX_TEMP = 50

HEATER_PIN = 0  # replace with actual GPIO pin number
TARGET_TEMP_ADDR = 0

heater_min_switch_time = 0

flash_memory = { # this is only for the simulation script. In actual implementation, use flash memory on the ESP
    0: 37.0
} 

def write_to_flash(address, value):
    # replace with actual code to write the target temperature to flash
    flash_memory[address] = value

def read_from_flash(address):
    # replace with actual code to read the last target temperature from flash
    return flash_memory.get(address)

def read_temperature():
    # replace with actual reading from sensors on the ESP
    return current_temp

last_target_update_timestamp = 0
heater_enabled = False
heater_switch_time = 0

current_temp = 37.0 # for simulation, because we don't have actual temperature sensors
current_temp = read_temperature()
target_temp = read_from_flash(TARGET_TEMP_ADDR)
if target_temp > MAX_TEMP or target_temp < MIN_TEMP:
    print("Invalid target temperature in flash. Setting to minimum temperature")
    target_temp = MIN_TEMP

def encrypt_message(message):
    """Encrypt message using AES"""
    # Ensure key is 32 bytes
    key = CONFIG['secret_key'].encode('utf-8').ljust(32, b'\0')
    
    # Generate IV
    iv = os.urandom(16)
    
    message_str = json.dumps(message)
    
    # Create cipher and encrypt
    cipher = AES.new(key, AES.MODE_CBC, iv)
    ct_bytes = cipher.encrypt(pad(message_str.encode('utf-8'), AES.block_size))
    
    # Create result object matching JavaScript format
    result = {
        'iv': base64.b64encode(iv).decode('utf-8'),
        'ciphertext': base64.b64encode(ct_bytes).decode('utf-8')
    }
    
    return json.dumps(result)

def decrypt_message(encrypted_data):
    """Decrypt message using AES"""
    try:
        # Parse the encrypted data
        data = json.loads(encrypted_data)
        
        # Ensure key is 32 bytes
        key = CONFIG['secret_key'].encode('utf-8').ljust(32, b'\0')
        
        # Get IV and ciphertext
        iv = base64.b64decode(data['iv'])
        ciphertext = base64.b64decode(data['ciphertext'])
        
        # Create cipher and decrypt
        cipher = AES.new(key, AES.MODE_CBC, iv)
        padded_data = cipher.decrypt(ciphertext)
        
        # Remove padding
        padding_length = padded_data[-1]
        unpadded_data = padded_data[:-padding_length]
        
        return json.loads(unpadded_data.decode('utf-8'))
    except Exception as e:
        print(f"Decryption error: {e}")
        print(f"Received data: {encrypted_data}")
        return None

def on_publish(client, userdata, mid):
    print(f"Message published successfully")

def on_connect(client, userdata, flags, rc):
    print(f"Connected with result code {rc}")
    client.subscribe(TOPICS['target_temp'])
    client.subscribe(TOPICS['initial_request'])

def on_message(client, userdata, msg):
    global last_target_update_timestamp, target_temp
    try:
        decrypted = decrypt_message(msg.payload)
        print("Received message: ", decrypted)
        
        if msg.topic == TOPICS['initial_request']:
            message = {
                'deviceId': CONFIG['device_id'],
                'value': current_temp,
                'target': target_temp,
            }
            encrypted_message = encrypt_message(message)
            client.publish(TOPICS['temperature'], encrypted_message)
            return
            
        if msg.topic == TOPICS['target_temp']:
            message_timestamp = decrypted.get('timestamp', 0)
            if message_timestamp < last_target_update_timestamp:
                print(f"Ignoring outdated target temperature request (timestamp: {message_timestamp})")
                return

            new_target = float(decrypted['value'])
            if MIN_TEMP <= new_target <= MAX_TEMP:
                write_to_flash(TARGET_TEMP_ADDR, target_temp)
                target_temp = new_target
                last_target_update_timestamp = message_timestamp
                status_message = {
                    'deviceId': CONFIG['device_id'],
                    'type': 'target_temp_update',
                    'status': 'ok',
                    'message': new_target,
                }
            else:
                status_message = {
                    'deviceId': CONFIG['device_id'],
                    'type': 'target_temp_update',
                    'status': 'error',
                    'message': f'Temperature out of valid range ({MIN_TEMP}-{MAX_TEMP}°C)',
                }
            
            client.publish(TOPICS['status'], encrypt_message(status_message))
    except Exception as e:
        print(f"Error processing message: {e}")

def enable_heater():
    global heater_enabled, heater_switch_time
    print("Turning on heater")
    heater_enabled = True
    heater_switch_time = heater_min_switch_time
    # add actual code to turn on the heater on HEATER_PIN

def disable_heater():
    global heater_enabled, heater_switch_time
    print("Turning off heater")
    heater_enabled = False
    heater_switch_time = heater_min_switch_time
    # add actual code to turn off the heater on HEATER_PIN

client = mqtt.Client()
client.on_connect = on_connect
client.on_message = on_message
client.on_publish = on_publish

print(f"Connecting to {CONFIG['broker']}...")
client.connect(CONFIG['broker'], CONFIG['port'], 60)

client.loop_start()

while True:
    current_temp = read_temperature()
    if current_temp <= target_temp and not heater_enabled and heater_switch_time <= 0 and target_temp != 0:
        enable_heater()
    elif current_temp > target_temp and heater_enabled and heater_switch_time <= 0:
        disable_heater()
    elif heater_switch_time > 0:
        heater_switch_time -= 1
        print(f"Heater: {'on' if heater_enabled else 'off'}. Waiting for heater to switch ({heater_switch_time}s)")

    message = {
        'deviceId': CONFIG['device_id'],
        'value': current_temp,
        'target': target_temp,
        'isHeating': heater_enabled,
    }
    
    encrypted_message = encrypt_message(message)
    client.publish(TOPICS['temperature'], encrypted_message)
    
    print(f"Current: {current_temp}°C, Target: {target_temp}°C")
    time.sleep(1)

    # simulate temperature changes. not needed in actual implementation
    change_factor = 0.1
    if heater_enabled:
        current_temp += change_factor
    else:
        current_temp -= change_factor