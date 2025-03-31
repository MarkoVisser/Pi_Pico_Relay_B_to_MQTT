import rp2
import network
import machine
import ubinascii
import time
import neopixel
from secrets import secrets
from umqtt.simple import MQTTClient
from machine import Pin, PWM

# Configuration constants
CONFIG = {
    'KA_THRESHOLD': 30,  # Keepalive interval (10 = 1 second)
    'WIFI_TIMEOUT': 15,  # Seconds to wait for WiFi connection
    'DEBOUNCE_MS': 200,  # Switch debounce time in milliseconds
    'LED_COLORS': {
        'BOOT': (50, 0, 50),    # Pink
        'ERROR': (20, 0, 0),    # Red
        'NORMAL': (0, 10, 0),   # Green
        'MQTT_RX': (0, 0, 20),  # Blue
        'STATUS': (30, 30, 30)  # White
    }
}

# Relay and switch pin assignments (DO NOT CHANGE RELAY PINS)
relays = {
    1: {"relay": Pin(21, Pin.OUT), "switch": Pin(12, Pin.IN), "last_state": None, "last_change": 0},
    2: {"relay": Pin(20, Pin.OUT), "switch": Pin(11, Pin.IN), "last_state": None, "last_change": 0},
    3: {"relay": Pin(19, Pin.OUT), "switch": Pin(10, Pin.IN), "last_state": None, "last_change": 0},
    4: {"relay": Pin(18, Pin.OUT), "switch": Pin(9, Pin.IN), "last_state": None, "last_change": 0},
    5: {"relay": Pin(17, Pin.OUT), "switch": Pin(8, Pin.IN), "last_state": None, "last_change": 0},
    6: {"relay": Pin(16, Pin.OUT), "switch": Pin(7, Pin.IN), "last_state": None, "last_change": 0},
    7: {"relay": Pin(15, Pin.OUT), "switch": Pin(5, Pin.IN), "last_state": None, "last_change": 0},
    8: {"relay": Pin(14, Pin.OUT), "switch": Pin(4, Pin.IN), "last_state": None, "last_change": 0}
}

# Initialize hardware
pwm = PWM(Pin(6))
pwm.freq(1000)
np = neopixel.NeoPixel(machine.Pin(13), 1)
buz = Pin(6, Pin.OUT)

# MQTT and device settings
MQTT_BROKER = secrets["MQTT_BROKER"]
MQTT_USER = secrets["MQTT_USER"]
MQTT_PWD = secrets["MQTT_PWD"]
MQTT_PORT = secrets["MQTT_PORT"]
MQTT_DEVICE_NAME = secrets["MQTT_DEVICE_NAME"]
MQTT_DEVICE_ID = "0x00" + ubinascii.hexlify(machine.unique_id()).decode()
MQTT_BASE = MQTT_DEVICE_NAME + "/"
MQTT_COMMAND_TOPIC = MQTT_BASE + "command/relay/#"
MQTT_STATUS_TOPIC = MQTT_BASE + "status"
MQTT_DISC_TOPIC = "homeassistant/switch/" + MQTT_DEVICE_ID

DEV_INFO = {
    "MANUFACTURER": "Waveshare",
    "MODEL": "Pico Relay B"
}

WLAN_SSID = secrets["WLAN_SSID"]
WLAN_PWD = secrets["WLAN_PWD"]

wlan = network.WLAN(network.STA_IF)
mqtt_client = None

def set_led(state):
    """Set LED color based on state"""
    np[0] = CONFIG['LED_COLORS'][state]
    np.write()

def activate_wlan():
    """Connect to WiFi with timeout"""
    set_led('BOOT')
    wlan.active(True)
    wlan.config(pm=0xa11140)  # Disable power-save mode
    wlan.connect(WLAN_SSID, WLAN_PWD)
    
    start_time = time.time()
    while time.time() - start_time < CONFIG['WIFI_TIMEOUT']:
        if wlan.status() == network.STAT_GOT_IP:
            print(f'Connected to {WLAN_SSID}, IP: {wlan.ifconfig()[0]}')
            set_led('NORMAL')
            return True
        time.sleep(0.5)
    
    set_led('ERROR')
    raise RuntimeError(f'Failed to connect to {WLAN_SSID} within {CONFIG["WIFI_TIMEOUT"]} seconds')

def msg_in(topic, msg):
    """Handle incoming MQTT messages with validation"""
    set_led('MQTT_RX')
    try:
        # Validate topic and extract relay number
        relay_num = int(topic.decode().split('/')[-1])
        if relay_num not in relays:
            print(f"Invalid relay number: {relay_num}")
            return

        # Validate message
        mode = int(msg.decode())
        if mode not in (0, 1):
            print(f"Invalid command value: {mode}")
            return

        state = "ON" if mode == 1 else "OFF"
        print(f"Channel {relay_num}: {state}")
        relays[relay_num]["relay"].value(mode)
        
    except ValueError as e:
        print(f"Message parsing error: {e}")
    except Exception as e:
        print(f"Unexpected error in msg_in: {e}")
    finally:
        set_led('NORMAL')

def setup_mqtt():
    """Initialize MQTT connection"""
    client = MQTTClient(
        MQTT_DEVICE_NAME,
        MQTT_BROKER,
        port=MQTT_PORT,
        user=MQTT_USER,
        password=MQTT_PWD,
        keepalive=10,
        ssl=False
    )
    client.set_callback(msg_in)
    client.set_last_will(MQTT_STATUS_TOPIC, "offline", qos=0, retain=False)
    client.connect()
    client.subscribe(MQTT_COMMAND_TOPIC)
    print('MQTT connection successful!')
    return client

def check_switches():
    """Check physical switches with debouncing"""
    current_time = time.ticks_ms()
    for relay_num, relay in relays.items():
        switch_state = relay["switch"].value()
        if time.ticks_diff(current_time, relay["last_change"]) > CONFIG['DEBOUNCE_MS']:
            if switch_state != relay["last_state"]:
                relay["relay"].value(switch_state)
                relay["last_state"] = switch_state
                relay["last_change"] = current_time
                mqtt_client.publish(f"{MQTT_STATUS_TOPIC}/relay/{relay_num}", str(switch_state))

def update_relay_states():
    """Update relay states and publish changes"""
    for i in range(1, 9):
        current_state = relays[i]["relay"].value()
        if relays[i]["last_state"] is None:
            relays[i]["last_state"] = current_state
            mqtt_client.publish(f"{MQTT_STATUS_TOPIC}/relay/{i}", str(current_state))
        elif current_state != relays[i]["last_state"]:
            mqtt_client.publish(f"{MQTT_STATUS_TOPIC}/relay/{i}", str(current_state))
            relays[i]["last_state"] = current_state

def update_status():
    """Publish online status"""
    mqtt_client.publish(MQTT_STATUS_TOPIC, "online")
    set_led('STATUS')
    time.sleep(0.1)
    set_led('NORMAL')

def cleanup():
    """Clean up resources"""
    if mqtt_client:
        mqtt_client.disconnect()
    wlan.active(False)
    set_led('ERROR')

def main():
    global mqtt_client
    ka_count = 0
    
    try:
        # Initial setup
        rp2.country('GB')
        activate_wlan()
        mqtt_client = setup_mqtt()
        
        # Publish Home Assistant discovery
        for i in range(1, 9):
            config_msg = (
                f'{{"availability": [{{"topic": "{MQTT_STATUS_TOPIC}"}}],'
                f'"command_topic": "{MQTT_DEVICE_NAME}/command/relay/{i}",'
                f'"device": {{"identifiers": ["{MQTT_DEVICE_NAME}"], "manufacturer": "{DEV_INFO["MANUFACTURER"]}",'
                f'"model": "{DEV_INFO["MODEL"]}", "name": "{MQTT_DEVICE_NAME}"}},'
                f'"name": "{MQTT_DEVICE_NAME}_ch_{i}", "payload_off": 0, "payload_on": 1,'
                f'"state_topic": "{MQTT_DEVICE_NAME}/status/relay/{i}",'
                f'"unique_id": "{MQTT_DEVICE_ID}_relay_{i}_pico"}}'
            )
            mqtt_client.publish(f"{MQTT_DISC_TOPIC}/switch_{i}/config", config_msg, retain=True)
        
        update_status()
        
        # Main loop
        while True:
            mqtt_client.check_msg()
            check_switches()
            update_relay_states()
            
            ka_count += 1
            if ka_count >= CONFIG['KA_THRESHOLD']:
                update_status()
                ka_count = 0
                
            time.sleep(0.1)
            
    except Exception as e:
        print(f"Fatal error: {e}")
        cleanup()
        time.sleep(5)
        machine.reset()

if __name__ == "__main__":
    main()
