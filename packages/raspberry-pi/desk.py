
import serial
import RPi.GPIO as GPIO
import sys
import time
import paho.mqtt.client as mqtt
import paho.mqtt.publish as publish
import asyncio
import config

broker = '192.168.2.2'
port = 1883
height_topic = "home-assistant/desk/height"
preset_topic = "home-assistant/desk/command"
client = None
# username = 'emqx'
# password = 'public'

SERIAL_PORT = "/dev/ttyS0" # GPIO14 (TX) and GPIO15 (RX)
PIN_20 = 12 # GPIO 12

SUPPORTED_COMMANDS = {
    "up": bytearray(b'\xa5\x00\x20\xdf\xff'),
    "down": bytearray(b'\xa5\x00\x40\xbf\xff'),
    "m": bytearray(b'\x9b\x06\x02\x20\x00\xac\xb8\x9d'),
    "wake_up": bytearray(b'\x9b\x06\x02\x00\x00\x6c\xa1\x9d'),
    "preset_1": bytearray(b'\xa5\x00\x02\xfd\xff'),
    "preset_2": bytearray(b'\xa5\x00\x04\xfb\xff'),
    "preset_3": bytearray(b'\x9b\x06\x02\x10\x00\xac\xac\x9d'),
    "preset_4": bytearray(b'\x9b\x06\x02\x00\x01\xac\x60\x9d'),
}

class LoctekMotion():
    is_showing_target_height = False
    previously_published_height = -1
    last_processed = 0

    def __init__(self, serial, mqtt_client):
        """Initialize LoctekMotion"""
        self.serial = serial
        self.mqtt_client = mqtt_client

        # Or GPIO.BOARD - GPIO Numbering vs Pin numbering
        GPIO.setmode(GPIO.BCM)

        # Turn desk in operating mode by setting controller pin20 to HIGH
        # This will allow us to send commands and to receive the current height

        # GPIO.output(pin_20, GPIO.HIGH)

    async def execute_command(self, command_name: str):
        """Execute command"""
        command = SUPPORTED_COMMANDS.get(command_name)

        if not command:
            raise Exception("Command not found")
        if command_name == "wake_up":
            print("WAKE UP")
            GPIO.setup(24, GPIO.OUT, initial=GPIO.LOW)
            await asyncio.sleep(0.5)
            GPIO.output(24, GPIO.HIGH)
        else:
            self.is_showing_target_height = True
            GPIO.setup(23, GPIO.OUT, initial=GPIO.LOW)
            await asyncio.sleep(0.05)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            await asyncio.sleep(0.05)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            self.serial.write(command)
            print("ran command")
            GPIO.output(23, GPIO.HIGH)
            await asyncio.sleep(2.2)
            self.is_showing_target_height = False

    def decode_seven_segment(self, byte):
        binaryByte = bin(byte).replace("0b","").zfill(8)
        decimal = False
        if binaryByte[0] == "1":
            decimal = True
        if binaryByte[1:] == "0111111":
            return 0, decimal
        if binaryByte[1:] == "0000110":
            return 1, decimal
        if binaryByte[1:] == "1011011":
            return 2, decimal
        if binaryByte[1:] == "1001111":
            return 3, decimal
        if binaryByte[1:] == "1100110":
            return 4, decimal
        if binaryByte[1:] == "1101101":
            return 5, decimal
        if binaryByte[1:] == "1111101":
            return 6, decimal
        if binaryByte[1:] == "0000111":
            return 7, decimal
        if binaryByte[1:] == "1111111":
            return 8, decimal
        if binaryByte[1:] == "1101111":
            return 9, decimal
        if binaryByte[1:] == "1000000":
            return 10, decimal
        return -1, decimal

    def current_height(self):
        history = [None] * 5
        while True:
            try:
                # read in each byte
                data = self.serial.read(1)
                if self.is_showing_target_height:
                    continue
                # 9b starts the data
                # the value after 9b has the length of the packet
                if history[2] == 0x5a:
                    if history[1] == 0:
                        print("height is empty                ", end='\r')
                    else:
                        height1, decimal1 = self.decode_seven_segment(history[1])
                        height1 = height1 * 100
                        height2, decimal2 = self.decode_seven_segment(history[0])
                        height2 = height2 * 10
                        height3, decimal3 = self.decode_seven_segment(data[0])
                        if height1 < 0 or height2 < 0 or height3 < 0:
                            print("Display Empty","          ",end='\r')
                        else:
                            finalHeight = height1 + height2 + height3
                            decimal = decimal1 or decimal2 or decimal3
                            if decimal == True:
                                finalHeight = finalHeight/10
                            print("Height:",finalHeight,"       ",end='\r')
                            if finalHeight != self.previously_published_height:
                                if time.time() - self.last_processed > 1:
                                    self.last_processed = time.time()
                                    self.previously_published_height = finalHeight
                                    self.mqtt_client.publish(height_topic, finalHeight)
                history[4] = history[3]
                history[3] = history[2]
                history[2] = history[1]
                history[1] = history[0]
                history[0] = data[0]
                    
            except Exception as e:
                print(e)
                break

def connect_mqtt():
    def on_connect(client, userdata, flags, rc):
        if rc == 0:
            print("Connected to MQTT Broker!")
            subscribe(client, preset_topic, locktek)
        else:
            print("Failed to connect, return code %d\n", rc)
    # Set Connecting Client ID
    client = mqtt.Client("ha-client")
    client.connect(broker)
    client.username_pw_set(config.mqtt_username, config.mqtt_password)
    client.on_connect = on_connect
    client.loop_start()
    return client

def subscribe(client, topic, locktek):
    if client == None: return
    if locktek == None: return

    def on_message(client, userdata, msg):
        print(f"Received `{msg.payload.decode()}` from `{msg.topic}` topic")

        if topic == preset_topic:
            asyncio.run(locktek.execute_command(msg.payload.decode()))


    client.subscribe(topic)
    client.on_message = on_message

def main():
    global client
    global locktek
    client = connect_mqtt()
    try:
        # command = sys.argv[1]
        ser = serial.Serial(SERIAL_PORT, 9600, timeout=500)
        locktek = LoctekMotion(ser, client)
        # locktek.execute_command(command)
        locktek.current_height()
    # Error handling for serial port
    except serial.SerialException as e:
        print(e)
        return
    # Error handling for command line arguments
    except IndexError:
        program = sys.argv[0]
        print("Usage: python3",program,"[COMMAND]")
        print("Supported Commands:")
        for command in SUPPORTED_COMMANDS:
            print("\t", command)
        sys.exit(1)
    except KeyboardInterrupt:
        sys.exit(1)
    finally:
        GPIO.cleanup()

if __name__ == "__main__":
    main()
