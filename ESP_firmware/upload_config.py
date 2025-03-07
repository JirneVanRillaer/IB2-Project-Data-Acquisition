from dotenv import load_dotenv
from os import environ
from sys import exit
from argparse import ArgumentParser, Namespace
from struct import pack
import esptool
import tempfile

PORT = "COM8"
BAUDRATE = 115200
EEPROM_ADDR = 0x3FB000  # For default (Arduino core) partition sheme
EEPROM_SIZE = 4096      # 4KB EEPROM

def calcCRC(data:bytes) -> int:
    crc_sum = sum(data)
    crc  = crc_sum & 0xff
    crc += int(crc_sum / 0x0100)
    crc = 0xff - crc
    return crc

def create_image(args:Namespace) -> bytes:
    print('using SSID =', args.ssid)
    print(f'using MQTT = {args.mqtt_server}:{args.mqtt_port}')

    """
    SSID: char[35]
    PSK: char[35]
    MQTT_server: char[35]
    MQTT_port: u16
    MQTT_topic: char[15]
    Classroom: char[5]
    CRC: u8
    """

    image = pack('<35s35s35sH15s5s', args.ssid.encode(), args.psk.encode(), args.mqtt_server.encode(), 
        args.mqtt_port, args.mqtt_topic.encode(), args.classroom.encode())
    crc = calcCRC(image)
    image += crc.to_bytes(1)    # Append CRC-byte
    image += bytes([0xff] * (EEPROM_SIZE - len(image))) # Pad with 0xff until EEPROM_SIZE is reached
    return image

def esp_upload(image_path:str) -> None:
    args = [
        "--port", PORT,
        "--baud", str(BAUDRATE),
        "write_flash",
        hex(EEPROM_ADDR), image_path
    ]
    esptool.main(args)

def main() -> None:
    # Parse cmdline arguments
    load_dotenv()

    argp = ArgumentParser(
        prog='Configuration Uploader', 
        description='Utility used to upload configuration and credentials to the ESP8266\'s EEPROM',   
    )
    argp.add_argument('--esp-port', type=str, required=True, help='Serial port on which ESP is listening, eg COM6 or /dev/ttyUSB0')
    argp.add_argument('--classroom', type=str, required=True, help='Classroom in which the node will be placed, eg E117')
    argp.add_argument('--ssid', type=str, required=False, default=environ['SSID'], help='network name the ESP should connect to')
    argp.add_argument('--psk', type=str, required=False, default=environ['PSK'], help='password of the network the ESP should connect to')
    argp.add_argument('--mqtt-server', type=str, required=False, default=environ['MQTT_SERVER'], help='Address of the MQTT server, IPv4 or hostname')
    argp.add_argument('--mqtt-port', type=int, required=False, default=environ['MQTT_PORT'], help='Port on which the MQTT server is listening')
    argp.add_argument('--mqtt-topic', type=str, required=False, default=environ['MQTT_TOPIC'], help='MQTT server\'s topic')
    args = argp.parse_args()

    # Create image
    print('Creating EEPROM image...')
    image = create_image(args)
    with tempfile.NamedTemporaryFile(delete_on_close=False) as fp:
        fp.write(image)
        fp.close()

        # Upload to ESP
        print('Starting uploader...')
        try:
            esp_upload(fp.name)
        except esptool.util.FatalError as ex:
            print('Failed to upload data to ESP:', ex)
        else:
            print('Done. Goodbye!')

if __name__ == '__main__':
    main()